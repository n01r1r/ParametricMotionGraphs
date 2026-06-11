#include "pmg/MotionRegistration.h"

#include "pmg/MotionDistance.h"
#include "pmg/ParameterVector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace pmg {

namespace {

// Monotone mapping from reference frame row -> mean matched example frame,
// extracted from the optimal DTW path between two cloud sequences. Indices are
// relative to the sequences (row 0 = first reference cloud). Endpoints are
// pinned by the DTW boundary condition.
std::vector<float> DtwMeanExampleFrames(
    const std::vector<PointCloud>& reference_clouds,
    const std::vector<PointCloud>& example_clouds) {
    const int rows = static_cast<int>(reference_clouds.size());
    const int cols = static_cast<int>(example_clouds.size());

    std::vector<float> cost(static_cast<std::size_t>(rows) * cols, 0.0f);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            cost[row * cols + col] = MotionDistance::AlignedPointCloudDistance(
                                         reference_clouds[row], example_clouds[col])
                                         .distance;
        }
    }

    constexpr float kInfinity = std::numeric_limits<float>::infinity();
    std::vector<float> accumulated(cost.size(), kInfinity);
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const float best_previous =
                (row == 0 && col == 0)
                    ? 0.0f
                    : std::min({row > 0 ? accumulated[(row - 1) * cols + col] : kInfinity,
                                col > 0 ? accumulated[row * cols + col - 1] : kInfinity,
                                row > 0 && col > 0
                                    ? accumulated[(row - 1) * cols + col - 1]
                                    : kInfinity});
            accumulated[row * cols + col] = cost[row * cols + col] + best_previous;
        }
    }

    // Backtrack; a reference row matched to several example columns collapses
    // to their mean so the per-row mapping stays single-valued.
    std::vector<float> column_sum(rows, 0.0f);
    std::vector<int> column_count(rows, 0);
    int row = rows - 1;
    int col = cols - 1;
    while (true) {
        column_sum[row] += static_cast<float>(col);
        ++column_count[row];
        if (row == 0 && col == 0) {
            break;
        }
        const float up = row > 0 ? accumulated[(row - 1) * cols + col] : kInfinity;
        const float left = col > 0 ? accumulated[row * cols + col - 1] : kInfinity;
        const float diagonal =
            row > 0 && col > 0 ? accumulated[(row - 1) * cols + col - 1] : kInfinity;
        if (diagonal <= up && diagonal <= left) {
            --row;
            --col;
        } else if (up <= left) {
            --row;
        } else {
            --col;
        }
    }

    std::vector<float> mean_example_frames(rows, 0.0f);
    for (int reference_row = 0; reference_row < rows; ++reference_row) {
        mean_example_frames[reference_row] =
            column_sum[reference_row] / static_cast<float>(column_count[reference_row]);
    }
    // The path is monotone but per-row means can locally tie; enforce
    // non-decreasing so phase interpolation below stays well defined.
    for (int reference_row = 1; reference_row < rows; ++reference_row) {
        mean_example_frames[reference_row] = std::max(
            mean_example_frames[reference_row], mean_example_frames[reference_row - 1]);
    }
    return mean_example_frames;
}

float InterpolateAt(const std::vector<float>& values, float position) {
    if (values.empty()) {
        return 0.0f;
    }
    position = std::clamp(position, 0.0f, static_cast<float>(values.size() - 1));
    const int low = static_cast<int>(position);
    const int high = std::min(low + 1, static_cast<int>(values.size()) - 1);
    const float alpha = position - static_cast<float>(low);
    return values[low] + alpha * (values[high] - values[low]);
}

int PhaseToFrame(float phase, int frame_count) {
    return static_cast<int>(
        std::lround(phase * static_cast<float>(frame_count - 1)));
}

}  // namespace

std::vector<float> ContactAnchorPhases(
    const std::vector<ContactInterval>& intervals,
    int clip_frame_count) {
    std::vector<float> anchors;
    anchors.reserve(intervals.size() * 2);

    for (const ContactInterval& interval : intervals) {
        anchors.push_back(interval.StrikePhase(clip_frame_count));
        anchors.push_back(interval.LiftPhase(clip_frame_count));
    }

    std::sort(anchors.begin(), anchors.end());
    anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
    anchors.erase(
        std::remove_if(anchors.begin(), anchors.end(),
                       [](float phase) { return phase <= 0.0f || phase >= 1.0f; }),
        anchors.end());
    return anchors;
}

std::vector<TimeWarp> BuildRegistrationWarps(
    const std::vector<std::vector<float>>& example_anchor_phases) {
    if (example_anchor_phases.empty()) {
        throw std::runtime_error("BuildRegistrationWarps: no examples given");
    }

    const std::size_t anchor_count = example_anchor_phases.front().size();
    for (std::size_t example_index = 0; example_index < example_anchor_phases.size();
         ++example_index) {
        if (example_anchor_phases[example_index].size() != anchor_count) {
            throw std::runtime_error(
                "BuildRegistrationWarps: example " + std::to_string(example_index) +
                " has " + std::to_string(example_anchor_phases[example_index].size()) +
                " anchors, expected " + std::to_string(anchor_count) +
                "; examples must share the same contact structure");
        }
    }

    // Canonical domain: each anchor sits at the mean of the examples' phases.
    std::vector<float> canonical_anchors(anchor_count, 0.0f);
    for (const std::vector<float>& anchors : example_anchor_phases) {
        for (std::size_t anchor_index = 0; anchor_index < anchor_count; ++anchor_index) {
            canonical_anchors[anchor_index] += anchors[anchor_index];
        }
    }
    for (float& canonical : canonical_anchors) {
        canonical /= static_cast<float>(example_anchor_phases.size());
    }

    std::vector<TimeWarp> warps;
    warps.reserve(example_anchor_phases.size());
    for (const std::vector<float>& anchors : example_anchor_phases) {
        warps.push_back(TimeWarp::FromAnchors(canonical_anchors, anchors));
    }
    return warps;
}

void RegisterSpaceByContacts(
    ParametricMotionSpace& space,
    const Skeleton& skeleton,
    const std::vector<int>& contact_joints,
    const ContactDetectionSettings& settings) {
    if (space.NumExamples() == 0) {
        throw std::runtime_error("RegisterSpaceByContacts: space has no examples");
    }

    std::vector<std::vector<float>> example_anchor_phases;
    example_anchor_phases.reserve(space.Examples().size());

    for (const ExampleMotion& example : space.Examples()) {
        const std::vector<ContactInterval> intervals =
            DetectContacts(skeleton, example.clip, contact_joints, settings);
        example_anchor_phases.push_back(
            ContactAnchorPhases(intervals, example.clip.NumFrames()));
    }

    space.SetExampleTimeWarps(BuildRegistrationWarps(example_anchor_phases));
}

void RefineRegistrationByDtw(
    ParametricMotionSpace& space,
    const Skeleton& skeleton,
    const DtwRefineSettings& settings) {
    if (!space.HasExampleTimeWarps()) {
        throw std::runtime_error(
            "RefineRegistrationByDtw: space has no registration warps; run "
            "RegisterSpaceByContacts first");
    }
    const std::vector<ExampleMotion>& examples = space.Examples();
    const std::size_t example_count = examples.size();
    if (example_count < 2 || settings.max_knots_per_segment <= 0) {
        return;
    }

    const std::vector<TimeWarp>& warps = space.ExampleTimeWarps();

    // Canonical segment boundaries: pinned endpoints plus the shared contact
    // anchors (identical across warps by construction).
    std::vector<float> canonical_knots = warps.front().InteriorFromPhases();
    canonical_knots.insert(canonical_knots.begin(), 0.0f);
    canonical_knots.push_back(1.0f);

    // Timing reference: the example nearest the parameter centroid, so warps
    // re-time everything toward the middle of the space rather than an extreme.
    ParameterVector centroid(examples.front().parameter.size(), 0.0f);
    for (const ExampleMotion& example : examples) {
        for (std::size_t dim = 0; dim < centroid.size(); ++dim) {
            centroid[dim] += example.parameter[dim];
        }
    }
    for (float& value : centroid) {
        value /= static_cast<float>(example_count);
    }
    std::size_t reference_index = 0;
    float best_distance = std::numeric_limits<float>::infinity();
    for (std::size_t example_index = 0; example_index < example_count; ++example_index) {
        const float distance = Distance(examples[example_index].parameter, centroid);
        if (distance < best_distance) {
            best_distance = distance;
            reference_index = example_index;
        }
    }

    std::vector<std::vector<PointCloud>> clouds(example_count);
    for (std::size_t example_index = 0; example_index < example_count; ++example_index) {
        const MotionClip& clip = examples[example_index].clip;
        clouds[example_index].reserve(static_cast<std::size_t>(clip.NumFrames()));
        for (int frame = 0; frame < clip.NumFrames(); ++frame) {
            clouds[example_index].push_back(MotionDistance::BuildPointCloud(
                skeleton, clip, frame, settings.window_size));
        }
    }

    constexpr float kMinKnotSpacing = 1.0e-3f;
    const int reference_frame_count = examples[reference_index].clip.NumFrames();

    std::vector<TimeWarp> refined_warps(warps);
    for (std::size_t example_index = 0; example_index < example_count; ++example_index) {
        if (example_index == reference_index) {
            continue;  // the reference keeps its contact-anchor warp
        }
        const int example_frame_count = examples[example_index].clip.NumFrames();

        std::vector<float> refined_from;
        std::vector<float> refined_to;
        for (std::size_t segment = 0; segment + 1 < canonical_knots.size(); ++segment) {
            const float canonical_start = canonical_knots[segment];
            const float canonical_end = canonical_knots[segment + 1];

            const int reference_first = PhaseToFrame(
                warps[reference_index].Evaluate(canonical_start), reference_frame_count);
            const int reference_last = PhaseToFrame(
                warps[reference_index].Evaluate(canonical_end), reference_frame_count);
            const int example_first = PhaseToFrame(
                warps[example_index].Evaluate(canonical_start), example_frame_count);
            const int example_last = PhaseToFrame(
                warps[example_index].Evaluate(canonical_end), example_frame_count);

            const float example_phase_start = warps[example_index].Evaluate(canonical_start);
            const float example_phase_end = warps[example_index].Evaluate(canonical_end);

            // Segments too short to hold an interior correspondence stay linear.
            if (reference_last - reference_first >= 2 && example_last - example_first >= 2) {
                const std::vector<PointCloud> reference_clouds(
                    clouds[reference_index].begin() + reference_first,
                    clouds[reference_index].begin() + reference_last + 1);
                const std::vector<PointCloud> example_clouds(
                    clouds[example_index].begin() + example_first,
                    clouds[example_index].begin() + example_last + 1);
                const std::vector<float> mean_example_frames =
                    DtwMeanExampleFrames(reference_clouds, example_clouds);

                for (int knot = 1; knot <= settings.max_knots_per_segment; ++knot) {
                    const float alpha =
                        static_cast<float>(knot) /
                        static_cast<float>(settings.max_knots_per_segment + 1);
                    const float canonical_phase =
                        canonical_start + alpha * (canonical_end - canonical_start);
                    const float reference_row =
                        warps[reference_index].Evaluate(canonical_phase) *
                            static_cast<float>(reference_frame_count - 1) -
                        static_cast<float>(reference_first);
                    const float example_frame =
                        static_cast<float>(example_first) +
                        InterpolateAt(mean_example_frames, reference_row);
                    const float example_phase =
                        example_frame / static_cast<float>(example_frame_count - 1);

                    // Keep DTW knots strictly inside the segment on both axes
                    // so the contact anchors that bound it are never displaced.
                    const float previous_to =
                        refined_to.empty() ? example_phase_start : refined_to.back();
                    if (example_phase > previous_to + kMinKnotSpacing &&
                        example_phase < example_phase_end - kMinKnotSpacing) {
                        refined_from.push_back(canonical_phase);
                        refined_to.push_back(example_phase);
                    }
                }
            }

            // Preserve the original contact anchor that closes this segment.
            if (segment + 2 < canonical_knots.size()) {
                refined_from.push_back(canonical_end);
                refined_to.push_back(example_phase_end);
            }
        }

        refined_warps[example_index] = TimeWarp::FromAnchors(refined_from, refined_to);
    }

    space.SetExampleTimeWarps(std::move(refined_warps));
}

MotionClip ExtractFirstCycle(
    const Skeleton& skeleton,
    const MotionClip& clip,
    int cycle_joint,
    const ContactDetectionSettings& settings) {
    const std::vector<ContactInterval> intervals =
        DetectContacts(skeleton, clip, {cycle_joint}, settings);
    if (intervals.size() < 2) {
        throw std::runtime_error(
            "ExtractFirstCycle: need at least two contacts of the cycle joint, found " +
            std::to_string(intervals.size()));
    }

    const int cycle_start = intervals[0].first_frame;
    const int cycle_end = intervals[1].first_frame;  // next strike closes the cycle

    MotionClip cycle;
    cycle.name = clip.name + "_cycle";
    cycle.frames_per_second = clip.frames_per_second;
    cycle.frames.assign(clip.frames.begin() + cycle_start,
                        clip.frames.begin() + cycle_end + 1);
    return cycle;
}

}  // namespace pmg
