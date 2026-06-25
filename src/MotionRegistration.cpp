#include "pmg/MotionRegistration.h"

#include "pmg/MotionDistance.h"
#include "pmg/ParameterVector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace pmg {

namespace {

// Monotone mapping from reference frame row -> matched example frame column,
// extracted from a slope-constrained DTW path between two cloud sequences
// (steps (1,1), (1,2), (2,1); local slope in [1/2, 2]). Unconstrained DTW
// can hug the pinned endpoints with flat runs, fabricating timing jumps that
// degrade the blend; the slope limit forbids flat runs entirely. Indices are
// relative to the sequences (row 0 = first reference cloud). Returns empty
// when the sequence lengths are too dissimilar for any valid path; callers
// keep the linear warp for that segment.
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
    constexpr int kSteps[3][2] = {{1, 1}, {1, 2}, {2, 1}};  // (d_row, d_col)
    std::vector<float> accumulated(cost.size(), kInfinity);
    accumulated[0] = cost[0];
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (row == 0 && col == 0) {
                continue;
            }
            float best_previous = kInfinity;
            for (const auto& step : kSteps) {
                const int previous_row = row - step[0];
                const int previous_col = col - step[1];
                if (previous_row >= 0 && previous_col >= 0) {
                    best_previous = std::min(
                        best_previous, accumulated[previous_row * cols + previous_col]);
                }
            }
            if (best_previous < kInfinity) {
                accumulated[row * cols + col] = cost[row * cols + col] + best_previous;
            }
        }
    }
    if (!(accumulated[(rows - 1) * cols + cols - 1] < kInfinity)) {
        return {};  // length ratio outside [1/2, 2]; no valid path
    }

    // Backtrack the path nodes; rows strictly increase along the path, so the
    // per-row mapping is single-valued after interpolating skipped rows.
    std::vector<int> path_rows;
    std::vector<int> path_cols;
    int row = rows - 1;
    int col = cols - 1;
    while (true) {
        path_rows.push_back(row);
        path_cols.push_back(col);
        if (row == 0 && col == 0) {
            break;
        }
        float best_previous = kInfinity;
        int best_row = -1;
        int best_col = -1;
        for (const auto& step : kSteps) {
            const int previous_row = row - step[0];
            const int previous_col = col - step[1];
            if (previous_row >= 0 && previous_col >= 0 &&
                accumulated[previous_row * cols + previous_col] < best_previous) {
                best_previous = accumulated[previous_row * cols + previous_col];
                best_row = previous_row;
                best_col = previous_col;
            }
        }
        row = best_row;
        col = best_col;
    }
    std::reverse(path_rows.begin(), path_rows.end());
    std::reverse(path_cols.begin(), path_cols.end());

    std::vector<float> mean_example_frames(rows, 0.0f);
    std::size_t node = 0;
    for (int reference_row = 0; reference_row < rows; ++reference_row) {
        while (node + 1 < path_rows.size() && path_rows[node + 1] <= reference_row) {
            ++node;
        }
        if (path_rows[node] == reference_row || node + 1 >= path_rows.size()) {
            mean_example_frames[reference_row] = static_cast<float>(path_cols[node]);
        } else {
            // Row skipped by a (2,1) step: interpolate between its neighbors.
            const float alpha =
                static_cast<float>(reference_row - path_rows[node]) /
                static_cast<float>(path_rows[node + 1] - path_rows[node]);
            mean_example_frames[reference_row] =
                static_cast<float>(path_cols[node]) +
                alpha * static_cast<float>(path_cols[node + 1] - path_cols[node]);
        }
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

// Cubic smoothing spline (discrete penalized form): minimize
// ||f - y||^2 + lambda * sum_i (f_{i-1} - 2 f_i + f_{i+1})^2. The minimizer
// solves (I + lambda * D^T D) f = y where D is the second-difference operator;
// its continuous analogue is the natural cubic smoothing spline. KG04 registers
// motions by fitting a cubic B-spline to the dense time-alignment correspondence
// -- this is the equivalent penalized form, used here to denoise the staircase
// DTW frame correspondence into a smooth monotone registration curve before it
// is sampled into warp knots. The result is projected to be non-decreasing.
// Returns the input unchanged when there are too few points to denoise.
std::vector<float> FitSmoothingSpline(
    const std::vector<float>& samples, float lambda) {
    const int count = static_cast<int>(samples.size());
    if (count < 4 || lambda <= 0.0f) {
        return samples;
    }

    // A = I + lambda * D^T D: symmetric positive definite, pentadiagonal.
    std::vector<float> matrix(static_cast<std::size_t>(count) * count, 0.0f);
    const auto at = [&](int row, int col) -> float& {
        return matrix[static_cast<std::size_t>(row) * count + col];
    };
    for (int i = 0; i < count; ++i) {
        at(i, i) = 1.0f;
    }
    for (int center = 1; center + 1 < count; ++center) {
        const int index[3] = {center - 1, center, center + 1};
        const float coeff[3] = {1.0f, -2.0f, 1.0f};
        for (int p = 0; p < 3; ++p) {
            for (int q = 0; q < 3; ++q) {
                at(index[p], index[q]) += lambda * coeff[p] * coeff[q];
            }
        }
    }

    // Dense Cholesky A = L L^T (count is a per-segment row count: tens).
    std::vector<float> lower(static_cast<std::size_t>(count) * count, 0.0f);
    const auto low = [&](int row, int col) -> float& {
        return lower[static_cast<std::size_t>(row) * count + col];
    };
    for (int i = 0; i < count; ++i) {
        for (int j = 0; j <= i; ++j) {
            float sum = at(i, j);
            for (int k = 0; k < j; ++k) {
                sum -= low(i, k) * low(j, k);
            }
            if (i == j) {
                if (sum <= 0.0f) {
                    return samples;  // not SPD (unexpected); bail safely
                }
                low(i, j) = std::sqrt(sum);
            } else {
                low(i, j) = sum / low(j, j);
            }
        }
    }

    // Forward solve L z = samples, back solve L^T f = z.
    std::vector<float> intermediate(count, 0.0f);
    for (int i = 0; i < count; ++i) {
        float sum = samples[i];
        for (int k = 0; k < i; ++k) {
            sum -= low(i, k) * intermediate[k];
        }
        intermediate[i] = sum / low(i, i);
    }
    std::vector<float> smoothed(count, 0.0f);
    for (int i = count - 1; i >= 0; --i) {
        float sum = intermediate[i];
        for (int k = i + 1; k < count; ++k) {
            sum -= low(k, i) * smoothed[k];
        }
        smoothed[i] = sum / low(i, i);
    }

    for (int i = 1; i < count; ++i) {
        smoothed[i] = std::max(smoothed[i], smoothed[i - 1]);
    }
    return smoothed;
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

std::vector<std::vector<float>> MatchedContactAnchors(
    const std::vector<std::vector<ContactInterval>>& example_intervals,
    const std::vector<int>& example_frame_counts) {
    const std::size_t example_count = example_intervals.size();
    std::vector<std::vector<float>> result(example_count);
    if (example_count == 0) {
        return result;
    }
    // Role key = (joint_index, event): 0 = strike, 1 = lift. For each example,
    // collect the interior (0<phase<1) phases per key, sorted by occurrence.
    using Key = std::pair<int, int>;
    std::vector<std::map<Key, std::vector<float>>> per_example(example_count);
    auto push_interior = [](std::vector<float>& v, float phase) {
        if (phase > 0.0f && phase < 1.0f) v.push_back(phase);
    };
    for (std::size_t e = 0; e < example_count; ++e) {
        for (const ContactInterval& interval : example_intervals[e]) {
            push_interior(per_example[e][{interval.joint_index, 0}],
                          interval.StrikePhase(example_frame_counts[e]));
            push_interior(per_example[e][{interval.joint_index, 1}],
                          interval.LiftPhase(example_frame_counts[e]));
        }
        for (auto& kv : per_example[e]) {
            std::sort(kv.second.begin(), kv.second.end());
        }
    }
    // A matched anchor exists for (key, occurrence i) only if EVERY example has at
    // least i+1 of that key. Keep min-count occurrences per key; drop the rest.
    struct Descriptor {
        Key key;
        std::size_t index;
        float mean_phase;
    };
    std::vector<Descriptor> descriptors;
    for (const auto& kv : per_example[0]) {
        const Key& key = kv.first;
        std::size_t common = kv.second.size();
        for (std::size_t e = 1; e < example_count; ++e) {
            const auto it = per_example[e].find(key);
            common = std::min(common,
                              it == per_example[e].end() ? std::size_t{0}
                                                         : it->second.size());
        }
        for (std::size_t i = 0; i < common; ++i) {
            float sum = 0.0f;
            for (std::size_t e = 0; e < example_count; ++e) {
                sum += per_example[e].at(key)[i];
            }
            descriptors.push_back({key, i, sum / static_cast<float>(example_count)});
        }
    }
    // Canonical order: by mean phase, so each example's emitted list follows one
    // shared logical ordering (the contract BuildRegistrationWarps expects).
    std::sort(descriptors.begin(), descriptors.end(),
              [](const Descriptor& a, const Descriptor& b) {
                  return a.mean_phase < b.mean_phase;
              });
    for (std::size_t e = 0; e < example_count; ++e) {
        result[e].reserve(descriptors.size());
        for (const Descriptor& d : descriptors) {
            result[e].push_back(per_example[e].at(d.key)[d.index]);
        }
    }
    return result;
}

void RequireBlendableMotionFamily(
    const std::vector<std::vector<ContactInterval>>& example_intervals,
    const std::vector<int>& example_frame_counts) {
    if (example_intervals.empty()) {
        throw std::runtime_error(
            "RequireBlendableMotionFamily: no examples given");
    }
    if (example_intervals.size() != example_frame_counts.size()) {
        throw std::runtime_error(
            "RequireBlendableMotionFamily: interval/frame-count size mismatch");
    }
    // A single-example space never blends, so it cannot be a cross-family blend;
    // the premise only constrains spaces that actually interpolate two or more
    // examples. (A single non-looping clip looping at runtime is a separate
    // concern, not a blend-space defect.)
    if (example_intervals.size() < 2) {
        return;
    }

    std::size_t expected_anchor_count = 0;
    bool expected_set = false;
    for (std::size_t example_index = 0;
         example_index < example_intervals.size(); ++example_index) {
        const std::vector<ContactInterval>& intervals =
            example_intervals[example_index];
        if (intervals.size() < 2) {
            throw std::runtime_error(
                "RequireBlendableMotionFamily: example " +
                std::to_string(example_index) +
                " is not a single looping cycle (found " +
                std::to_string(intervals.size()) +
                " contact interval(s)); a non-looping motion (e.g. a vault) "
                "must be its own graph node joined by a transition edge, not a "
                "blend-space example");
        }
        const std::size_t anchor_count =
            ContactAnchorPhases(intervals, example_frame_counts[example_index])
                .size();
        if (!expected_set) {
            expected_anchor_count = anchor_count;
            expected_set = true;
        } else if (anchor_count != expected_anchor_count) {
            throw std::runtime_error(
                "RequireBlendableMotionFamily: example " +
                std::to_string(example_index) + " has " +
                std::to_string(anchor_count) + " contact anchors, expected " +
                std::to_string(expected_anchor_count) +
                "; different motion families must be separate nodes joined by "
                "a transition edge, not examples of one blend space");
        }
    }
}

void RequireBlendableMotionFamily(
    const Skeleton& skeleton,
    const std::vector<MotionClip>& examples,
    const std::vector<int>& contact_joints,
    const ContactDetectionSettings& settings) {
    if (contact_joints.empty()) {
        throw std::runtime_error(
            "RequireBlendableMotionFamily: no contact joints given; cannot "
            "verify motion-family compatibility");
    }

    std::vector<std::vector<ContactInterval>> example_intervals;
    std::vector<int> example_frame_counts;
    example_intervals.reserve(examples.size());
    example_frame_counts.reserve(examples.size());
    for (const MotionClip& clip : examples) {
        example_intervals.push_back(
            DetectContacts(skeleton, clip, contact_joints, settings));
        example_frame_counts.push_back(clip.NumFrames());
    }
    RequireBlendableMotionFamily(example_intervals, example_frame_counts);
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
    const ContactDetectionSettings& settings,
    bool tolerant_structure) {
    if (space.NumExamples() == 0) {
        throw std::runtime_error("RegisterSpaceByContacts: space has no examples");
    }

    // Tolerant path (opt-in): align examples with DIFFERENT contact structure by
    // matching anchors per (joint, strike/lift, occurrence) and warping over the
    // common subset, instead of requiring identical anchor counts. Faithful §3
    // tweak; default off keeps byte-identical legacy behavior.
    if (tolerant_structure) {
        std::vector<std::vector<ContactInterval>> example_intervals;
        std::vector<int> example_frame_counts;
        example_intervals.reserve(space.Examples().size());
        example_frame_counts.reserve(space.Examples().size());
        for (const ExampleMotion& example : space.Examples()) {
            example_intervals.push_back(
                DetectContacts(skeleton, example.clip, contact_joints, settings));
            example_frame_counts.push_back(example.clip.NumFrames());
        }
        std::vector<std::vector<float>> matched =
            MatchedContactAnchors(example_intervals, example_frame_counts);
        if (matched.empty() || matched.front().empty()) {
            throw std::runtime_error(
                "RegisterSpaceByContacts: tolerant registration found no contact "
                "anchor common to every example");
        }
        space.SetExampleTimeWarps(BuildRegistrationWarps(matched));
        return;
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
                const std::vector<float> raw_example_frames =
                    DtwMeanExampleFrames(reference_clouds, example_clouds);
                // Denoise the staircase DTW correspondence into a smooth monotone
                // curve (KG04 cubic-spline registration), then sample it densely
                // so the stored warp follows the spline rather than the raw steps.
                const std::vector<float> mean_example_frames =
                    FitSmoothingSpline(raw_example_frames, settings.smoothing_strength);

                // Place one knot per few reference rows, capped, so dense
                // segments track the smooth curve and short ones stay sparse.
                const int interior_rows = reference_last - reference_first - 1;
                const int knot_count = std::clamp(
                    std::min(settings.max_knots_per_segment, interior_rows),
                    0, settings.max_knots_per_segment);
                for (int knot = 1;
                     !mean_example_frames.empty() && knot <= knot_count;
                     ++knot) {
                    const float alpha =
                        static_cast<float>(knot) /
                        static_cast<float>(knot_count + 1);
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
