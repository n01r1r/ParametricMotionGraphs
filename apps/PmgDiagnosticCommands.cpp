#include "PmgCommandModules.h"

#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/CyclicContinuity.h"
#include "pmg/FootLocking.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/GraphIo.h"
#include "pmg/GraphSpec.h"
#include "pmg/MotionDistance.h"
#include "pmg/MotionRegistration.h"
#include "pmg/MotionSpacePreparation.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/RuntimeController.h"
#include "pmg/RootCanonicalization.h"
#include "pmg/SkeletonCompatibility.h"
#include "pmg/legacy/FrameCountClipGeneration.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>


namespace {


float HorizontalLength(const pmg::Vec3& value) {
    return std::sqrt(value.x * value.x + value.z * value.z);
}

std::vector<std::string> SplitCommaList(const std::string& text) {
    std::vector<std::string> items;
    std::istringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
        if (!item.empty()) {
            items.push_back(item);
        }
    }
    return items;
}

struct SpaceSweepOptions {
    std::string spec_path;
    std::string node_name;
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    std::string cycle_joint;  // empty = use example clips as authored
    bool contact_joints_set = false;
    bool cycle_joint_set = false;
    int sweep_steps = 11;
    // Floor for contact run length; real mocap shows spurious 1-2 frame
    // grazes that would break anchor-structure matching across examples.
    int min_contact_frames = 3;
    bool min_contact_frames_set = false;
    // Assert thresholds; negative = report only.
    int min_contacts = -1;
    float max_foot_slide = -1.0f;
    float max_adjacent_step = -1.0f;
    bool assert_no_regression = false;  // registered must not be worse than naive
    // Also measure a DTW-refined registration and gate it against the
    // contact-anchor registration the same way.
    bool dtw_refine = false;
    // Also measure IK foot locking applied to the best registered variant's
    // generated clips and require it to cut the residual slide.
    bool foot_lock = false;
};

void RequireMatchingProductionRegistration(
    const pmg::NodeRegistrationMetadata& registration,
    const std::string& node_name,
    const std::string& cycle_joint,
    bool cycle_joint_set,
    const std::vector<std::string>& contact_joints,
    bool contact_joints_set,
    int min_contact_frames,
    bool min_contact_frames_set,
    const char* command) {
    if (cycle_joint_set && registration.cycle_joint != cycle_joint) {
        throw std::runtime_error(
            std::string(command) + ": --cycle-joint does not match node '" +
            node_name + "' production registration");
    }
    if (contact_joints_set &&
        registration.contact_joints != contact_joints) {
        throw std::runtime_error(
            std::string(command) +
            ": --contact-joints does not match node '" + node_name +
            "' production registration");
    }
    if (min_contact_frames_set &&
        registration.min_contact_frames != min_contact_frames) {
        throw std::runtime_error(
            std::string(command) +
            ": --min-contact-frames does not match node '" + node_name +
            "' production registration");
    }
}

struct SweepMetrics {
    int min_contacts = 0;
    // Total planted frames in the worst sweep step. Degraded blends lift the
    // foot during what should be stance, so low coverage = lost contacts.
    int min_contact_coverage = 0;
    float max_foot_slide = 0.0f;
    // Slide per second of contact. Comparable across blends whose detected
    // stance lengths differ (longer honest stance accumulates more absolute
    // drift than a short smeared one).
    float max_slide_rate = 0.0f;
    float max_adjacent_step = 0.0f;
};

// Horizontal drift of a joint while "planted": max distance from its position
// at the strike frame, over the contact interval. The numeric form of the
// foot-slide artifact.
float ContactSlide(
    const pmg::Skeleton& skeleton,
    const pmg::MotionClip& clip,
    const pmg::ContactInterval& interval) {
    const pmg::Vec3 strike_position =
        pmg::ComputeJointWorldPositions(skeleton, clip.frames[interval.first_frame])
            [interval.joint_index];
    float max_drift = 0.0f;
    for (int frame = interval.first_frame; frame <= interval.last_frame; ++frame) {
        const pmg::Vec3 position =
            pmg::ComputeJointWorldPositions(skeleton, clip.frames[frame])[interval.joint_index];
        const pmg::Vec3 delta = position - strike_position;
        max_drift = std::max(max_drift, HorizontalLength(delta));
    }
    return max_drift;
}

// Mean joint world distance between two equally-sized poses.
float MeanJointDistance(
    const pmg::Skeleton& skeleton,
    const pmg::Pose& first,
    const pmg::Pose& second) {
    const std::vector<pmg::Vec3> first_positions =
        pmg::ComputeJointWorldPositions(skeleton, first);
    const std::vector<pmg::Vec3> second_positions =
        pmg::ComputeJointWorldPositions(skeleton, second);
    float sum = 0.0f;
    for (std::size_t i = 0; i < first_positions.size(); ++i) {
        sum += (first_positions[i] - second_positions[i]).Norm();
    }
    return sum / static_cast<float>(first_positions.size());
}

// Sweep the first parameter dimension across the space's domain (other
// dimensions pinned at the domain midpoint), generate a clip per step, and
// measure: contacts found per step, worst foot slide inside any contact, and
// the worst frame-wise mean joint distance between adjacent steps.
SweepMetrics MeasureSpaceSweep(
    const pmg::ParametricMotionSpace& space,
    const pmg::Skeleton& skeleton,
    const std::vector<int>& contact_joints,
    const pmg::ContactDetectionSettings& settings,
    int sweep_steps,
    int generated_frame_count,
    float frames_per_second,
    const char* label,
    bool foot_lock = false) {
    const std::vector<float> min_parameter = space.MinParameter();
    const std::vector<float> max_parameter = space.MaxParameter();

    SweepMetrics metrics;
    metrics.min_contacts = std::numeric_limits<int>::max();
    metrics.min_contact_coverage = std::numeric_limits<int>::max();

    std::vector<pmg::MotionClip> generated;
    generated.reserve(static_cast<std::size_t>(sweep_steps));

    std::cout << "--- sweep " << label << " ---\n";
    std::cout << "step,parameter,contacts,contact_frames,foot_slide,slide_rate\n";
    for (int step = 0; step < sweep_steps; ++step) {
        const float alpha =
            sweep_steps == 1 ? 0.0f
                             : static_cast<float>(step) / static_cast<float>(sweep_steps - 1);
        pmg::ParameterVector parameter(min_parameter.size());
        for (std::size_t dim = 0; dim < parameter.size(); ++dim) {
            const float mid = 0.5f * (min_parameter[dim] + max_parameter[dim]);
            parameter[dim] =
                dim == 0 ? min_parameter[0] + alpha * (max_parameter[0] - min_parameter[0])
                         : mid;
        }

        pmg::MotionClip clip = pmg::legacy::GenerateClipWithFrameCount(
            space, parameter, generated_frame_count, frames_per_second);
        if (foot_lock) {
            pmg::FootLockSettings lock_settings;
            lock_settings.contacts = settings;
            pmg::LockFootContacts(skeleton, clip, contact_joints, lock_settings);
        }

        const std::vector<pmg::ContactInterval> intervals =
            pmg::DetectContacts(skeleton, clip, contact_joints, settings);
        float step_slide = 0.0f;
        float step_slide_rate = 0.0f;
        int step_coverage = 0;
        for (const pmg::ContactInterval& interval : intervals) {
            const float slide = ContactSlide(skeleton, clip, interval);
            const int contact_frames = interval.last_frame - interval.first_frame + 1;
            const float contact_seconds =
                static_cast<float>(contact_frames) / frames_per_second;
            step_slide = std::max(step_slide, slide);
            step_slide_rate = std::max(step_slide_rate, slide / contact_seconds);
            step_coverage += contact_frames;
        }

        metrics.min_contacts = std::min(metrics.min_contacts, static_cast<int>(intervals.size()));
        metrics.min_contact_coverage = std::min(metrics.min_contact_coverage, step_coverage);
        metrics.max_foot_slide = std::max(metrics.max_foot_slide, step_slide);
        metrics.max_slide_rate = std::max(metrics.max_slide_rate, step_slide_rate);

        std::cout << step << "," << parameter[0] << "," << intervals.size() << ","
                  << step_coverage << "," << step_slide << "," << step_slide_rate << "\n";
        generated.push_back(clip);
    }

    for (std::size_t step = 0; step + 1 < generated.size(); ++step) {
        for (int frame = 0; frame < generated_frame_count; ++frame) {
            const float distance = MeanJointDistance(
                skeleton, generated[step].frames[frame], generated[step + 1].frames[frame]);
            metrics.max_adjacent_step = std::max(metrics.max_adjacent_step, distance);
        }
    }

    std::cout << label << "_min_contacts=" << metrics.min_contacts << "\n";
    std::cout << label << "_min_contact_coverage=" << metrics.min_contact_coverage << "\n";
    std::cout << label << "_max_foot_slide=" << metrics.max_foot_slide << "\n";
    std::cout << label << "_max_slide_rate=" << metrics.max_slide_rate << "\n";
    std::cout << label << "_max_adjacent_step=" << metrics.max_adjacent_step << "\n";
    return metrics;
}

// Phase 2 diagnostic: numerically verify that a parametric motion space
// produces smooth, contact-preserving motion across its parameter range, and
// that contact registration does not regress the naive blend. Exit code is
// nonzero when an assert threshold is violated.
int SpaceSweepCommand(const SpaceSweepOptions& options) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(options.spec_path);
    pmg::MotionSpacePreparationConfig preparation_config;
    preparation_config.default_cycle_joint = options.cycle_joint;
    preparation_config.default_contact_joints =
        SplitCommaList(options.contact_joints_csv);
    preparation_config.default_min_contact_frames =
        options.min_contact_frames;
    preparation_config.default_dtw_refine = options.dtw_refine;
    const pmg::PreparedMotionSpaces prepared =
        pmg::PrepareMotionSpaces(spec, preparation_config);
    const pmg::PreparedMotionSpace& prepared_node =
        prepared.Node(options.node_name);
    RequireMatchingProductionRegistration(
        prepared_node.registration, options.node_name,
        options.cycle_joint, options.cycle_joint_set,
        SplitCommaList(options.contact_joints_csv),
        options.contact_joints_set, options.min_contact_frames,
        options.min_contact_frames_set, "--space-sweep");
    if (!prepared_node.contact_registered.has_value() ||
        !prepared_node.contact_settings.has_value()) {
        throw std::runtime_error(
            "--space-sweep: node '" + options.node_name +
            "' has no contact Registration");
    }

    const pmg::Skeleton& skeleton = prepared.skeleton;
    const pmg::ParametricMotionSpace& naive_space = prepared_node.authored;
    const pmg::ParametricMotionSpace& registered_space =
        *prepared_node.contact_registered;
    const std::vector<int>& contact_joints =
        prepared_node.contact_joint_indices;
    const pmg::ContactDetectionSettings& settings =
        *prepared_node.contact_settings;
    std::cout << "contact settings: height_threshold=" << settings.height_threshold
              << " speed_threshold=" << settings.speed_threshold
              << " min_contact_frames=" << settings.min_contact_frames << "\n";

    for (const pmg::ExampleMotion& example : naive_space.Examples()) {
        const std::vector<pmg::ContactInterval> intervals =
            pmg::DetectContacts(skeleton, example.clip, contact_joints, settings);
        const std::vector<float> anchors =
            pmg::ContactAnchorPhases(intervals, example.clip.NumFrames());
        std::cout << "example " << example.clip.name
                  << ": frames=" << example.clip.NumFrames()
                  << " contacts=" << intervals.size()
                  << " anchors=" << anchors.size() << "\n";
    }

    const int generated_frame_count = naive_space.Examples().front().clip.NumFrames();
    const float frames_per_second = naive_space.Examples().front().clip.frames_per_second;

    std::cout << "registered=yes\n";

    const SweepMetrics naive = MeasureSpaceSweep(
        naive_space, skeleton, contact_joints, settings, options.sweep_steps,
        generated_frame_count, frames_per_second, "naive");
    const SweepMetrics registered = MeasureSpaceSweep(
        registered_space, skeleton, contact_joints, settings, options.sweep_steps,
        generated_frame_count, frames_per_second, "registered");

    std::optional<SweepMetrics> dtw;
    pmg::ParametricMotionSpace best_space = registered_space;
    if (options.dtw_refine) {
        if (!prepared_node.dtw_refined.has_value()) {
            throw std::runtime_error(
                "--space-sweep: --dtw-refine requested, but node '" +
                options.node_name +
                "' disables DTW refinement in its production registration");
        }
        best_space = *prepared_node.dtw_refined;
        std::cout << "dtw_refined=yes\n";
        dtw = MeasureSpaceSweep(
            best_space, skeleton, contact_joints, settings, options.sweep_steps,
            generated_frame_count, frames_per_second, "dtw");
    }

    // IK foot locking post-processes the best registered variant's clips.
    std::optional<SweepMetrics> locked;
    if (options.foot_lock) {
        std::cout << "foot_lock=yes\n";
        locked = MeasureSpaceSweep(
            best_space, skeleton, contact_joints, settings, options.sweep_steps,
            generated_frame_count, frames_per_second, "locked", /*foot_lock=*/true);
    }

    bool failed = false;
    auto fail_if = [&failed](bool condition, const std::string& message) {
        if (condition) {
            std::cout << "ASSERT FAIL: " << message << "\n";
            failed = true;
        }
    };

    if (options.min_contacts >= 0) {
        fail_if(registered.min_contacts < options.min_contacts,
                "registered_min_contacts=" + std::to_string(registered.min_contacts) +
                    " < " + std::to_string(options.min_contacts));
    }
    if (options.max_foot_slide >= 0.0f) {
        fail_if(registered.max_foot_slide > options.max_foot_slide,
                "registered_max_foot_slide=" + std::to_string(registered.max_foot_slide) +
                    " > " + std::to_string(options.max_foot_slide));
    }
    if (options.max_adjacent_step >= 0.0f) {
        fail_if(registered.max_adjacent_step > options.max_adjacent_step,
                "registered_max_adjacent_step=" + std::to_string(registered.max_adjacent_step) +
                    " > " + std::to_string(options.max_adjacent_step));
    }
    if (options.assert_no_regression) {
        fail_if(registered.min_contacts < naive.min_contacts,
                "registration lost contacts: " + std::to_string(registered.min_contacts) +
                    " < " + std::to_string(naive.min_contacts));
        fail_if(registered.min_contact_coverage < naive.min_contact_coverage,
                "registration lost contact coverage: " +
                    std::to_string(registered.min_contact_coverage) + " < " +
                    std::to_string(naive.min_contact_coverage));
        fail_if(registered.max_slide_rate > naive.max_slide_rate * 1.05f + 1.0e-4f,
                "registration increased slide rate: " +
                    std::to_string(registered.max_slide_rate) + " > " +
                    std::to_string(naive.max_slide_rate));
    }
    if (dtw.has_value()) {
        if (options.min_contacts >= 0) {
            fail_if(dtw->min_contacts < options.min_contacts,
                    "dtw_min_contacts=" + std::to_string(dtw->min_contacts) + " < " +
                        std::to_string(options.min_contacts));
        }
        if (options.max_foot_slide >= 0.0f) {
            fail_if(dtw->max_foot_slide > options.max_foot_slide,
                    "dtw_max_foot_slide=" + std::to_string(dtw->max_foot_slide) + " > " +
                        std::to_string(options.max_foot_slide));
        }
        if (options.max_adjacent_step >= 0.0f) {
            fail_if(dtw->max_adjacent_step > options.max_adjacent_step,
                    "dtw_max_adjacent_step=" + std::to_string(dtw->max_adjacent_step) +
                        " > " + std::to_string(options.max_adjacent_step));
        }
        if (options.assert_no_regression) {
            fail_if(dtw->min_contacts < registered.min_contacts,
                    "dtw refinement lost contacts: " + std::to_string(dtw->min_contacts) +
                        " < " + std::to_string(registered.min_contacts));
            fail_if(dtw->min_contact_coverage < registered.min_contact_coverage,
                    "dtw refinement lost contact coverage: " +
                        std::to_string(dtw->min_contact_coverage) + " < " +
                        std::to_string(registered.min_contact_coverage));
            fail_if(dtw->max_slide_rate > registered.max_slide_rate * 1.05f + 1.0e-4f,
                    "dtw refinement increased slide rate: " +
                        std::to_string(dtw->max_slide_rate) + " > " +
                        std::to_string(registered.max_slide_rate));
        }
    }
    if (locked.has_value()) {
        const SweepMetrics& baseline = dtw.has_value() ? *dtw : registered;
        if (options.min_contacts >= 0) {
            fail_if(locked->min_contacts < options.min_contacts,
                    "locked_min_contacts=" + std::to_string(locked->min_contacts) + " < " +
                        std::to_string(options.min_contacts));
        }
        if (options.max_adjacent_step >= 0.0f) {
            fail_if(locked->max_adjacent_step > options.max_adjacent_step,
                    "locked_max_adjacent_step=" + std::to_string(locked->max_adjacent_step) +
                        " > " + std::to_string(options.max_adjacent_step));
        }
        if (options.assert_no_regression) {
            fail_if(locked->min_contacts < baseline.min_contacts,
                    "foot lock lost contacts: " + std::to_string(locked->min_contacts) +
                        " < " + std::to_string(baseline.min_contacts));
            fail_if(locked->min_contact_coverage < baseline.min_contact_coverage,
                    "foot lock lost contact coverage: " +
                        std::to_string(locked->min_contact_coverage) + " < " +
                        std::to_string(baseline.min_contact_coverage));
            // Locking exists to cut residual slide; require a real reduction,
            // not parity (measured 0.41 -> 0.20 on the walk corpus).
            fail_if(locked->max_slide_rate > baseline.max_slide_rate * 0.6f,
                    "foot lock did not cut slide rate by 40%: " +
                        std::to_string(locked->max_slide_rate) + " > 0.6 * " +
                        std::to_string(baseline.max_slide_rate));
        }
    }

    std::cout << (failed ? "RESULT=FAIL" : "RESULT=PASS") << "\n";
    return failed ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Phase 3 diagnostic: rebuild PMG edges on registered motion spaces and check,
// numerically, that registration does not degrade edge quality (and report
// GOOD fractions / boxes / distances so threshold drift is visible).
// ---------------------------------------------------------------------------

struct ValidateGraphOptions {
    std::string spec_path;
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    std::string cycle_joint;
    bool contact_joints_set = false;
    bool cycle_joint_set = false;
    int min_contact_frames = 3;
    bool min_contact_frames_set = false;
    pmg::PmgBuilderConfig builder;
    bool tgood_set = false;
    bool tbad_set = false;
    bool source_samples_set = false;
    bool target_samples_set = false;
    bool seed_set = false;
    // Assert thresholds on the registered build; negative = report only.
    int min_edge_samples = -1;
    float min_good_fraction = -1.0f;
    bool assert_no_regression = false;
    // Negative disables the comparison. A value of 1.0 requires no increase.
    float max_preparation_min_distance_ratio = -1.0f;
};

struct CyclicAuditOptions {
    std::string spec_path;
    std::string output_csv;
    std::string output_md;
    float frames_per_second = 30.0f;
};

struct CyclicRecutSearchOptions {
    std::string bvh_dir = "BVH";
    std::string output_csv;
    std::string output_md;
    std::vector<std::string> clip_files{
        "walkCurve.bvh",
        "walkMoreCurve.bvh",
        "walkTightCurve.bvh",
        "jogCurve.bvh",
    };
    std::string cycle_joint = "LeftAnkle";
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    int min_contact_frames = 3;
    int min_window_frames = 18;
    int max_window_frames = 45;
    int top_per_clip = 5;
};

struct NamedRecutWindow {
    std::string clip_file;
    int start_frame = 0;
    int end_frame = 0;
};

struct CyclicRecutEvaluateOptions {
    std::string bvh_dir = "BVH";
    std::string output_cyclic_csv;
    std::string output_edge_csv;
    std::string output_md;
    std::string cycle_joint = "LeftAnkle";
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    int min_contact_frames = 3;
    float frames_per_second = 30.0f;
    std::vector<NamedRecutWindow> recut_windows;
};

struct CyclicAuditRow {
    std::string node;
    std::string sample_type;
    pmg::ParameterVector parameter;
    std::string source_clip;
    int num_frames = 0;
    pmg::CyclicContinuityRecord record;
};

struct CyclicRecutCandidate {
    std::string clip_file;
    int start_frame = 0;
    int end_frame = 0;
    int num_frames = 0;
    float score = 0.0f;
    pmg::CyclicContinuityRecord record;
};

struct CandidateEdgeReportRow {
    std::string edge_name;
    bool created = false;
    int transition_samples = 0;
    float mean_good_fraction = 0.0f;
    float mean_min_distance = 0.0f;
    float mean_median_distance = 0.0f;
    std::string reject_reason;
};

std::string JoinParameter(const pmg::ParameterVector& parameter) {
    std::ostringstream cell;
    for (std::size_t axis = 0; axis < parameter.size(); ++axis) {
        if (axis != 0) {
            cell << '|';
        }
        cell << parameter[axis];
    }
    return cell.str();
}

std::string EscapeCsvCell(const std::string& cell) {
    if (cell.find_first_of(",\"\n") == std::string::npos) {
        return cell;
    }
    std::string escaped = "\"";
    for (const char ch : cell) {
        if (ch == '"') {
            escaped += "\"\"";
        } else {
            escaped += ch;
        }
    }
    escaped += '"';
    return escaped;
}

std::string CurrentGitCommit() {
    std::array<char, 128> buffer{};
    std::string output;
#ifdef _WIN32
    FILE* pipe = _popen("git rev-parse --short HEAD", "r");
#else
    FILE* pipe = popen("git rev-parse --short HEAD", "r");
#endif
    if (pipe == nullptr) {
        return "unknown";
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) !=
           nullptr) {
        output += buffer.data();
    }
#ifdef _WIN32
    _pclose(pipe);
#else
    pclose(pipe);
#endif
    while (!output.empty() &&
           (output.back() == '\n' || output.back() == '\r')) {
        output.pop_back();
    }
    return output.empty() ? "unknown" : output;
}

std::string CommandLineString(int argc, char** argv) {
    std::ostringstream command;
    for (int index = 0; index < argc; ++index) {
        if (index != 0) {
            command << ' ';
        }
        command << argv[index];
    }
    return command.str();
}

int ResolveJointIndex(
    const pmg::Skeleton& skeleton,
    const std::string& joint_name) {
    for (int joint_index = 0; joint_index < skeleton.NumJoints();
         ++joint_index) {
        if (skeleton.joints[static_cast<std::size_t>(joint_index)].name ==
            joint_name) {
            return joint_index;
        }
    }
    return -1;
}

pmg::CyclicContinuityContext CyclicContextForNode(
    const pmg::Skeleton& skeleton,
    const pmg::PreparedMotionSpace& prepared_node) {
    pmg::CyclicContinuityContext context;
    for (const std::string& joint_name :
         prepared_node.registration.contact_joints) {
        std::string lower_name = joint_name;
        std::transform(
            lower_name.begin(), lower_name.end(), lower_name.begin(),
            [](unsigned char character) {
                return static_cast<char>(std::tolower(character));
            });
        if (lower_name.find("left") != std::string::npos) {
            context.left_foot_joint = ResolveJointIndex(skeleton, joint_name);
        } else if (lower_name.find("right") != std::string::npos) {
            context.right_foot_joint = ResolveJointIndex(skeleton, joint_name);
        }
    }
    context.contact_settings = prepared_node.contact_settings;
    return context;
}

pmg::MotionClip CropClipInclusive(
    const pmg::MotionClip& source,
    int start_frame,
    int end_frame) {
    if (start_frame < 0 || end_frame < start_frame ||
        end_frame >= source.NumFrames()) {
        throw std::runtime_error("CropClipInclusive: invalid frame range");
    }

    pmg::MotionClip cropped;
    cropped.name = source.name + "_recut_" + std::to_string(start_frame) +
                   "_" + std::to_string(end_frame);
    cropped.frames_per_second = source.frames_per_second;
    cropped.frames.assign(
        source.frames.begin() + start_frame,
        source.frames.begin() + end_frame + 1);
    return cropped;
}

float ThresholdNormalizedRecutScore(
    const pmg::CyclicContinuityRecord& record,
    const pmg::CyclicContinuityConfig& config) {
    const float contact_score =
        record.has_contact_evidence && !record.contact_state_matches
            ? 1.0f + record.max_contact_drift / config.contact_drift_threshold
            : record.max_contact_drift / config.contact_drift_threshold;
    return std::max({
        record.seam_step_ratio / config.pose_seam_ratio_threshold,
        record.root_speed_ratio / config.root_speed_ratio_threshold,
        record.yaw_rate_ratio / config.yaw_rate_ratio_threshold,
        contact_score,
    });
}

std::vector<int> ResolveContactJointIndices(
    const pmg::Skeleton& skeleton,
    const std::vector<std::string>& joint_names,
    const char* context) {
    std::vector<int> joint_indices;
    joint_indices.reserve(joint_names.size());
    for (const std::string& joint_name : joint_names) {
        const int joint_index = ResolveJointIndex(skeleton, joint_name);
        if (joint_index < 0) {
            throw std::runtime_error(
                std::string(context) + ": unknown joint '" + joint_name + "'");
        }
        joint_indices.push_back(joint_index);
    }
    return joint_indices;
}

pmg::CyclicContinuityContext CyclicContextForRawClip(
    const pmg::Skeleton& skeleton,
    const pmg::MotionClip& clip,
    const CyclicRecutSearchOptions& options) {
    const std::vector<std::string> contact_joint_names =
        SplitCommaList(options.contact_joints_csv);
    const std::vector<int> contact_joints = ResolveContactJointIndices(
        skeleton, contact_joint_names, "--search-cyclic-recuts");

    pmg::CyclicContinuityContext context;
    for (std::size_t joint = 0; joint < contact_joint_names.size(); ++joint) {
        std::string lower_name = contact_joint_names[joint];
        std::transform(
            lower_name.begin(), lower_name.end(), lower_name.begin(),
            [](unsigned char character) {
                return static_cast<char>(std::tolower(character));
            });
        if (lower_name.find("left") != std::string::npos) {
            context.left_foot_joint = contact_joints[joint];
        } else if (lower_name.find("right") != std::string::npos) {
            context.right_foot_joint = contact_joints[joint];
        }
    }

    pmg::ContactDetectionSettings settings =
        pmg::EstimateContactSettings(skeleton, clip, contact_joints);
    settings.min_contact_frames = options.min_contact_frames;
    context.contact_settings = settings;
    return context;
}

std::vector<NamedRecutWindow> DefaultGroupBRecuts() {
    return {
        {"walkCurve.bvh", 86, 119},
        {"walkMoreCurve.bvh", 76, 110},
        {"walkTightCurve.bvh", 58, 95},
        {"jogCurve.bvh", 39, 63},
    };
}

NamedRecutWindow ParseRecutWindowSpec(const std::string& text) {
    const std::size_t first_colon = text.find(':');
    const std::size_t second_colon =
        first_colon == std::string::npos
            ? std::string::npos
            : text.find(':', first_colon + 1);
    if (first_colon == std::string::npos ||
        second_colon == std::string::npos ||
        text.find(':', second_colon + 1) != std::string::npos) {
        throw std::runtime_error(
            "--recuts entries must have form clip.bvh:start:end");
    }
    NamedRecutWindow window;
    window.clip_file = text.substr(0, first_colon);
    window.start_frame =
        std::stoi(text.substr(first_colon + 1,
                              second_colon - first_colon - 1));
    window.end_frame = std::stoi(text.substr(second_colon + 1));
    if (window.clip_file.empty() || window.start_frame < 0 ||
        window.end_frame < window.start_frame) {
        throw std::runtime_error("--recuts contains an invalid frame window");
    }
    return window;
}

std::vector<NamedRecutWindow> ParseRecutWindows(const std::string& csv) {
    std::vector<NamedRecutWindow> windows;
    for (const std::string& item : SplitCommaList(csv)) {
        windows.push_back(ParseRecutWindowSpec(item));
    }
    if (windows.empty()) {
        throw std::runtime_error("--recuts must contain at least one window");
    }
    return windows;
}

pmg::ParameterVector ParameterForGroupBRecut(const std::string& clip_file) {
    if (clip_file == "walkCurve.bvh") {
        return {0.0f};
    }
    if (clip_file == "walkMoreCurve.bvh") {
        return {0.5f};
    }
    if (clip_file == "walkTightCurve.bvh") {
        return {1.0f};
    }
    if (clip_file == "jogCurve.bvh") {
        return {0.0f};
    }
    throw std::runtime_error(
        "ParameterForGroupBRecut: unsupported clip '" + clip_file + "'");
}

bool IsWalkRecut(const std::string& clip_file) {
    return clip_file == "walkCurve.bvh" ||
           clip_file == "walkMoreCurve.bvh" ||
           clip_file == "walkTightCurve.bvh";
}

void PrepareCandidateSpace(
    pmg::ParametricMotionSpace& space,
    const pmg::Skeleton& skeleton,
    const std::vector<int>& contact_joint_indices,
    const pmg::ContactDetectionSettings& settings,
    bool dtw_refine,
    const std::vector<pmg::ParameterMetric>& calibration_metrics,
    float frames_per_second) {
    std::vector<std::vector<pmg::ContactInterval>> intervals_by_example;
    std::vector<int> frame_counts;
    intervals_by_example.reserve(space.Examples().size());
    frame_counts.reserve(space.Examples().size());
    for (const pmg::ExampleMotion& example : space.Examples()) {
        intervals_by_example.push_back(
            pmg::DetectContacts(
                skeleton, example.clip, contact_joint_indices, settings));
        frame_counts.push_back(example.clip.NumFrames());
    }
    pmg::RequireBlendableMotionFamily(intervals_by_example, frame_counts);
    pmg::RegisterSpaceByContacts(
        space, skeleton, contact_joint_indices, settings);
    if (dtw_refine) {
        pmg::RefineRegistrationByDtw(space, skeleton);
    }
    if (!calibration_metrics.empty()) {
        space.SetParameterCalibration(
            pmg::CalibrateParameterMetrics(
                space, calibration_metrics, frames_per_second));
    }
}

CandidateEdgeReportRow SummarizeCandidateEdge(
    const std::string& edge_name,
    const pmg::EdgeBuildResult& result) {
    CandidateEdgeReportRow row;
    row.edge_name = edge_name;
    row.created = result.report.edge_created;
    row.transition_samples = static_cast<int>(result.edge.samples.size());
    row.reject_reason = result.report.reject_reason;

    float good_fraction_sum = 0.0f;
    float min_distance_sum = 0.0f;
    float median_distance_sum = 0.0f;
    int report_count = 0;
    for (const pmg::SourceSampleBuildReport& report :
         result.report.source_reports) {
        const int total =
            report.good_count + report.neutral_count + report.bad_count;
        if (total > 0) {
            good_fraction_sum +=
                static_cast<float>(report.good_count) /
                static_cast<float>(total);
        }
        min_distance_sum += report.min_distance;
        median_distance_sum += report.median_distance;
        ++report_count;
    }
    if (report_count > 0) {
        row.mean_good_fraction =
            good_fraction_sum / static_cast<float>(report_count);
        row.mean_min_distance =
            min_distance_sum / static_cast<float>(report_count);
        row.mean_median_distance =
            median_distance_sum / static_cast<float>(report_count);
    }
    return row;
}

pmg::PmgBuilderConfig CandidateEdgeConfig(
    const std::string& source_node,
    const std::string& target_node) {
    pmg::PmgBuilderConfig config;
    if (source_node == "walk" && target_node == "walk") {
        config.good_transition_threshold = 225.0f;
        config.bad_transition_threshold = 250.0f;
        config.source_sample_count = 8;
        config.target_sample_count = 40;
        config.seed = 17;
    } else if (source_node == "walk" && target_node == "jog") {
        config.good_transition_threshold = 450.0f;
        config.bad_transition_threshold = 500.0f;
        config.source_sample_count = 8;
        config.target_sample_count = 20;
        config.seed = 19;
    } else if (source_node == "jog" && target_node == "walk") {
        config.good_transition_threshold = 300.0f;
        config.bad_transition_threshold = 350.0f;
        config.source_sample_count = 4;
        config.target_sample_count = 40;
        config.seed = 23;
    } else if (source_node == "jog" && target_node == "jog") {
        config.good_transition_threshold = 80.0f;
        config.bad_transition_threshold = 100.0f;
        config.source_sample_count = 4;
        config.target_sample_count = 20;
        config.seed = 29;
    } else {
        throw std::runtime_error(
            "CandidateEdgeConfig: unsupported edge " + source_node +
            "->" + target_node);
    }
    return config;
}

std::vector<pmg::ParameterVector> GeneratedAuditParameters(
    const pmg::ParametricMotionSpace& space) {
    const int dimension = space.ParameterDimension();
    const std::vector<pmg::ParameterVector> examples =
        space.ExampleParameters();
    if (examples.empty()) {
        throw std::runtime_error(
            "GeneratedAuditParameters: space has no examples");
    }
    pmg::ParameterVector min_corner(static_cast<std::size_t>(dimension),
                                    std::numeric_limits<float>::infinity());
    pmg::ParameterVector max_corner(static_cast<std::size_t>(dimension),
                                    -std::numeric_limits<float>::infinity());
    for (const pmg::ParameterVector& parameter : examples) {
        pmg::RequireSameParameterDimension(
            min_corner, parameter, "GeneratedAuditParameters");
        for (int axis = 0; axis < dimension; ++axis) {
            min_corner[static_cast<std::size_t>(axis)] = std::min(
                min_corner[static_cast<std::size_t>(axis)],
                parameter[static_cast<std::size_t>(axis)]);
            max_corner[static_cast<std::size_t>(axis)] = std::max(
                max_corner[static_cast<std::size_t>(axis)],
                parameter[static_cast<std::size_t>(axis)]);
        }
    }

    std::vector<pmg::ParameterVector> samples;

    if (dimension == 1) {
        const float min_value = min_corner[0];
        const float max_value = max_corner[0];
        samples.push_back({min_value});
        if (std::abs(max_value - min_value) > 1.0e-6f) {
            samples.push_back({0.5f * (min_value + max_value)});
            samples.push_back({max_value});
        }
        return samples;
    }

    if (dimension == 2) {
        for (int x = 0; x < 2; ++x) {
            for (int y = 0; y < 2; ++y) {
                samples.push_back({
                    x == 0 ? min_corner[0] : max_corner[0],
                    y == 0 ? min_corner[1] : max_corner[1],
                });
            }
        }
        samples.push_back({
            0.5f * (min_corner[0] + max_corner[0]),
            0.5f * (min_corner[1] + max_corner[1]),
        });
        return samples;
    }

    pmg::ParameterVector center(static_cast<std::size_t>(dimension), 0.0f);
    for (int axis = 0; axis < dimension; ++axis) {
        center[static_cast<std::size_t>(axis)] =
            0.5f * (min_corner[static_cast<std::size_t>(axis)] +
                    max_corner[static_cast<std::size_t>(axis)]);
    }
    samples.push_back(center);
    return samples;
}

void WriteCyclicRecutSearchCsv(
    const std::string& path,
    const std::vector<CyclicRecutCandidate>& candidates) {
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error("--search-cyclic-recuts: cannot write CSV");
    }

    csv << "clip,start_frame,end_frame,num_frames,score,"
           "cycle_delta_dx,cycle_delta_dz,cycle_delta_yaw,"
           "seam_step,median_step,seam_step_ratio,"
           "pre_root_speed,seam_root_speed,post_root_speed,root_speed_ratio,"
           "pre_yaw_rate,seam_yaw_rate,post_yaw_rate,yaw_rate_ratio,"
           "left_foot_drift,right_foot_drift,max_contact_drift,"
           "has_contact_evidence,contact_state_matches,classification\n";
    for (const CyclicRecutCandidate& candidate : candidates) {
        const pmg::CyclicContinuityRecord& r = candidate.record;
        csv << EscapeCsvCell(candidate.clip_file) << ','
            << candidate.start_frame << ',' << candidate.end_frame << ','
            << candidate.num_frames << ',' << candidate.score << ','
            << r.cycle_delta.dx << ',' << r.cycle_delta.dz << ','
            << r.cycle_delta.yaw << ',' << r.seam_step << ','
            << r.median_step << ',' << r.seam_step_ratio << ','
            << r.pre_root_speed << ',' << r.seam_root_speed << ','
            << r.post_root_speed << ',' << r.root_speed_ratio << ','
            << r.pre_yaw_rate << ',' << r.seam_yaw_rate << ','
            << r.post_yaw_rate << ',' << r.yaw_rate_ratio << ','
            << r.left_foot_drift << ',' << r.right_foot_drift << ','
            << r.max_contact_drift << ','
            << (r.has_contact_evidence ? 1 : 0) << ','
            << (r.contact_state_matches ? 1 : 0) << ','
            << pmg::CyclicContinuityClassificationName(r.classification)
            << '\n';
    }
}

std::vector<CyclicRecutCandidate> BestRecutCandidates(
    const std::vector<CyclicRecutCandidate>& candidates,
    const std::string& clip_file,
    int limit) {
    std::vector<CyclicRecutCandidate> filtered;
    for (const CyclicRecutCandidate& candidate : candidates) {
        if (candidate.clip_file == clip_file) {
            filtered.push_back(candidate);
        }
    }
    std::sort(
        filtered.begin(), filtered.end(),
        [](const CyclicRecutCandidate& left,
           const CyclicRecutCandidate& right) {
            if (left.score != right.score) {
                return left.score < right.score;
            }
            return left.num_frames < right.num_frames;
        });
    if (static_cast<int>(filtered.size()) > limit) {
        filtered.resize(static_cast<std::size_t>(limit));
    }
    return filtered;
}

void WriteCyclicRecutSearchMarkdown(
    const std::string& path,
    const CyclicRecutSearchOptions& options,
    const std::string& command_line,
    const std::vector<CyclicRecutCandidate>& candidates) {
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error(
            "--search-cyclic-recuts: cannot write markdown");
    }

    md << "# Cyclic Recut Search Report\n\n";
    md << "- Commit: `" << CurrentGitCommit() << "`\n";
    md << "- Command: `" << command_line << "`\n";
    md << "- BVH directory: `" << options.bvh_dir << "`\n";
    md << "- Cycle joint: `" << options.cycle_joint << "`\n";
    md << "- Contact joints: `" << options.contact_joints_csv << "`\n";
    md << "- Candidate frame count: " << options.min_window_frames << "-"
       << options.max_window_frames << " inclusive\n";
    md << "- Score: max of threshold-normalized seam pose, root-speed, "
          "yaw-rate, and contact evidence metrics from CyclicContinuity; "
          "lower is better.\n\n";

    md << "## Recommended Recut\n\n";
    md << "| Clip | Start | End | Frames | Score | Seam ratio | Root ratio | "
          "Yaw ratio | Contact drift | Contact match | Classification |\n";
    md << "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|\n";
    for (const std::string& clip_file : options.clip_files) {
        const std::vector<CyclicRecutCandidate> best =
            BestRecutCandidates(candidates, clip_file, 1);
        if (best.empty()) {
            md << "| `" << clip_file
               << "` | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | "
                  "n/a | insufficient_candidates |\n";
            continue;
        }
        const CyclicRecutCandidate& candidate = best.front();
        const pmg::CyclicContinuityRecord& r = candidate.record;
        md << "| `" << clip_file << "` | " << candidate.start_frame << " | "
           << candidate.end_frame << " | " << candidate.num_frames << " | "
           << candidate.score << " | " << r.seam_step_ratio << " | "
           << r.root_speed_ratio << " | " << r.yaw_rate_ratio << " | "
           << r.max_contact_drift << " | "
           << (r.contact_state_matches ? "yes" : "no") << " | "
           << pmg::CyclicContinuityClassificationName(r.classification)
           << " |\n";
    }

    md << "\n## Top Candidates By Clip\n\n";
    for (const std::string& clip_file : options.clip_files) {
        md << "### `" << clip_file << "`\n\n";
        md << "| Rank | Start | End | Frames | Score | Seam ratio | Root ratio | "
              "Yaw ratio | Contact drift | Classification |\n";
        md << "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n";
        const std::vector<CyclicRecutCandidate> best =
            BestRecutCandidates(candidates, clip_file, options.top_per_clip);
        for (std::size_t index = 0; index < best.size(); ++index) {
            const CyclicRecutCandidate& candidate = best[index];
            const pmg::CyclicContinuityRecord& r = candidate.record;
            md << "| " << (index + 1) << " | " << candidate.start_frame
               << " | " << candidate.end_frame << " | "
               << candidate.num_frames << " | " << candidate.score << " | "
               << r.seam_step_ratio << " | " << r.root_speed_ratio << " | "
               << r.yaw_rate_ratio << " | " << r.max_contact_drift << " | "
               << pmg::CyclicContinuityClassificationName(r.classification)
               << " |\n";
        }
        md << '\n';
    }

    md << "## Interpretation\n\n";
    md << "This report recommends cut points only. It does not edit BVH files, "
          "specs, PMG artifact construction, graph edges, transition "
          "thresholds, or runtime scheduling. A recommended window still needs "
          "visual review plus a separate artifact/runtime comparison before it "
          "can replace a tracked anchor.\n";
}

void WriteCandidateEdgeCsv(
    const std::string& path,
    const std::vector<CandidateEdgeReportRow>& rows) {
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error(
            "--evaluate-cyclic-recuts: cannot write edge CSV");
    }
    csv << "edge,created,transition_samples,mean_good_fraction,"
           "mean_min_distance,mean_median_distance,reject_reason\n";
    for (const CandidateEdgeReportRow& row : rows) {
        csv << EscapeCsvCell(row.edge_name) << ','
            << (row.created ? 1 : 0) << ','
            << row.transition_samples << ','
            << row.mean_good_fraction << ','
            << row.mean_min_distance << ','
            << row.mean_median_distance << ','
            << EscapeCsvCell(row.reject_reason) << '\n';
    }
}

void WriteCyclicRecutEvaluationMarkdown(
    const std::string& path,
    const CyclicRecutEvaluateOptions& options,
    const std::string& command_line,
    const std::vector<CyclicAuditRow>& cyclic_rows,
    const std::vector<CandidateEdgeReportRow>& edge_rows) {
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error(
            "--evaluate-cyclic-recuts: cannot write markdown");
    }

    int strong_count = 0;
    for (const CyclicAuditRow& row : cyclic_rows) {
        if (row.record.classification ==
            pmg::CyclicContinuityClassification::kStrong) {
            ++strong_count;
        }
    }

    md << "# Cyclic Recut Candidate Evaluation\n\n";
    md << "- Commit: `" << CurrentGitCommit() << "`\n";
    md << "- Command: `" << command_line << "`\n";
    md << "- BVH directory: `" << options.bvh_dir << "`\n";
    md << "- Cycle joint: `" << options.cycle_joint << "`\n";
    md << "- Contact joints: `" << options.contact_joints_csv << "`\n";
    md << "- Sampling rate: " << options.frames_per_second << " fps\n\n";

    md << "## Cyclic Summary\n\n";
    md << "- Strong samples: " << strong_count << " / "
       << cyclic_rows.size() << "\n\n";
    md << "| Node | Type | Parameter | Clip | Frames | Seam ratio | "
          "Root ratio | Yaw ratio | Contact drift | Classification |\n";
    md << "|---|---|---|---|---:|---:|---:|---:|---:|---|\n";
    for (const CyclicAuditRow& row : cyclic_rows) {
        const pmg::CyclicContinuityRecord& r = row.record;
        md << "| " << row.node << " | " << row.sample_type << " | `"
           << JoinParameter(row.parameter) << "` | `" << row.source_clip
           << "` | " << row.num_frames << " | " << r.seam_step_ratio
           << " | " << r.root_speed_ratio << " | " << r.yaw_rate_ratio
           << " | " << r.max_contact_drift << " | "
           << pmg::CyclicContinuityClassificationName(r.classification)
           << " |\n";
    }

    md << "\n## Edge Build Summary\n\n";
    md << "| Edge | Created | Samples | Mean GOOD fraction | Mean min D | "
          "Mean median D | Reject reason |\n";
    md << "|---|---:|---:|---:|---:|---:|---|\n";
    for (const CandidateEdgeReportRow& row : edge_rows) {
        md << "| `" << row.edge_name << "` | " << (row.created ? 1 : 0)
           << " | " << row.transition_samples << " | "
           << row.mean_good_fraction << " | " << row.mean_min_distance
           << " | " << row.mean_median_distance << " | "
           << row.reject_reason << " |\n";
    }

    md << "\n## Interpretation\n\n";
    md << "This evaluation applies the recommended recut windows in memory, "
          "then runs the same contact registration, DTW refinement, turn-rate "
          "calibration, generated cyclic sampling, and edge-build thresholds "
          "used by `demo_walk_jog_topology`. It is still diagnostic-only: no "
          "BVH, spec, artifact, graph build, or runtime behavior is changed.\n";
}

void WriteCyclicAuditCsv(
    const std::string& path,
    const std::vector<CyclicAuditRow>& rows) {
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error("--audit-cyclic-continuity: cannot write CSV");
    }

    csv << "node,sample_type,parameter,source_clip,num_frames,"
           "cycle_delta_dx,cycle_delta_dz,cycle_delta_yaw,"
           "seam_step,median_step,seam_step_ratio,"
           "pre_root_speed,seam_root_speed,post_root_speed,root_speed_ratio,"
           "pre_yaw_rate,seam_yaw_rate,post_yaw_rate,yaw_rate_ratio,"
           "left_foot_drift,right_foot_drift,max_contact_drift,"
           "has_contact_evidence,contact_state_matches,classification\n";
    for (const CyclicAuditRow& row : rows) {
        const pmg::CyclicContinuityRecord& r = row.record;
        csv << EscapeCsvCell(row.node) << ','
            << EscapeCsvCell(row.sample_type) << ','
            << EscapeCsvCell(JoinParameter(row.parameter)) << ','
            << EscapeCsvCell(row.source_clip) << ',' << row.num_frames << ','
            << r.cycle_delta.dx << ',' << r.cycle_delta.dz << ','
            << r.cycle_delta.yaw << ',' << r.seam_step << ','
            << r.median_step << ',' << r.seam_step_ratio << ','
            << r.pre_root_speed << ',' << r.seam_root_speed << ','
            << r.post_root_speed << ',' << r.root_speed_ratio << ','
            << r.pre_yaw_rate << ',' << r.seam_yaw_rate << ','
            << r.post_yaw_rate << ',' << r.yaw_rate_ratio << ','
            << r.left_foot_drift << ',' << r.right_foot_drift << ','
            << r.max_contact_drift << ','
            << (r.has_contact_evidence ? 1 : 0) << ','
            << (r.contact_state_matches ? 1 : 0) << ','
            << pmg::CyclicContinuityClassificationName(r.classification)
            << '\n';
    }
}

void WriteCyclicAuditMarkdown(
    const std::string& path,
    const CyclicAuditOptions& options,
    const std::string& command_line,
    const std::vector<CyclicAuditRow>& rows) {
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error("--audit-cyclic-continuity: cannot write markdown");
    }

    md << "# Cyclic Continuity Audit\n\n";
    md << "- Commit: `" << CurrentGitCommit() << "`\n";
    md << "- Command: `" << command_line << "`\n";
    md << "- Spec: `" << options.spec_path << "`\n";
    md << "- Sampling rate: " << options.frames_per_second << " fps\n\n";

    md << "## Summary By Node\n\n";
    md << "| Node | Samples | Strong | Weak pose | Weak root speed | "
          "Weak yaw rate | Weak contact | Insufficient |\n";
    md << "|---|---:|---:|---:|---:|---:|---:|---:|\n";
    std::map<std::string, std::array<int, 6>> counts_by_node;
    for (const CyclicAuditRow& row : rows) {
        auto& counts = counts_by_node[row.node];
        const int index = static_cast<int>(row.record.classification);
        counts[static_cast<std::size_t>(index)] += 1;
    }
    for (const auto& [node, counts] : counts_by_node) {
        int total = 0;
        for (const int count : counts) {
            total += count;
        }
        md << "| " << node << " | " << total << " | "
           << counts[0] << " | " << counts[1] << " | "
           << counts[2] << " | " << counts[3] << " | "
           << counts[4] << " | " << counts[5] << " |\n";
    }

    md << "\n## Per Sample\n\n";
    md << "| Node | Type | Parameter | Clip | Frames | Seam ratio | "
          "Root ratio | Yaw ratio | Contact drift | Classification |\n";
    md << "|---|---|---|---|---:|---:|---:|---:|---:|---|\n";
    for (const CyclicAuditRow& row : rows) {
        const pmg::CyclicContinuityRecord& r = row.record;
        md << "| " << row.node << " | " << row.sample_type << " | `"
           << JoinParameter(row.parameter) << "` | `" << row.source_clip
           << "` | " << row.num_frames << " | " << r.seam_step_ratio
           << " | " << r.root_speed_ratio << " | " << r.yaw_rate_ratio
           << " | " << r.max_contact_drift << " | "
           << pmg::CyclicContinuityClassificationName(r.classification)
           << " |\n";
    }

    std::vector<CyclicAuditRow> ranked = rows;
    std::sort(
        ranked.begin(), ranked.end(),
        [](const CyclicAuditRow& left, const CyclicAuditRow& right) {
            const auto worst = [](const CyclicAuditRow& row) {
                return std::max({
                    row.record.seam_step_ratio,
                    row.record.root_speed_ratio,
                    row.record.yaw_rate_ratio,
                });
            };
            return worst(left) < worst(right);
        });

    md << "\n## Strongest Cyclic Anchors\n\n";
    for (std::size_t index = 0; index < std::min<std::size_t>(3, ranked.size());
         ++index) {
        const CyclicAuditRow& row = ranked[index];
        md << "- `" << row.node << "` " << row.sample_type << " `"
           << JoinParameter(row.parameter) << "`: "
           << pmg::CyclicContinuityClassificationName(
                  row.record.classification)
           << ", seam ratio " << row.record.seam_step_ratio
           << ", root ratio " << row.record.root_speed_ratio
           << ", yaw ratio " << row.record.yaw_rate_ratio << "\n";
    }

    md << "\n## Weakest Cyclic Anchors\n\n";
    for (std::size_t offset = 0;
         offset < std::min<std::size_t>(3, ranked.size()); ++offset) {
        const CyclicAuditRow& row = ranked[ranked.size() - 1 - offset];
        md << "- `" << row.node << "` " << row.sample_type << " `"
           << JoinParameter(row.parameter) << "`: "
           << pmg::CyclicContinuityClassificationName(
                  row.record.classification)
           << ", seam ratio " << row.record.seam_step_ratio
           << ", root ratio " << row.record.root_speed_ratio
           << ", yaw ratio " << row.record.yaw_rate_ratio << "\n";
    }

    const bool all_strong = std::all_of(
        rows.begin(), rows.end(), [](const CyclicAuditRow& row) {
            return row.record.classification ==
                   pmg::CyclicContinuityClassification::kStrong;
        });
    md << "\n## Conclusion\n\n";
    md << (all_strong
               ? "Every audited cyclic sample classified strong under this "
                 "local seam diagnostic."
               : "At least one audited cyclic sample classified weak, so the "
                 "current corpus should not be described as uniformly strong "
                 "for PMG cyclic self-edge streaming.")
       << " This diagnostic measures local seam continuity only; it does not "
          "claim perceptual smoothness.\n";
}

int CyclicAuditCommand(
    const CyclicAuditOptions& options,
    const std::string& command_line) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(options.spec_path);
    pmg::MotionSpacePreparationConfig preparation_config;
    preparation_config.calibration_frames_per_second =
        options.frames_per_second;
    const pmg::PreparedMotionSpaces prepared =
        pmg::PrepareMotionSpaces(spec, preparation_config);

    std::vector<CyclicAuditRow> rows;
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        const pmg::PreparedMotionSpace& prepared_node =
            prepared.Node(node.name);
        if (prepared_node.registration.cycle_joint.empty()) {
            continue;
        }

        const pmg::CyclicContinuityContext context =
            CyclicContextForNode(prepared.skeleton, prepared_node);
        for (std::size_t example_index = 0;
             example_index < prepared_node.authored.Examples().size();
             ++example_index) {
            const pmg::ExampleMotion& example =
                prepared_node.authored.Examples()[example_index];
            CyclicAuditRow row;
            row.node = node.name;
            row.sample_type = "authored";
            row.parameter = example.parameter;
            row.source_clip =
                example.clip.name.empty()
                    ? "authored_example_" + std::to_string(example_index)
                    : example.clip.name;
            row.num_frames = example.clip.NumFrames();
            row.record = pmg::MeasureCyclicContinuity(
                prepared.skeleton, example.clip, context);
            rows.push_back(std::move(row));
        }

        for (const pmg::ParameterVector& parameter :
             GeneratedAuditParameters(prepared_node.production)) {
            const pmg::MotionClip clip =
                prepared_node.production.GenerateClip(
                    parameter, options.frames_per_second);
            CyclicAuditRow row;
            row.node = node.name;
            row.sample_type = "generated";
            row.parameter = parameter;
            row.source_clip = "";
            row.num_frames = clip.NumFrames();
            row.record = pmg::MeasureCyclicContinuity(
                prepared.skeleton, clip, context);
            rows.push_back(std::move(row));
        }
    }

    WriteCyclicAuditCsv(options.output_csv, rows);
    WriteCyclicAuditMarkdown(options.output_md, options, command_line, rows);

    std::cout << "cyclic_audit_rows=" << rows.size() << "\n";
    std::cout << "cyclic_audit_csv=" << options.output_csv << "\n";
    std::cout << "cyclic_audit_md=" << options.output_md << "\n";
    return 0;
}

struct RootCanonicalizationAuditOptions {
    std::string spec_path;
    std::string output_csv;
    std::string output_md;
};

struct RootCanonicalizationAuditRow {
    std::string node_name;
    std::string bvh_path;
    pmg::ParameterVector parameter;
    pmg::RootStartSummary raw;
    pmg::RootStartSummary normalized;
    bool start_is_origin = false;
    bool heading_is_canonical = false;
    bool displacement_preserved = false;
    std::string verdict;
};

constexpr float kRootOriginTolerance = 1.0e-4f;
constexpr float kRootHeadingTolerance = 1.0e-4f;
constexpr float kDisplacementTolerance = 1.0e-3f;

float HorizontalDistance(const pmg::Vec3& value) {
    return std::sqrt(value.x * value.x + value.z * value.z);
}

std::string ParameterCsv(const pmg::ParameterVector& parameter) {
    std::ostringstream out;
    out << '"';
    for (std::size_t i = 0; i < parameter.size(); ++i) {
        if (i > 0) {
            out << ' ';
        }
        out << parameter[i];
    }
    out << '"';
    return out.str();
}

std::string ParameterMd(const pmg::ParameterVector& parameter) {
    std::ostringstream out;
    out << '(';
    for (std::size_t i = 0; i < parameter.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << parameter[i];
    }
    out << ')';
    return out.str();
}

std::string Vec3Csv(const pmg::Vec3& v) {
    std::ostringstream out;
    out << '"' << v.x << ' ' << v.y << ' ' << v.z << '"';
    return out.str();
}

std::string BoundsCsv(const pmg::Vec3& min_value, const pmg::Vec3& max_value) {
    std::ostringstream out;
    out << '"' << min_value.x << ' ' << min_value.y << ' ' << min_value.z
        << " .. " << max_value.x << ' ' << max_value.y << ' ' << max_value.z
        << '"';
    return out.str();
}

std::string RootAuditVerdict(
    const pmg::RootStartSummary& raw,
    const pmg::RootStartSummary& normalized,
    bool start_is_origin,
    bool heading_is_canonical,
    bool displacement_preserved) {
    if (!start_is_origin) {
        return "FAIL_NOT_CANONICALIZED";
    }
    if (!heading_is_canonical) {
        return "FAIL_HEADING_NOT_CANONICALIZED";
    }
    if (!displacement_preserved) {
        return "FAIL_NOT_CANONICALIZED";
    }
    const bool raw_has_offset =
        HorizontalDistance(raw.first_root_position) > kRootOriginTolerance ||
        std::abs(raw.first_heading_yaw) > kRootHeadingTolerance;
    return raw_has_offset ? "WARN_RAW_OFFSET_ONLY" : "PASS_CANONICALIZED";
}

std::vector<RootCanonicalizationAuditRow> BuildRootCanonicalizationAudit(
    const pmg::GraphSpec& spec,
    const pmg::PreparedMotionSpaces& prepared) {
    std::vector<RootCanonicalizationAuditRow> rows;
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        const pmg::PreparedMotionSpace& prepared_node = prepared.Node(node.name);
        std::size_t node_example_index = 0;
        for (const pmg::GraphSpecExample& example : spec.examples) {
            if (example.node_name != node.name) {
                continue;
            }
            if (node_example_index >= prepared_node.production.Examples().size()) {
                throw std::runtime_error(
                    "--audit-root-canonicalization: prepared example count mismatch");
            }
            const pmg::BvhData raw_bvh = pmg::BvhLoader::Load(example.bvh_path);
            const pmg::MotionClip raw_canonical =
                pmg::CanonicalizeRootOrigin(raw_bvh.clip);
            const pmg::MotionClip& normalized_clip =
                prepared_node.production.Examples()[node_example_index].clip;
            RootCanonicalizationAuditRow row;
            row.node_name = node.name;
            row.bvh_path = example.bvh_path;
            row.parameter = example.parameter;
            row.raw = pmg::SummarizeRootStart(raw_bvh.clip);
            row.normalized = pmg::SummarizeRootStart(normalized_clip);
            row.start_is_origin =
                HorizontalDistance(row.normalized.first_root_position) <=
                kRootOriginTolerance;
            row.heading_is_canonical =
                std::abs(row.normalized.first_heading_yaw) <=
                kRootHeadingTolerance;
            const pmg::RootStartSummary expected =
                pmg::SummarizeRootStart(raw_canonical);
            const pmg::Vec3 displacement_delta =
                row.normalized.final_relative_displacement -
                expected.final_relative_displacement;
            row.displacement_preserved =
                HorizontalDistance(displacement_delta) <= kDisplacementTolerance;
            row.verdict = RootAuditVerdict(
                row.raw, row.normalized, row.start_is_origin,
                row.heading_is_canonical, row.displacement_preserved);
            rows.push_back(std::move(row));
            ++node_example_index;
        }
    }
    return rows;
}

void WriteRootCanonicalizationCsv(
    const std::string& path,
    const std::vector<RootCanonicalizationAuditRow>& rows) {
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error("--audit-root-canonicalization: cannot write CSV");
    }
    csv << std::setprecision(9)
        << "node,bvh,parameter,raw_first_root,raw_first_x,raw_first_z,"
           "raw_first_yaw,normalized_first_root,normalized_first_x,"
           "normalized_first_z,normalized_first_yaw,final_relative_dx,"
           "final_relative_dz,final_relative_heading,normalized_bounds,"
           "start_is_origin,heading_is_canonical,verdict\n";
    for (const RootCanonicalizationAuditRow& row : rows) {
        csv << row.node_name << ',' << row.bvh_path << ','
            << ParameterCsv(row.parameter) << ',' << Vec3Csv(row.raw.first_root_position)
            << ',' << row.raw.first_root_position.x << ','
            << row.raw.first_root_position.z << ',' << row.raw.first_heading_yaw
            << ',' << Vec3Csv(row.normalized.first_root_position) << ','
            << row.normalized.first_root_position.x << ','
            << row.normalized.first_root_position.z << ','
            << row.normalized.first_heading_yaw << ','
            << row.normalized.final_relative_displacement.x << ','
            << row.normalized.final_relative_displacement.z << ','
            << row.normalized.final_relative_heading << ','
            << BoundsCsv(row.normalized.normalized_bounds_min,
                         row.normalized.normalized_bounds_max)
            << ',' << (row.start_is_origin ? "true" : "false")
            << ',' << (row.heading_is_canonical ? "true" : "false")
            << ',' << row.verdict << "\n";
    }
}

void WriteRootCanonicalizationMarkdown(
    const std::string& path,
    const RootCanonicalizationAuditOptions& options,
    const std::vector<RootCanonicalizationAuditRow>& rows) {
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error("--audit-root-canonicalization: cannot write markdown");
    }
    const bool all_origin = std::all_of(rows.begin(), rows.end(), [](const auto& row) {
        return row.start_is_origin;
    });
    const bool all_heading = std::all_of(rows.begin(), rows.end(), [](const auto& row) {
        return row.heading_is_canonical;
    });
    const bool raw_starts_differ = [&]() {
        if (rows.size() < 2) {
            return false;
        }
        const pmg::Vec3 first = rows.front().raw.first_root_position;
        const float yaw = rows.front().raw.first_heading_yaw;
        for (const auto& row : rows) {
            if (HorizontalDistance(row.raw.first_root_position - first) >
                    kRootOriginTolerance ||
                std::abs(row.raw.first_heading_yaw - yaw) >
                    kRootHeadingTolerance) {
                return true;
            }
        }
        return false;
    }();
    std::string recommendation = "PASS_NO_CANONICALIZATION_CHANGE_NEEDED";
    if (!all_origin) {
        recommendation = "NEED_LOAD_TIME_CANONICALIZATION";
    } else if (!all_heading) {
        recommendation = "NEED_HEADING_CANONICALIZATION";
    }

    md << "# Root Canonicalization Audit\n\n";
    md << "- Spec: `" << options.spec_path << "`\n";
    md << "- Raw BVH starts differ: " << (raw_starts_differ ? "yes" : "no") << "\n";
    md << "- Prepared normalized starts align: " << (all_origin ? "yes" : "no") << "\n";
    md << "- Heading normalization consistent: " << (all_heading ? "yes" : "no") << "\n";
    md << "- Final recommendation: `" << recommendation << "`\n\n";
    md << "| Node | BVH | Parameter | Raw start x/z | Raw yaw | Normalized start x/z | Normalized yaw | Final dx/dz | Bounds x/z | Verdict |\n";
    md << "|---|---|---|---:|---:|---:|---:|---:|---:|---|\n";
    for (const RootCanonicalizationAuditRow& row : rows) {
        md << "| " << row.node_name << " | "
           << std::filesystem::path(row.bvh_path).filename().string() << " | "
           << ParameterMd(row.parameter) << " | "
           << row.raw.first_root_position.x << ", " << row.raw.first_root_position.z
           << " | " << row.raw.first_heading_yaw << " | "
           << row.normalized.first_root_position.x << ", "
           << row.normalized.first_root_position.z << " | "
           << row.normalized.first_heading_yaw << " | "
           << row.normalized.final_relative_displacement.x << ", "
           << row.normalized.final_relative_displacement.z << " | "
           << row.normalized.normalized_bounds_min.x << ".."
           << row.normalized.normalized_bounds_max.x << ", "
           << row.normalized.normalized_bounds_min.z << ".."
           << row.normalized.normalized_bounds_max.z << " | `"
           << row.verdict << "` |\n";
    }
}

RootCanonicalizationAuditOptions ParseRootCanonicalizationAuditOptions(
    int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--audit-root-canonicalization needs <spec>");
    }
    RootCanonicalizationAuditOptions options;
    options.spec_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--output-csv") {
            options.output_csv = require_value("--output-csv");
        } else if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else {
            throw std::runtime_error(
                "unknown audit-root-canonicalization option '" + option + "'");
        }
    }
    if (options.output_csv.empty()) {
        throw std::runtime_error(
            "--audit-root-canonicalization requires --output-csv");
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-root-canonicalization requires --output-md");
    }
    return options;
}

int RootCanonicalizationAuditCommand(
    const RootCanonicalizationAuditOptions& options) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(options.spec_path);
    const pmg::PreparedMotionSpaces prepared = pmg::PrepareMotionSpaces(spec);
    const std::vector<RootCanonicalizationAuditRow> rows =
        BuildRootCanonicalizationAudit(spec, prepared);
    WriteRootCanonicalizationCsv(options.output_csv, rows);
    WriteRootCanonicalizationMarkdown(options.output_md, options, rows);
    std::cout << "root_canonicalization_rows=" << rows.size() << "\n";
    std::cout << "root_canonicalization_csv=" << options.output_csv << "\n";
    std::cout << "root_canonicalization_md=" << options.output_md << "\n";
    return 0;
}

int CyclicRecutSearchCommand(
    const CyclicRecutSearchOptions& options,
    const std::string& command_line) {
    if (options.output_csv.empty()) {
        throw std::runtime_error(
            "--search-cyclic-recuts requires --output-csv");
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--search-cyclic-recuts requires --output-md");
    }
    if (options.min_window_frames < 3) {
        throw std::runtime_error(
            "--min-window-frames must be at least 3");
    }
    if (options.max_window_frames < options.min_window_frames) {
        throw std::runtime_error(
            "--max-window-frames must be >= --min-window-frames");
    }
    if (options.top_per_clip <= 0) {
        throw std::runtime_error("--top-per-clip must be positive");
    }

    std::vector<CyclicRecutCandidate> candidates;
    const pmg::CyclicContinuityConfig config;

    for (const std::string& clip_file : options.clip_files) {
        const std::filesystem::path bvh_path =
            std::filesystem::path(options.bvh_dir) / clip_file;
        const pmg::BvhData bvh = pmg::BvhLoader::Load(bvh_path.string());
        if (ResolveJointIndex(bvh.skeleton, options.cycle_joint) < 0) {
            throw std::runtime_error(
                "--search-cyclic-recuts: unknown cycle joint '" +
                options.cycle_joint + "' in " + clip_file);
        }

        const pmg::CyclicContinuityContext context =
            CyclicContextForRawClip(bvh.skeleton, bvh.clip, options);

        const int max_window =
            std::min(options.max_window_frames, bvh.clip.NumFrames());
        for (int start_frame = 0; start_frame < bvh.clip.NumFrames();
             ++start_frame) {
            for (int num_frames = options.min_window_frames;
                 num_frames <= max_window; ++num_frames) {
                const int end_frame = start_frame + num_frames - 1;
                if (end_frame >= bvh.clip.NumFrames()) {
                    break;
                }

                const pmg::MotionClip recut =
                    CropClipInclusive(bvh.clip, start_frame, end_frame);
                CyclicRecutCandidate candidate;
                candidate.clip_file = clip_file;
                candidate.start_frame = start_frame;
                candidate.end_frame = end_frame;
                candidate.num_frames = num_frames;
                candidate.record =
                    pmg::MeasureCyclicContinuity(
                        bvh.skeleton, recut, context, config);
                candidate.score =
                    ThresholdNormalizedRecutScore(candidate.record, config);
                candidates.push_back(std::move(candidate));
            }
        }
    }

    WriteCyclicRecutSearchCsv(options.output_csv, candidates);
    WriteCyclicRecutSearchMarkdown(
        options.output_md, options, command_line, candidates);

    std::cout << "cyclic_recut_candidates=" << candidates.size() << "\n";
    std::cout << "cyclic_recut_csv=" << options.output_csv << "\n";
    std::cout << "cyclic_recut_md=" << options.output_md << "\n";
    return 0;
}

int CyclicRecutEvaluateCommand(
    const CyclicRecutEvaluateOptions& options,
    const std::string& command_line) {
    if (options.output_cyclic_csv.empty()) {
        throw std::runtime_error(
            "--evaluate-cyclic-recuts requires --output-cyclic-csv");
    }
    if (options.output_edge_csv.empty()) {
        throw std::runtime_error(
            "--evaluate-cyclic-recuts requires --output-edge-csv");
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--evaluate-cyclic-recuts requires --output-md");
    }
    if (options.frames_per_second <= 0.0f) {
        throw std::runtime_error("--fps must be positive");
    }
    if (options.min_contact_frames <= 0) {
        throw std::runtime_error("--min-contact-frames must be positive");
    }

    std::optional<pmg::Skeleton> skeleton;
    pmg::ParametricMotionSpace walk_space("walk", 1);
    pmg::ParametricMotionSpace jog_space("jog", 1);
    const std::vector<NamedRecutWindow> recut_windows =
        options.recut_windows.empty()
            ? DefaultGroupBRecuts()
            : options.recut_windows;
    for (const NamedRecutWindow& window : recut_windows) {
        const std::filesystem::path bvh_path =
            std::filesystem::path(options.bvh_dir) / window.clip_file;
        const pmg::BvhData bvh = pmg::BvhLoader::Load(bvh_path.string());
        if (!skeleton.has_value()) {
            skeleton = bvh.skeleton;
        } else {
            pmg::RequireSkeletonCompatible(
                *skeleton, bvh.skeleton, "--evaluate-cyclic-recuts");
        }

        pmg::MotionClip recut =
            CropClipInclusive(bvh.clip, window.start_frame, window.end_frame);
        recut.name = window.clip_file + "_recut_" +
                     std::to_string(window.start_frame) + "_" +
                     std::to_string(window.end_frame);
        if (IsWalkRecut(window.clip_file)) {
            walk_space.AddExample(
                ParameterForGroupBRecut(window.clip_file), std::move(recut));
        } else {
            jog_space.AddExample(
                ParameterForGroupBRecut(window.clip_file), std::move(recut));
        }
    }
    if (!skeleton.has_value()) {
        throw std::runtime_error(
            "--evaluate-cyclic-recuts: no candidate clips loaded");
    }

    const std::vector<std::string> contact_joint_names =
        SplitCommaList(options.contact_joints_csv);
    const std::vector<int> contact_joint_indices = ResolveContactJointIndices(
        *skeleton, contact_joint_names, "--evaluate-cyclic-recuts");

    pmg::ContactDetectionSettings walk_settings =
        pmg::EstimateContactSettings(
            *skeleton, walk_space.Examples().front().clip,
            contact_joint_indices);
    walk_settings.min_contact_frames = options.min_contact_frames;
    pmg::ContactDetectionSettings jog_settings =
        pmg::EstimateContactSettings(
            *skeleton, jog_space.Examples().front().clip,
            contact_joint_indices);
    jog_settings.min_contact_frames = options.min_contact_frames;

    PrepareCandidateSpace(
        walk_space, *skeleton, contact_joint_indices, walk_settings,
        /*dtw_refine=*/true, {pmg::ParameterMetric::kTurnRate},
        options.frames_per_second);
    PrepareCandidateSpace(
        jog_space, *skeleton, contact_joint_indices, jog_settings,
        /*dtw_refine=*/false, {}, options.frames_per_second);

    pmg::CyclicContinuityContext walk_context;
    pmg::CyclicContinuityContext jog_context;
    for (std::size_t joint = 0; joint < contact_joint_names.size(); ++joint) {
        std::string lower_name = contact_joint_names[joint];
        std::transform(
            lower_name.begin(), lower_name.end(), lower_name.begin(),
            [](unsigned char character) {
                return static_cast<char>(std::tolower(character));
            });
        if (lower_name.find("left") != std::string::npos) {
            walk_context.left_foot_joint = contact_joint_indices[joint];
            jog_context.left_foot_joint = contact_joint_indices[joint];
        } else if (lower_name.find("right") != std::string::npos) {
            walk_context.right_foot_joint = contact_joint_indices[joint];
            jog_context.right_foot_joint = contact_joint_indices[joint];
        }
    }
    walk_context.contact_settings = walk_settings;
    jog_context.contact_settings = jog_settings;

    std::vector<CyclicAuditRow> cyclic_rows;
    const auto add_rows =
        [&](const std::string& node_name,
            const pmg::ParametricMotionSpace& space,
            const pmg::CyclicContinuityContext& context) {
        for (const pmg::ExampleMotion& example : space.Examples()) {
            CyclicAuditRow row;
            row.node = node_name;
            row.sample_type = "recut_authored";
            row.parameter = example.parameter;
            row.source_clip = example.clip.name;
            row.num_frames = example.clip.NumFrames();
            row.record =
                pmg::MeasureCyclicContinuity(
                    *skeleton, example.clip, context);
            cyclic_rows.push_back(std::move(row));
        }
        for (const pmg::ParameterVector& parameter :
             GeneratedAuditParameters(space)) {
            const pmg::MotionClip generated =
                space.GenerateClip(parameter, options.frames_per_second);
            CyclicAuditRow row;
            row.node = node_name;
            row.sample_type = "recut_generated";
            row.parameter = parameter;
            row.source_clip = "";
            row.num_frames = generated.NumFrames();
            row.record =
                pmg::MeasureCyclicContinuity(
                    *skeleton, generated, context);
            cyclic_rows.push_back(std::move(row));
        }
    };
    add_rows("walk", walk_space, walk_context);
    add_rows("jog", jog_space, jog_context);

    std::vector<CandidateEdgeReportRow> edge_rows;
    const auto build_edge =
        [&](const std::string& source_name,
            const std::string& target_name,
            const pmg::ParametricMotionSpace& source_space,
            const pmg::ParametricMotionSpace& target_space,
            int source_index,
            int target_index) {
        const pmg::PmgBuilderConfig config =
            CandidateEdgeConfig(source_name, target_name);
        const pmg::EdgeBuildResult result =
            pmg::PmgBuilder::BuildEdgeWithReport(
                *skeleton, source_index, target_index,
                source_space, target_space, config);
        edge_rows.push_back(
            SummarizeCandidateEdge(
                source_name + "->" + target_name, result));
    };
    build_edge("walk", "walk", walk_space, walk_space, 0, 0);
    build_edge("walk", "jog", walk_space, jog_space, 0, 1);
    build_edge("jog", "walk", jog_space, walk_space, 1, 0);
    build_edge("jog", "jog", jog_space, jog_space, 1, 1);

    WriteCyclicAuditCsv(options.output_cyclic_csv, cyclic_rows);
    WriteCandidateEdgeCsv(options.output_edge_csv, edge_rows);
    WriteCyclicRecutEvaluationMarkdown(
        options.output_md, options, command_line, cyclic_rows, edge_rows);

    const int strong_count = static_cast<int>(
        std::count_if(
            cyclic_rows.begin(), cyclic_rows.end(),
            [](const CyclicAuditRow& row) {
                return row.record.classification ==
                       pmg::CyclicContinuityClassification::kStrong;
            }));
    const int created_edges = static_cast<int>(
        std::count_if(
            edge_rows.begin(), edge_rows.end(),
            [](const CandidateEdgeReportRow& row) {
                return row.created;
            }));
    std::cout << "cyclic_recut_eval_rows=" << cyclic_rows.size() << "\n";
    std::cout << "cyclic_recut_eval_strong=" << strong_count << "\n";
    std::cout << "cyclic_recut_eval_edges_created=" << created_edges << "\n";
    std::cout << "cyclic_recut_eval_cyclic_csv="
              << options.output_cyclic_csv << "\n";
    std::cout << "cyclic_recut_eval_edge_csv="
              << options.output_edge_csv << "\n";
    std::cout << "cyclic_recut_eval_md=" << options.output_md << "\n";
    return 0;
}

pmg::PmgBuilderConfig EffectiveEdgeConfig(
    const pmg::GraphSpecEdge& edge,
    const ValidateGraphOptions& options) {
    pmg::PmgBuilderConfig config =
        edge.has_build_config ? edge.build_config : options.builder;

    // Explicit CLI values are diagnostic overrides. Unspecified values retain
    // the authored per-edge config so validation matches artifact construction.
    if (options.tgood_set) {
        config.good_transition_threshold =
            options.builder.good_transition_threshold;
    }
    if (options.tbad_set) {
        config.bad_transition_threshold =
            options.builder.bad_transition_threshold;
    }
    if (options.source_samples_set) {
        config.source_sample_count = options.builder.source_sample_count;
    }
    if (options.target_samples_set) {
        config.target_sample_count = options.builder.target_sample_count;
    }
    if (options.seed_set) {
        config.seed = options.builder.seed;
    }
    return config;
}

void PrintEffectiveEdgeConfig(const pmg::PmgBuilderConfig& config) {
    std::cout << "effective_edge_config:"
              << " TGOOD=" << config.good_transition_threshold
              << " TBAD=" << config.bad_transition_threshold
              << " source_samples=" << config.source_sample_count
              << " target_samples=" << config.target_sample_count
              << " window_size=" << config.distance_grid.window_size
              << " convention="
              << pmg::TransitionWindowConventionName(
                     config.transition_convention)
              << "\n";
}

struct EdgeQuality {
    bool created = false;
    int samples = 0;
    float mean_good_fraction = 0.0f;
    float mean_min_distance = 0.0f;
    float mean_p25_distance = 0.0f;
    float mean_median_distance = 0.0f;
    float mean_box_volume_fraction = 0.0f;
};

EdgeQuality MeasureEdgeQuality(
    const pmg::EdgeBuildResult& result,
    const pmg::ParametricMotionSpace& target_space) {
    EdgeQuality quality;
    quality.created = result.report.edge_created;
    quality.samples = static_cast<int>(result.edge.samples.size());

    float good_fraction_sum = 0.0f;
    float min_distance_sum = 0.0f;
    float p25_distance_sum = 0.0f;
    float median_distance_sum = 0.0f;
    int report_count = 0;
    for (const pmg::SourceSampleBuildReport& report : result.report.source_reports) {
        const int total = report.good_count + report.neutral_count + report.bad_count;
        if (total > 0) {
            good_fraction_sum += static_cast<float>(report.good_count) / static_cast<float>(total);
        }
        min_distance_sum += report.min_distance;
        p25_distance_sum += report.p25_distance;
        median_distance_sum += report.median_distance;
        ++report_count;
    }
    if (report_count > 0) {
        quality.mean_good_fraction = good_fraction_sum / static_cast<float>(report_count);
        quality.mean_min_distance = min_distance_sum / static_cast<float>(report_count);
        quality.mean_p25_distance = p25_distance_sum / static_cast<float>(report_count);
        quality.mean_median_distance = median_distance_sum / static_cast<float>(report_count);
    }

    const std::vector<float> domain_min = target_space.MinParameter();
    const std::vector<float> domain_max = target_space.MaxParameter();
    float volume_sum = 0.0f;
    for (const pmg::TransitionSample& sample : result.edge.samples) {
        float volume = 1.0f;
        for (std::size_t dim = 0; dim < domain_min.size(); ++dim) {
            const float domain_extent = std::max(domain_max[dim] - domain_min[dim], 1.0e-6f);
            const float box_extent =
                sample.target_parameter_box.max_corner[dim] -
                sample.target_parameter_box.min_corner[dim];
            volume *= std::clamp(box_extent / domain_extent, 0.0f, 1.0f);
        }
        volume_sum += volume;
    }
    if (quality.samples > 0) {
        quality.mean_box_volume_fraction = volume_sum / static_cast<float>(quality.samples);
    }
    return quality;
}

void PrintEdgeQuality(const char* label, const EdgeQuality& quality) {
    std::cout << label << "_edge_created=" << (quality.created ? 1 : 0) << "\n";
    std::cout << label << "_samples=" << quality.samples << "\n";
    std::cout << label << "_mean_good_fraction=" << quality.mean_good_fraction << "\n";
    std::cout << label << "_mean_min_distance=" << quality.mean_min_distance << "\n";
    std::cout << label << "_mean_p25_distance=" << quality.mean_p25_distance << "\n";
    std::cout << label << "_mean_median_distance=" << quality.mean_median_distance << "\n";
    std::cout << label << "_mean_box_volume_fraction=" << quality.mean_box_volume_fraction << "\n";
}

int ValidateGraphCommand(const ValidateGraphOptions& options) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(options.spec_path);
    pmg::MotionSpacePreparationConfig preparation_config;
    preparation_config.default_cycle_joint = options.cycle_joint;
    preparation_config.default_contact_joints =
        SplitCommaList(options.contact_joints_csv);
    preparation_config.default_min_contact_frames =
        options.min_contact_frames;
    preparation_config.default_dtw_refine = true;
    preparation_config.calibration_frames_per_second =
        options.builder.generated_frames_per_second;
    const pmg::PreparedMotionSpaces prepared =
        pmg::PrepareMotionSpaces(spec, preparation_config);

    // Compare authored inputs against the exact production motion spaces
    // consumed by artifact edge construction.
    std::map<std::string, pmg::ParametricMotionSpace> naive_spaces;
    std::map<std::string, pmg::ParametricMotionSpace> production_spaces;
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        const pmg::PreparedMotionSpace& prepared_node =
            prepared.Node(node.name);
        RequireMatchingProductionRegistration(
            prepared_node.registration, node.name, options.cycle_joint,
            options.cycle_joint_set,
            SplitCommaList(options.contact_joints_csv),
            options.contact_joints_set, options.min_contact_frames,
            options.min_contact_frames_set, "--validate-graph");
        naive_spaces.emplace(node.name, prepared_node.authored);
        production_spaces.emplace(node.name, prepared_node.production);
    }

    bool failed = false;
    auto fail_if = [&failed](bool condition, const std::string& message) {
        if (condition) {
            std::cout << "ASSERT FAIL: " << message << "\n";
            failed = true;
        }
    };

    for (const pmg::GraphSpecEdge& edge : spec.edges) {
        std::cout << "=== edge " << edge.source_node << " -> " << edge.target_node << " ===\n";
        const pmg::PmgBuilderConfig edge_config =
            EffectiveEdgeConfig(edge, options);
        PrintEffectiveEdgeConfig(edge_config);

        const pmg::EdgeBuildResult naive_result = pmg::PmgBuilder::BuildEdgeWithReport(
            prepared.skeleton, 0, 0, naive_spaces.at(edge.source_node),
            naive_spaces.at(edge.target_node), edge_config);
        const pmg::EdgeBuildResult production_result =
            pmg::PmgBuilder::BuildEdgeWithReport(
                prepared.skeleton, 0, 0,
                production_spaces.at(edge.source_node),
                production_spaces.at(edge.target_node), edge_config);

        const EdgeQuality naive = MeasureEdgeQuality(
            naive_result, naive_spaces.at(edge.target_node));
        const EdgeQuality production = MeasureEdgeQuality(
            production_result, production_spaces.at(edge.target_node));

        PrintEdgeQuality("naive", naive);
        PrintEdgeQuality("production", production);
        if (!production_result.report.edge_created) {
            std::cout << "production_reject_reason="
                      << production_result.report.reject_reason
                      << "\n";
        }

        fail_if(!production.created, "production edge not created");
        if (options.min_edge_samples >= 0) {
            fail_if(production.samples < options.min_edge_samples,
                    "production_samples=" + std::to_string(production.samples) + " < " +
                        std::to_string(options.min_edge_samples));
        }
        if (options.min_good_fraction >= 0.0f) {
            fail_if(production.mean_good_fraction < options.min_good_fraction,
                    "production_mean_good_fraction=" +
                        std::to_string(production.mean_good_fraction) + " < " +
                        std::to_string(options.min_good_fraction));
        }
        // Distribution comparisons are only meaningful when both builds
        // covered the same source samples (a rejected build aborts early).
        if (options.assert_no_regression) {
            fail_if(!naive.created || !production.created,
                    "no-regression comparison needs both builds to complete; "
                    "loosen --tgood/--tbad");
            if (naive.created && production.created) {
                fail_if(production.mean_good_fraction < naive.mean_good_fraction * 0.95f,
                        "production preparation shrank GOOD fraction: " +
                            std::to_string(production.mean_good_fraction) + " < " +
                            std::to_string(naive.mean_good_fraction));
            }
        }
        float max_distance_ratio =
            options.max_preparation_min_distance_ratio;
        if (options.assert_no_regression && max_distance_ratio < 0.0f) {
            max_distance_ratio = 1.05f;
        }
        if (max_distance_ratio >= 0.0f) {
            fail_if(!naive.created || !production.created,
                    "preparation-distance comparison needs both builds to complete; "
                    "loosen --tgood/--tbad");
            if (naive.created && production.created) {
                // Gate only on the BEST transition per source sample: that is
                // what the runtime schedules. Median/p25 remain report-only.
                fail_if(
                    production.mean_min_distance >
                        naive.mean_min_distance * max_distance_ratio,
                    "production/naive mean-min-distance ratio exceeded " +
                        std::to_string(max_distance_ratio) + ": " +
                        std::to_string(production.mean_min_distance) + " / " +
                        std::to_string(naive.mean_min_distance));
            }
        }
    }

    std::cout << (failed ? "RESULT=FAIL" : "RESULT=PASS") << "\n";
    return failed ? 1 : 0;
}

ValidateGraphOptions ParseValidateGraphOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--validate-graph needs <spec>");
    }
    ValidateGraphOptions options;
    options.spec_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--contact-joints") {
            options.contact_joints_csv = require_value("--contact-joints");
            options.contact_joints_set = true;
        } else if (option == "--cycle-joint") {
            options.cycle_joint = require_value("--cycle-joint");
            options.cycle_joint_set = true;
        } else if (option == "--min-contact-frames") {
            options.min_contact_frames = std::stoi(require_value("--min-contact-frames"));
            options.min_contact_frames_set = true;
        } else if (option == "--tgood") {
            options.builder.good_transition_threshold = std::stof(require_value("--tgood"));
            options.tgood_set = true;
        } else if (option == "--tbad") {
            options.builder.bad_transition_threshold = std::stof(require_value("--tbad"));
            options.tbad_set = true;
        } else if (option == "--source-samples") {
            options.builder.source_sample_count = std::stoi(require_value("--source-samples"));
            options.source_samples_set = true;
        } else if (option == "--target-samples") {
            options.builder.target_sample_count = std::stoi(require_value("--target-samples"));
            options.target_samples_set = true;
        } else if (option == "--seed") {
            options.builder.seed =
                static_cast<unsigned int>(std::stoul(require_value("--seed")));
            options.seed_set = true;
        } else if (option == "--min-edge-samples") {
            options.min_edge_samples = std::stoi(require_value("--min-edge-samples"));
        } else if (option == "--min-good-fraction") {
            options.min_good_fraction = std::stof(require_value("--min-good-fraction"));
        } else if (option == "--assert-no-regression") {
            options.assert_no_regression = true;
        } else if (option == "--max-preparation-distance-ratio") {
            options.max_preparation_min_distance_ratio =
                std::stof(require_value("--max-preparation-distance-ratio"));
            if (!std::isfinite(options.max_preparation_min_distance_ratio) ||
                options.max_preparation_min_distance_ratio <= 0.0f) {
                throw std::runtime_error(
                    "--max-preparation-distance-ratio must be positive");
            }
        } else {
            throw std::runtime_error("unknown validate-graph option '" + option + "'");
        }
    }
    return options;
}

SpaceSweepOptions ParseSpaceSweepOptions(int argc, char** argv) {
    if (argc < 4) {
        throw std::runtime_error("--space-sweep needs <spec> <node>");
    }
    SpaceSweepOptions options;
    options.spec_path = argv[2];
    options.node_name = argv[3];
    for (int index = 4; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--contact-joints") {
            options.contact_joints_csv = require_value("--contact-joints");
            options.contact_joints_set = true;
        } else if (option == "--cycle-joint") {
            options.cycle_joint = require_value("--cycle-joint");
            options.cycle_joint_set = true;
        } else if (option == "--sweep-steps") {
            options.sweep_steps = std::stoi(require_value("--sweep-steps"));
        } else if (option == "--min-contact-frames") {
            options.min_contact_frames = std::stoi(require_value("--min-contact-frames"));
            options.min_contact_frames_set = true;
        } else if (option == "--min-contacts") {
            options.min_contacts = std::stoi(require_value("--min-contacts"));
        } else if (option == "--max-foot-slide") {
            options.max_foot_slide = std::stof(require_value("--max-foot-slide"));
        } else if (option == "--max-adjacent-step") {
            options.max_adjacent_step = std::stof(require_value("--max-adjacent-step"));
        } else if (option == "--assert-no-regression") {
            options.assert_no_regression = true;
        } else if (option == "--dtw-refine") {
            options.dtw_refine = true;
        } else if (option == "--foot-lock") {
            options.foot_lock = true;
        } else {
            throw std::runtime_error("unknown space-sweep option '" + option + "'");
        }
    }
    return options;
}

CyclicAuditOptions ParseCyclicAuditOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--audit-cyclic-continuity needs <spec>");
    }
    CyclicAuditOptions options;
    options.spec_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(
                    std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--output-csv") {
            options.output_csv = require_value("--output-csv");
        } else if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else if (option == "--fps") {
            options.frames_per_second = std::stof(require_value("--fps"));
        } else {
            throw std::runtime_error(
                "unknown audit-cyclic-continuity option '" + option + "'");
        }
    }
    if (options.output_csv.empty()) {
        throw std::runtime_error(
            "--audit-cyclic-continuity requires --output-csv");
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-cyclic-continuity requires --output-md");
    }
    if (options.frames_per_second <= 0.0f) {
        throw std::runtime_error("--fps must be positive");
    }
    return options;
}

CyclicRecutSearchOptions ParseCyclicRecutSearchOptions(int argc, char** argv) {
    CyclicRecutSearchOptions options;
    for (int index = 2; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(
                    std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--bvh-dir") {
            options.bvh_dir = require_value("--bvh-dir");
        } else if (option == "--clips") {
            options.clip_files = SplitCommaList(require_value("--clips"));
        } else if (option == "--cycle-joint") {
            options.cycle_joint = require_value("--cycle-joint");
        } else if (option == "--contact-joints") {
            options.contact_joints_csv = require_value("--contact-joints");
        } else if (option == "--min-contact-frames") {
            options.min_contact_frames =
                std::stoi(require_value("--min-contact-frames"));
        } else if (option == "--min-window-frames") {
            options.min_window_frames =
                std::stoi(require_value("--min-window-frames"));
        } else if (option == "--max-window-frames") {
            options.max_window_frames =
                std::stoi(require_value("--max-window-frames"));
        } else if (option == "--top-per-clip") {
            options.top_per_clip = std::stoi(require_value("--top-per-clip"));
        } else if (option == "--output-csv") {
            options.output_csv = require_value("--output-csv");
        } else if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else {
            throw std::runtime_error(
                "unknown search-cyclic-recuts option '" + option + "'");
        }
    }
    if (options.clip_files.empty()) {
        throw std::runtime_error("--clips must name at least one BVH file");
    }
    if (options.min_contact_frames <= 0) {
        throw std::runtime_error("--min-contact-frames must be positive");
    }
    return options;
}

CyclicRecutEvaluateOptions ParseCyclicRecutEvaluateOptions(
    int argc,
    char** argv) {
    CyclicRecutEvaluateOptions options;
    for (int index = 2; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(
                    std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--bvh-dir") {
            options.bvh_dir = require_value("--bvh-dir");
        } else if (option == "--cycle-joint") {
            options.cycle_joint = require_value("--cycle-joint");
        } else if (option == "--contact-joints") {
            options.contact_joints_csv = require_value("--contact-joints");
        } else if (option == "--min-contact-frames") {
            options.min_contact_frames =
                std::stoi(require_value("--min-contact-frames"));
        } else if (option == "--fps") {
            options.frames_per_second = std::stof(require_value("--fps"));
        } else if (option == "--recuts") {
            options.recut_windows = ParseRecutWindows(require_value("--recuts"));
        } else if (option == "--output-cyclic-csv") {
            options.output_cyclic_csv =
                require_value("--output-cyclic-csv");
        } else if (option == "--output-edge-csv") {
            options.output_edge_csv = require_value("--output-edge-csv");
        } else if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else {
            throw std::runtime_error(
                "unknown evaluate-cyclic-recuts option '" + option + "'");
        }
    }
    return options;
}

struct SimplexAuditOptions {
    std::string pmg_path;
    std::string node_name;
    std::string output_csv;
    std::string output_md;
    int samples_per_axis = 10;
};

SimplexAuditOptions ParseSimplexAuditOptions(int argc, char** argv) {
    SimplexAuditOptions options;
    if (argc < 4) {
        throw std::runtime_error("Usage: pmg_cli --audit-parameter-node graph.pmg node_name --output-csv ... --output-md ... [--samples N]");
    }
    options.pmg_path = argv[2];
    options.node_name = argv[3];
    for (int index = 4; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
            return argv[++index];
        };
        if (option == "--output-csv") options.output_csv = require_value("--output-csv");
        else if (option == "--output-md") options.output_md = require_value("--output-md");
        else if (option == "--samples") options.samples_per_axis = std::stoi(require_value("--samples"));
        else throw std::runtime_error("Unknown option: " + option);
    }
    return options;
}

int SimplexAuditCommand(const SimplexAuditOptions& options) {
    if (options.output_csv.empty() || options.output_md.empty()) {
        throw std::runtime_error("--audit-parameter-node requires --output-csv and --output-md");
    }
    const pmg::BuiltPmgArtifact artifact = pmg::LoadPmgArtifactText(options.pmg_path);
    int node_idx = -1;
    for (int i = 0; i < artifact.graph.NumNodes(); ++i) {
        if (artifact.graph.Node(i).name == options.node_name) {
            node_idx = i;
            break;
        }
    }
    if (node_idx < 0) throw std::runtime_error("Node not found: " + options.node_name);

    const pmg::ParametricMotionSpace& space = artifact.graph.Node(node_idx).motion_space;
    if (!space.HasExplicitParameterSupport()) {
        throw std::runtime_error("Node has no explicit parameter support");
    }
    const pmg::ParameterSupport& support = *space.ExplicitSupport();
    
    const pmg::ParameterDomain domain = space.Domain();
    
    std::ofstream csv(options.output_csv);
    // Foot contact/sliding proxy intentionally omitted until a formal contact metric is available.
    csv << "p0,p1,duration,root_disp,speed,net_heading,curvature,projected\n";
    
    std::vector<pmg::ParameterVector> grid;
    if (domain.Dimension() == 1) {
        for (int i = 0; i < options.samples_per_axis; ++i) {
            float t = i / std::max(1.0f, float(options.samples_per_axis - 1));
            pmg::ParameterVector pt = {domain.Bounds().min_corner[0] + t * (domain.Bounds().max_corner[0] - domain.Bounds().min_corner[0])};
            if (support.Contains(pt)) grid.push_back(pt);
        }
    } else if (domain.Dimension() == 2) {
        for (int i = 0; i < options.samples_per_axis; ++i) {
            float ty = i / std::max(1.0f, float(options.samples_per_axis - 1));
            float y = domain.Bounds().min_corner[1] + ty * (domain.Bounds().max_corner[1] - domain.Bounds().min_corner[1]);
            for (int j = 0; j < options.samples_per_axis; ++j) {
                float tx = j / std::max(1.0f, float(options.samples_per_axis - 1));
                float x = domain.Bounds().min_corner[0] + tx * (domain.Bounds().max_corner[0] - domain.Bounds().min_corner[0]);
                pmg::ParameterVector pt = {x, y};
                if (support.Contains(pt)) grid.push_back(pt);
            }
        }
    } else {
        throw std::runtime_error("Only 1D and 2D spaces supported");
    }
    
    for (const auto& p : grid) {
        bool projected = !support.Contains(p);
        pmg::ParameterVector sp = support.Project(p);
        float duration = space.BlendedDurationSeconds(sp);
        pmg::MotionClip clip = space.GenerateClip(sp, artifact.metadata.frames_per_second);
        float speed = pmg::MeasureParameterMetric(pmg::ParameterMetric::kTravelSpeed, clip);
        float turn_rate = pmg::MeasureParameterMetric(pmg::ParameterMetric::kTurnRate, clip);
        float heading = turn_rate * duration;
        float disp = speed * duration;
        float curvature = disp > 1e-4f ? heading / disp : 0.0f;
        
        csv << (p.size() > 0 ? p[0] : 0) << "," 
            << (p.size() > 1 ? p[1] : 0) << ","
            << duration << "," << disp << "," << speed << "," << heading << "," << curvature << "," << projected << "\n";
    }
    
    std::ofstream md(options.output_md);
    md << "# Parameter Node Audit: " << options.node_name << "\n\n";
    md << "Audit complete. " << grid.size() << " samples taken.\n\n";
    md << "**Curvature Definition**: curvature = net_heading / max(root_disp, 1e-4)\n\n";
    md << "The audit samples from the node's declared support; therefore projected=false is expected for normal samples. Projection becomes relevant only when evaluating externally requested parameters outside support.\n";
    return 0;
}

struct InventoryBvhOptions {
    std::string bvh_dir;
    std::string pmg_path;
    std::string output_csv;
};

InventoryBvhOptions ParseInventoryBvhOptions(int argc, char** argv) {
    InventoryBvhOptions options;
    if (argc < 5) {
        throw std::runtime_error("Usage: pmg_cli --inventory-bvh <dir> <pmg> <csv>");
    }
    options.bvh_dir = argv[2];
    options.pmg_path = argv[3];
    options.output_csv = argv[4];
    return options;
}

int InventoryBvhCommand(const InventoryBvhOptions& options) {
    const pmg::BuiltPmgArtifact artifact = pmg::LoadPmgArtifactText(options.pmg_path);
    std::ofstream csv(options.output_csv);
    csv << "filename,compatible,frames,duration,root_disp,speed,net_heading,curvature,loopability,locomotion,idle,start,stop,turn,jump,unknown\n";
    
    for (const auto& entry : std::filesystem::directory_iterator(options.bvh_dir)) {
        if (entry.path().extension() != ".bvh") continue;
        std::string filename = entry.path().filename().string();
        try {
            pmg::BvhData bvh = pmg::BvhLoader::Load(entry.path().string());
            bool compat = pmg::CheckSkeletonCompatibility(artifact.skeleton, bvh.skeleton).compatible;
            int frames = bvh.clip.NumFrames();
            float duration = bvh.clip.DurationSeconds();
            float speed = pmg::MeasureParameterMetric(pmg::ParameterMetric::kTravelSpeed, bvh.clip);
            float heading_rate = pmg::MeasureParameterMetric(pmg::ParameterMetric::kTurnRate, bvh.clip);
            float heading = heading_rate * duration;
            float root_disp = speed * duration;
            float curvature = root_disp > 1e-4f ? heading / root_disp : 0.0f;
            
            float loopability = 0.0f;
            if (frames > 0) {
                pmg::PointCloud p1 = pmg::MotionDistance::BuildPointCloudFromFirstFrame(bvh.skeleton, bvh.clip, 0, 1);
                pmg::PointCloud p2 = pmg::MotionDistance::BuildPointCloudFromFirstFrame(bvh.skeleton, bvh.clip, frames-1, 1);
                loopability = pmg::MotionDistance::AlignedPointCloudDistance(p1, p2).distance;
            }
            
            std::string lower = filename;
            for (char& c : lower) c = std::tolower(c);
            
            bool is_locomotion = lower.find("walk") != std::string::npos || lower.find("jog") != std::string::npos || lower.find("run") != std::string::npos || lower.find("sneak") != std::string::npos || lower.find("strut") != std::string::npos;
            bool is_idle = lower.find("stand") != std::string::npos || lower.find("idle") != std::string::npos;
            bool is_start = lower.find("start") != std::string::npos || lower.find("to") != std::string::npos;
            bool is_stop = lower.find("stop") != std::string::npos;
            bool is_turn = lower.find("turn") != std::string::npos || lower.find("aboutface") != std::string::npos;
            bool is_jump = lower.find("jump") != std::string::npos || lower.find("vault") != std::string::npos;
            bool is_unknown = !is_locomotion && !is_idle && !is_start && !is_stop && !is_turn && !is_jump;
            
            csv << filename << "," << compat << "," << frames << "," << duration << "," 
                << root_disp << "," << speed << "," << heading << "," << curvature << "," 
                << loopability << "," << is_locomotion << "," << is_idle << "," << is_start << "," 
                << is_stop << "," << is_turn << "," << is_jump << "," << is_unknown << "\n";
        } catch (...) {
            csv << filename << ",0,0,0,0,0,0,0,0,0,0,0,0,0,0,1\n";
        }
    }
    return 0;
}

}  // namespace

namespace pmgcli {

std::optional<int> TryRunDiagnosticCommand(int argc, char** argv) {
    const std::string command = argc > 1 ? argv[1] : "";
    if (command == "--space-sweep" && argc >= 4) {
        return SpaceSweepCommand(ParseSpaceSweepOptions(argc, argv));
    }
    if (command == "--validate-graph" && argc >= 3) {
        return ValidateGraphCommand(ParseValidateGraphOptions(argc, argv));
    }
    if (command == "--audit-cyclic-continuity" && argc >= 3) {
        return CyclicAuditCommand(
            ParseCyclicAuditOptions(argc, argv),
            CommandLineString(argc, argv));
    }
    if (command == "--audit-root-canonicalization" && argc >= 3) {
        return RootCanonicalizationAuditCommand(
            ParseRootCanonicalizationAuditOptions(argc, argv));
    }
    if (command == "--search-cyclic-recuts") {
        return CyclicRecutSearchCommand(
            ParseCyclicRecutSearchOptions(argc, argv),
            CommandLineString(argc, argv));
    }
    if (command == "--evaluate-cyclic-recuts") {
        return CyclicRecutEvaluateCommand(
            ParseCyclicRecutEvaluateOptions(argc, argv),
            CommandLineString(argc, argv));
    }
    if (command == "--audit-simplex-node" || command == "--audit-parameter-node") {
        if (argc >= 4) {
            return SimplexAuditCommand(ParseSimplexAuditOptions(argc, argv));
        }
    }
    if (command == "--inventory-bvh" && argc >= 5) {
        return InventoryBvhCommand(ParseInventoryBvhOptions(argc, argv));
    }
    return std::nullopt;
}

}  // namespace pmgcli
