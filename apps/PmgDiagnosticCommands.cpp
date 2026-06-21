#include "PmgCommandModules.h"

#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/CyclicContinuity.h"
#include "pmg/FootLocking.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/GraphIo.h"
#include "pmg/GraphSpec.h"
#include "pmg/MotionDistance.h"
#include "pmg/MotionRegistration.h"
#include "pmg/MotionSpacePreparation.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/PoseBlend.h"
#include "pmg/RuntimeController.h"
#include "pmg/RootCanonicalization.h"
#include "pmg/SkeletonCompatibility.h"
#include "pmg/TransitionQuality.h"

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
#include <set>
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

        pmg::MotionClip clip = space.GenerateClip(parameter, frames_per_second);
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
        const int shared_frame_count = std::min(
            generated[step].NumFrames(), generated[step + 1].NumFrames());
        if (shared_frame_count <= 0) {
            throw std::runtime_error("MeasureSpaceSweep: generated clip must not be empty");
        }
        for (int frame = 0; frame < shared_frame_count; ++frame) {
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

    const float frames_per_second = naive_space.Examples().front().clip.frames_per_second;

    std::cout << "registered=yes\n";

    const SweepMetrics naive = MeasureSpaceSweep(
        naive_space, skeleton, contact_joints, settings, options.sweep_steps,
        frames_per_second, "naive");
    const SweepMetrics registered = MeasureSpaceSweep(
        registered_space, skeleton, contact_joints, settings, options.sweep_steps,
        frames_per_second, "registered");

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
            frames_per_second, "dtw");
    }

    // IK foot locking post-processes the best registered variant's clips.
    std::optional<SweepMetrics> locked;
    if (options.foot_lock) {
        std::cout << "foot_lock=yes\n";
        locked = MeasureSpaceSweep(
            best_space, skeleton, contact_joints, settings, options.sweep_steps,
            frames_per_second, "locked", /*foot_lock=*/true);
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

struct RegistrationPhaseAlignmentAuditOptions {
    std::string spec_path;
    std::string output_csv;
    std::string output_md;
};

struct RegistrationPhaseAlignmentClipRow {
    std::string node_name;
    std::string clip_name;
    std::string bvh_path;
    pmg::ParameterVector parameter;
    int frame_count = 0;
    float frames_per_second = 0.0f;
    pmg::RootStartSummary normalized;
    pmg::CyclicContinuityRecord seam;
    bool start_is_origin = false;
    bool heading_is_canonical = false;
    std::vector<int> source_phase_frames;
    std::vector<int> target_phase_frames;
    std::string conclusion;
};

struct RegistrationPhaseAlignmentPairRow {
    std::string node_name;
    std::string source_clip_name;
    std::string target_clip_name;
    std::string source_bvh_path;
    std::string target_bvh_path;
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector target_parameter;
    std::vector<int> source_phase_frames;
    std::vector<int> target_phase_frames;
    float root_trajectory_rmse = 0.0f;
    float phase_alignment_offset = 0.0f;
    float pose_seam_distance = 0.0f;
    std::string pose_seam_class;
    float velocity_seam_cost = 0.0f;
    float root_motion_cost = 0.0f;
    float foot_mismatch_ratio = -1.0f;
    bool safe_for_transition_sampling = false;
    bool jog_like_pair = false;
    std::string conclusion;
};

struct RegistrationPhaseAlignmentAuditData {
    std::vector<RegistrationPhaseAlignmentClipRow> clip_rows;
    std::vector<RegistrationPhaseAlignmentPairRow> pair_rows;
    std::string conclusion;
};

std::string LowercaseCopy(std::string text) {
    std::transform(
        text.begin(), text.end(), text.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return text;
}

std::vector<int> SamplePhaseFrames(
    int frame_count,
    float phase_start,
    float phase_end,
    int stride) {
    if (frame_count <= 0) {
        throw std::runtime_error("SamplePhaseFrames: frame_count must be positive");
    }
    if (stride < 1) {
        throw std::runtime_error("SamplePhaseFrames: stride must be >= 1");
    }
    if (phase_start < 0.0f || phase_end > 1.0f || phase_start > phase_end) {
        throw std::runtime_error("SamplePhaseFrames: invalid phase range");
    }
    const int last_frame = frame_count - 1;
    const int first = static_cast<int>(std::lround(phase_start * last_frame));
    const int last = static_cast<int>(std::lround(phase_end * last_frame));
    std::vector<int> frames;
    for (int frame = first; frame <= last; frame += stride) {
        frames.push_back(frame);
    }
    if (frames.empty()) {
        frames.push_back(first);
    }
    return frames;
}

std::string IntegerListCsv(const std::vector<int>& values) {
    std::ostringstream out;
    out << '"';
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ' ';
        }
        out << values[index];
    }
    out << '"';
    return out.str();
}

const pmg::GraphSpecEdge* FindSpecEdge(
    const pmg::GraphSpec& spec,
    const std::string& source_node,
    const std::string& target_node) {
    for (const pmg::GraphSpecEdge& edge : spec.edges) {
        if (edge.source_node == source_node && edge.target_node == target_node) {
            return &edge;
        }
    }
    return nullptr;
}

float WrapPhaseOffset(float phase_offset) {
    while (phase_offset > 0.5f) {
        phase_offset -= 1.0f;
    }
    while (phase_offset < -0.5f) {
        phase_offset += 1.0f;
    }
    return phase_offset;
}

float RootTrajectoryRmse(
    const pmg::MotionClip& source_clip,
    const pmg::MotionClip& target_clip) {
    constexpr int kPhaseSamples = 17;
    float squared_error_sum = 0.0f;
    for (int sample = 0; sample < kPhaseSamples; ++sample) {
        const float phase =
            static_cast<float>(sample) / static_cast<float>(kPhaseSamples - 1);
        const pmg::Pose source_pose = source_clip.SampleNormalizedPhase(phase);
        const pmg::Pose target_pose = target_clip.SampleNormalizedPhase(phase);
        const float dx = source_pose.root_position.x - target_pose.root_position.x;
        const float dz = source_pose.root_position.z - target_pose.root_position.z;
        squared_error_sum += dx * dx + dz * dz;
    }
    return std::sqrt(squared_error_sum / static_cast<float>(kPhaseSamples));
}

std::string PoseSeamClass(
    float distance,
    const pmg::PmgBuilderConfig& config) {
    if (!std::isfinite(distance)) {
        return "EMPTY";
    }
    if (distance <= config.good_transition_threshold) {
        return "GOOD";
    }
    if (distance >= config.bad_transition_threshold) {
        return "BAD";
    }
    return "NEUTRAL";
}

bool LooksJogLike(const std::string& clip_name) {
    return LowercaseCopy(clip_name).find("jog") != std::string::npos;
}

RegistrationPhaseAlignmentAuditData BuildRegistrationPhaseAlignmentAudit(
    const pmg::GraphSpec& spec,
    const pmg::PreparedMotionSpaces& prepared) {
    RegistrationPhaseAlignmentAuditData data;
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        const pmg::PreparedMotionSpace& prepared_node = prepared.Node(node.name);
        const pmg::GraphSpecEdge* self_edge =
            FindSpecEdge(spec, node.name, node.name);
        const pmg::PmgBuilderConfig builder_config =
            self_edge ? self_edge->build_config : pmg::PmgBuilderConfig{};
        const pmg::DistanceGridConfig& grid_config = builder_config.distance_grid;
        const pmg::CyclicContinuityContext cyclic_context =
            CyclicContextForNode(prepared.skeleton, prepared_node);
        const auto& examples = prepared_node.production.Examples();

        std::vector<const pmg::GraphSpecExample*> node_examples;
        node_examples.reserve(examples.size());
        for (const pmg::GraphSpecExample& example : spec.examples) {
            if (example.node_name == node.name) {
                node_examples.push_back(&example);
            }
        }
        if (node_examples.size() != examples.size()) {
            throw std::runtime_error(
                "--audit-registration-phase-alignment: prepared example count mismatch");
        }

        for (std::size_t example_index = 0; example_index < examples.size();
             ++example_index) {
            const pmg::ExampleMotion& example = examples[example_index];
            const pmg::MotionClip& clip = example.clip;
            const pmg::RootStartSummary normalized =
                pmg::SummarizeRootStart(clip);
            RegistrationPhaseAlignmentClipRow row;
            row.node_name = node.name;
            row.clip_name = std::filesystem::path(
                                node_examples[example_index]->bvh_path)
                                .filename()
                                .string();
            row.bvh_path = node_examples[example_index]->bvh_path;
            row.parameter = example.parameter;
            row.frame_count = clip.NumFrames();
            row.frames_per_second = clip.frames_per_second;
            row.normalized = normalized;
            row.seam = pmg::MeasureCyclicContinuity(
                prepared.skeleton, clip, cyclic_context);
            row.start_is_origin =
                HorizontalDistance(normalized.first_root_position) <=
                kRootOriginTolerance;
            row.heading_is_canonical =
                std::abs(normalized.first_heading_yaw) <=
                kRootHeadingTolerance;
            row.source_phase_frames = SamplePhaseFrames(
                clip.NumFrames(), grid_config.source_phase_start,
                grid_config.source_phase_end, grid_config.source_frame_stride);
            row.target_phase_frames = SamplePhaseFrames(
                clip.NumFrames(), grid_config.target_phase_start,
                grid_config.target_phase_end, grid_config.target_frame_stride);
            if (!row.start_is_origin || !row.heading_is_canonical) {
                row.conclusion = "FAIL_REGISTRATION_INCONSISTENT";
            } else if (row.seam.classification ==
                       pmg::CyclicContinuityClassification::kInsufficientData) {
                row.conclusion = "FAIL_CLIP_NOT_CYCLIC_ENOUGH";
            } else if (row.seam.classification !=
                       pmg::CyclicContinuityClassification::kStrong) {
                row.conclusion = "WARN_WEAK_CYCLE_SEAM";
            } else {
                row.conclusion = "PASS_CLIP_PHASE_ALIGNMENT";
            }
            data.clip_rows.push_back(std::move(row));
        }

        pmg::TransitionMetricConfig metric_config =
            builder_config.transition_metric;
        metric_config.contact_joint_indices = prepared_node.contact_joint_indices;
        metric_config.contact_settings = prepared_node.contact_settings;
        for (std::size_t source_index = 0; source_index < examples.size();
             ++source_index) {
            for (std::size_t target_index = source_index + 1;
                 target_index < examples.size();
                 ++target_index) {
                const pmg::ExampleMotion& source = examples[source_index];
                const pmg::ExampleMotion& target = examples[target_index];
                const pmg::OptimalTransition optimal =
                    pmg::MotionDistance::FindOptimalTransition(
                        prepared.skeleton, source.clip, target.clip,
                        grid_config);
                RegistrationPhaseAlignmentPairRow row;
                row.node_name = node.name;
                row.source_clip_name = std::filesystem::path(
                                           node_examples[source_index]->bvh_path)
                                           .filename()
                                           .string();
                row.target_clip_name = std::filesystem::path(
                                           node_examples[target_index]->bvh_path)
                                           .filename()
                                           .string();
                row.source_bvh_path = node_examples[source_index]->bvh_path;
                row.target_bvh_path = node_examples[target_index]->bvh_path;
                row.source_parameter = source.parameter;
                row.target_parameter = target.parameter;
                row.source_phase_frames = SamplePhaseFrames(
                    source.clip.NumFrames(), grid_config.source_phase_start,
                    grid_config.source_phase_end,
                    grid_config.source_frame_stride);
                row.target_phase_frames = SamplePhaseFrames(
                    target.clip.NumFrames(), grid_config.target_phase_start,
                    grid_config.target_phase_end,
                    grid_config.target_frame_stride);
                row.root_trajectory_rmse =
                    RootTrajectoryRmse(source.clip, target.clip);
                row.phase_alignment_offset =
                    optimal.source_frame >= 0
                        ? WrapPhaseOffset(
                              optimal.source_phase - optimal.target_phase)
                        : 0.0f;
                row.pose_seam_distance = optimal.distance;
                row.pose_seam_class =
                    PoseSeamClass(optimal.distance, builder_config);
                if (optimal.source_frame >= 0 && optimal.target_frame >= 0) {
                    const pmg::TransitionMetricResult dynamics =
                        pmg::MotionDistance::EvaluateDynamicsTransition(
                            prepared.skeleton, source.clip, target.clip,
                            optimal.source_frame, optimal.target_frame,
                            grid_config, metric_config);
                    row.velocity_seam_cost = dynamics.velocity_cost;
                    row.root_motion_cost = dynamics.root_cost;
                    row.foot_mismatch_ratio =
                        dynamics.foot_comparison_count > 0
                            ? static_cast<float>(dynamics.foot_mismatch_count) /
                                  static_cast<float>(
                                      dynamics.foot_comparison_count)
                            : -1.0f;
                }
                row.safe_for_transition_sampling =
                    row.pose_seam_class == "GOOD";
                row.jog_like_pair = LooksJogLike(row.source_clip_name) ||
                                    LooksJogLike(row.target_clip_name);
                if (row.jog_like_pair && !row.safe_for_transition_sampling) {
                    row.conclusion = "WARN_PHASE_MISMATCH_FOR_JOG";
                } else if (row.pose_seam_class == "BAD") {
                    row.conclusion = "WARN_PAIR_NOT_SAFE";
                } else {
                    row.conclusion = "PASS_PAIR_PHASE_ALIGNMENT";
                }
                data.pair_rows.push_back(std::move(row));
            }
        }
    }

    data.conclusion = "PASS_PHASE_ALIGNMENT";
    for (const RegistrationPhaseAlignmentClipRow& row : data.clip_rows) {
        if (row.conclusion == "FAIL_REGISTRATION_INCONSISTENT") {
            data.conclusion = "FAIL_REGISTRATION_INCONSISTENT";
            return data;
        }
        if (row.conclusion == "FAIL_CLIP_NOT_CYCLIC_ENOUGH") {
            data.conclusion = "FAIL_CLIP_NOT_CYCLIC_ENOUGH";
            return data;
        }
    }
    for (const RegistrationPhaseAlignmentPairRow& row : data.pair_rows) {
        if (row.conclusion == "WARN_PHASE_MISMATCH_FOR_JOG") {
            data.conclusion = "WARN_PHASE_MISMATCH_FOR_JOG";
            return data;
        }
    }
    for (const RegistrationPhaseAlignmentClipRow& row : data.clip_rows) {
        if (row.conclusion == "WARN_WEAK_CYCLE_SEAM") {
            data.conclusion = "WARN_WEAK_CYCLE_SEAM";
            return data;
        }
    }
    return data;
}

void WriteRegistrationPhaseAlignmentCsv(
    const std::string& path,
    const RegistrationPhaseAlignmentAuditData& data) {
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error(
            "--audit-registration-phase-alignment: cannot write CSV");
    }
    csv << std::setprecision(9)
        << "row_type,node,source_clip,target_clip,source_bvh,target_bvh,"
           "source_parameter,target_parameter,frame_count,fps,first_root,"
           "first_yaw,final_relative_dx,final_relative_dz,final_relative_heading,"
           "seam_pose_distance,seam_pose_ratio,root_speed_ratio,yaw_rate_ratio,"
           "has_contact_evidence,contact_state_matches,source_phase_frames,"
           "target_phase_frames,root_trajectory_rmse,phase_alignment_offset,"
           "pose_seam_class,velocity_seam_cost,root_motion_cost,"
           "foot_mismatch_ratio,safe_for_transition_sampling,conclusion\n";
    for (const RegistrationPhaseAlignmentClipRow& row : data.clip_rows) {
        csv << "clip," << row.node_name << ',' << row.clip_name << ",,"
            << row.bvh_path << ",,"
            << ParameterCsv(row.parameter) << ",,"
            << row.frame_count << ',' << row.frames_per_second << ','
            << Vec3Csv(row.normalized.first_root_position) << ','
            << row.normalized.first_heading_yaw << ','
            << row.normalized.final_relative_displacement.x << ','
            << row.normalized.final_relative_displacement.z << ','
            << row.normalized.final_relative_heading << ','
            << row.seam.seam_step << ',' << row.seam.seam_step_ratio << ','
            << row.seam.root_speed_ratio << ',' << row.seam.yaw_rate_ratio
            << ',' << (row.seam.has_contact_evidence ? "true" : "false") << ','
            << (row.seam.contact_state_matches ? "true" : "false") << ','
            << IntegerListCsv(row.source_phase_frames) << ','
            << IntegerListCsv(row.target_phase_frames)
            << ",,,,"
            << ",,,,"
            << row.conclusion << "\n";
    }
    for (const RegistrationPhaseAlignmentPairRow& row : data.pair_rows) {
        csv << "pair," << row.node_name << ',' << row.source_clip_name << ','
            << row.target_clip_name << ',' << row.source_bvh_path << ','
            << row.target_bvh_path << ',' << ParameterCsv(row.source_parameter)
            << ',' << ParameterCsv(row.target_parameter) << ",,,,,,,"
            << row.pose_seam_distance << ",,,,,"
            << IntegerListCsv(row.source_phase_frames) << ','
            << IntegerListCsv(row.target_phase_frames) << ','
            << row.root_trajectory_rmse << ','
            << row.phase_alignment_offset << ',' << row.pose_seam_class << ','
            << row.velocity_seam_cost << ',' << row.root_motion_cost << ','
            << row.foot_mismatch_ratio << ','
            << (row.safe_for_transition_sampling ? "true" : "false") << ','
            << row.conclusion << "\n";
    }
}

void WriteRegistrationPhaseAlignmentMarkdown(
    const std::string& path,
    const RegistrationPhaseAlignmentAuditOptions& options,
    const RegistrationPhaseAlignmentAuditData& data) {
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error(
            "--audit-registration-phase-alignment: cannot write markdown");
    }
    const int weak_clip_count = static_cast<int>(std::count_if(
        data.clip_rows.begin(), data.clip_rows.end(), [](const auto& row) {
            return row.conclusion == "WARN_WEAK_CYCLE_SEAM";
        }));
    const int jog_warning_count = static_cast<int>(std::count_if(
        data.pair_rows.begin(), data.pair_rows.end(), [](const auto& row) {
            return row.conclusion == "WARN_PHASE_MISMATCH_FOR_JOG";
        }));
    const bool canonical_ok = std::all_of(
        data.clip_rows.begin(), data.clip_rows.end(), [](const auto& row) {
            return row.start_is_origin && row.heading_is_canonical;
        });

    md << "# Registration / Phase Alignment Audit\n\n";
    md << "## Purpose\n\n";
    md << "Report canonical starts, cyclic seam quality, and pairwise transition "
          "compatibility for current authored anchors. Report only; no clips, "
          "registration, thresholds, or runtime behavior change.\n\n";
    md << "## Inputs\n\n";
    md << "- Spec: `" << options.spec_path << "`\n";
    md << "- Clip rows: " << data.clip_rows.size() << "\n";
    md << "- Pair rows: " << data.pair_rows.size() << "\n";
    md << "- Canonical origin/heading verified: "
       << (canonical_ok ? "yes" : "no") << "\n";
    md << "- Weak seams: " << weak_clip_count << "\n";
    md << "- Jog pair warnings: " << jog_warning_count << "\n";
    md << "- Final conclusion: `" << data.conclusion << "`\n\n";
    md << "## Clip audit\n\n";
    md << "| Node | Clip | Parameter | Frames | FPS | First root x/z | First yaw | Final dx/dz | Final heading | Seam ratio | Root speed ratio | Yaw rate ratio | Contact match | Source phases | Target phases | Conclusion |\n";
    md << "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|---|\n";
    for (const RegistrationPhaseAlignmentClipRow& row : data.clip_rows) {
        md << "| " << row.node_name << " | " << row.clip_name << " | "
           << ParameterMd(row.parameter) << " | " << row.frame_count << " | "
           << row.frames_per_second << " | "
           << row.normalized.first_root_position.x << ", "
           << row.normalized.first_root_position.z << " | "
           << row.normalized.first_heading_yaw << " | "
           << row.normalized.final_relative_displacement.x << ", "
           << row.normalized.final_relative_displacement.z << " | "
           << row.normalized.final_relative_heading << " | "
           << row.seam.seam_step_ratio << " | "
           << row.seam.root_speed_ratio << " | "
           << row.seam.yaw_rate_ratio << " | "
           << (row.seam.has_contact_evidence
                   ? (row.seam.contact_state_matches ? "yes" : "no")
                   : "n/a")
           << " | " << IntegerListCsv(row.source_phase_frames) << " | "
           << IntegerListCsv(row.target_phase_frames) << " | `"
           << row.conclusion << "` |\n";
    }
    md << "\n## Pair audit\n\n";
    md << "| Node | Source | Target | Source p | Target p | Trajectory RMSE | Phase offset | Pose seam | Velocity cost | Root cost | Foot mismatch | Safe | Conclusion |\n";
    md << "|---|---|---|---|---|---:|---:|---|---:|---:|---:|---|---|\n";
    for (const RegistrationPhaseAlignmentPairRow& row : data.pair_rows) {
        md << "| " << row.node_name << " | " << row.source_clip_name << " | "
           << row.target_clip_name << " | " << ParameterMd(row.source_parameter)
           << " | " << ParameterMd(row.target_parameter) << " | "
           << row.root_trajectory_rmse << " | "
           << row.phase_alignment_offset << " | "
           << row.pose_seam_class << " (" << row.pose_seam_distance << ") | "
           << row.velocity_seam_cost << " | " << row.root_motion_cost << " | ";
        if (row.foot_mismatch_ratio < 0.0f) {
            md << "n/a";
        } else {
            md << row.foot_mismatch_ratio;
        }
        md << " | " << (row.safe_for_transition_sampling ? "yes" : "no")
           << " | `" << row.conclusion << "` |\n";
    }
    md << "\nArtifacts: `" << options.output_csv << "`, `" << options.output_md
       << "`.\n";
}

RegistrationPhaseAlignmentAuditOptions ParseRegistrationPhaseAlignmentAuditOptions(
    int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error(
            "--audit-registration-phase-alignment needs <spec>");
    }
    RegistrationPhaseAlignmentAuditOptions options;
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
                "unknown audit-registration-phase-alignment option '" +
                option + "'");
        }
    }
    if (options.output_csv.empty()) {
        throw std::runtime_error(
            "--audit-registration-phase-alignment requires --output-csv");
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-registration-phase-alignment requires --output-md");
    }
    return options;
}

int RegistrationPhaseAlignmentAuditCommand(
    const RegistrationPhaseAlignmentAuditOptions& options) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(options.spec_path);
    const pmg::PreparedMotionSpaces prepared = pmg::PrepareMotionSpaces(spec);
    const RegistrationPhaseAlignmentAuditData data =
        BuildRegistrationPhaseAlignmentAudit(spec, prepared);
    WriteRegistrationPhaseAlignmentCsv(options.output_csv, data);
    WriteRegistrationPhaseAlignmentMarkdown(options.output_md, options, data);
    std::cout << "registration_phase_alignment_clip_rows="
              << data.clip_rows.size() << "\n";
    std::cout << "registration_phase_alignment_pair_rows="
              << data.pair_rows.size() << "\n";
    std::cout << "registration_phase_alignment_csv=" << options.output_csv
              << "\n";
    std::cout << "registration_phase_alignment_md=" << options.output_md
              << "\n";
    std::cout << "registration_phase_alignment_conclusion="
              << data.conclusion << "\n";
    return 0;
}

struct ReachableRegionAuditOptions {
    std::string pmg_path;
    std::string output_csv;
    std::string output_md;
    std::string output_dir;
    std::string source_node;
    std::string target_node;
    int samples_per_axis = 11;
};

struct TransitionMontageAuditOptions {
    std::string pmg_path;
    std::string output_md;
    std::string output_dir;
    std::string source_node;
    std::string target_node;
    int samples_per_axis = 9;
};

struct ContactTransitionAuditOptions {
    std::string pmg_path;
    std::string output_csv;
    std::string output_md;
    std::string source_node;
    std::string target_node;
    int samples_per_axis = 9;
};

struct TransitionAcceptanceConsistencyAuditOptions {
    std::string pmg_path;
    std::string output_csv;
    std::string output_md;
    std::string source_node;
    std::string target_node;
    int samples_per_axis = 9;
    pmg::TransitionQualityGateConfig quality_gate{
        true, 1.5f, 50.0f, 2.0f, false, 2.0f};
};

struct TransitionProbeOptions {
    std::string pmg_path;
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector target_parameter;
    std::string output_csv;
    std::string output_md;
    std::string source_node;
    std::string target_node;
    int frames_before = 3;
    int frames_after = 3;
    std::string expected_decision;
    pmg::TransitionQualityGateConfig quality_gate{
        true, 1.5f, 50.0f, 2.0f, false, 2.0f};
};

struct ReachableRegionAuditRow {
    pmg::ParameterVector source_parameter;
    pmg::ParameterAabb target_box;
    std::vector<pmg::ParameterVector> accepted_target_parameters;
    int accepted_target_count = 0;
    float support_coverage = 0.0f;
    int good_count = 0;
    int neutral_count = 0;
    int bad_count = 0;
    float worst_accepted_distance = 0.0f;
    float worst_root_jump = 0.0f;
    float worst_heading_jump = 0.0f;
    float worst_velocity_jump = 0.0f;
    bool empty_region = false;
    std::string conclusion;
};

struct ReachableRegionAuditData {
    std::string source_node;
    std::string target_node;
    pmg::PmgBuilderConfig config;
    std::vector<pmg::ParameterVector> target_samples;
    std::vector<ReachableRegionAuditRow> rows;
    std::string conclusion;
};

void AppendUniqueParameter(
    std::vector<pmg::ParameterVector>& parameters,
    const pmg::ParameterVector& candidate) {
    for (const pmg::ParameterVector& existing : parameters) {
        if (existing.size() != candidate.size()) {
            continue;
        }
        bool same = true;
        for (std::size_t axis = 0; axis < existing.size(); ++axis) {
            if (std::abs(existing[axis] - candidate[axis]) > 1.0e-5f) {
                same = false;
                break;
            }
        }
        if (same) {
            return;
        }
    }
    parameters.push_back(candidate);
}

std::string ParameterListCsv(
    const std::vector<pmg::ParameterVector>& parameters) {
    std::ostringstream out;
    out << '"';
    for (std::size_t index = 0; index < parameters.size(); ++index) {
        if (index > 0) {
            out << ';';
        }
        out << '[';
        for (std::size_t axis = 0; axis < parameters[index].size(); ++axis) {
            if (axis > 0) {
                out << ' ';
            }
            out << parameters[index][axis];
        }
        out << ']';
    }
    out << '"';
    return out.str();
}

std::vector<pmg::ParameterVector> BuildAuditParameterSamples(
    const pmg::ParametricMotionSpace& space,
    int samples_per_axis) {
    if (samples_per_axis < 2) {
        throw std::runtime_error(
            "--audit-reachable-region: --samples must be at least 2");
    }

    std::vector<pmg::ParameterVector> samples;
    for (const pmg::ParameterVector& parameter : space.ExampleParameters()) {
        AppendUniqueParameter(samples, parameter);
    }

    if (space.HasExplicitParameterSupport()) {
        const pmg::ParameterSupport& support = *space.ExplicitSupport();
        for (const pmg::ParameterVector& vertex : support.Vertices()) {
            AppendUniqueParameter(samples, vertex);
        }
        if (support.GetType() == pmg::ParameterSupport::Type::kTriangulated2D) {
            std::set<std::pair<int, int>> edges;
            for (const auto& triangle : support.Triangles()) {
                pmg::ParameterVector centroid(2, 0.0f);
                for (int vertex : triangle) {
                    centroid[0] += support.Vertices()[vertex][0] / 3.0f;
                    centroid[1] += support.Vertices()[vertex][1] / 3.0f;
                }
                AppendUniqueParameter(samples, centroid);
                for (int edge = 0; edge < 3; ++edge) {
                    int a = triangle[edge];
                    int b = triangle[(edge + 1) % 3];
                    if (a > b) {
                        std::swap(a, b);
                    }
                    if (!edges.insert({a, b}).second) {
                        continue;
                    }
                    AppendUniqueParameter(
                        samples,
                        {(support.Vertices()[a][0] + support.Vertices()[b][0]) * 0.5f,
                         (support.Vertices()[a][1] + support.Vertices()[b][1]) * 0.5f});
                }
            }
        }
    }

    const pmg::ParameterDomain domain = space.Domain();
    const pmg::ParameterAabb& bounds = domain.Bounds();
    if (domain.Dimension() == 1) {
        for (int index = 0; index < samples_per_axis; ++index) {
            const float t =
                static_cast<float>(index) /
                static_cast<float>(samples_per_axis - 1);
            const pmg::ParameterVector parameter = {
                bounds.min_corner[0] +
                t * (bounds.max_corner[0] - bounds.min_corner[0])};
            if (!space.HasExplicitParameterSupport() ||
                space.ExplicitSupport()->Contains(parameter)) {
                AppendUniqueParameter(samples, parameter);
            }
        }
    } else if (domain.Dimension() == 2) {
        for (int y_index = 0; y_index < samples_per_axis; ++y_index) {
            const float ty =
                static_cast<float>(y_index) /
                static_cast<float>(samples_per_axis - 1);
            const float y = bounds.min_corner[1] +
                            ty * (bounds.max_corner[1] - bounds.min_corner[1]);
            for (int x_index = 0; x_index < samples_per_axis; ++x_index) {
                const float tx =
                    static_cast<float>(x_index) /
                    static_cast<float>(samples_per_axis - 1);
                const float x =
                    bounds.min_corner[0] +
                    tx * (bounds.max_corner[0] - bounds.min_corner[0]);
                const pmg::ParameterVector parameter = {x, y};
                if (!space.HasExplicitParameterSupport() ||
                    space.ExplicitSupport()->Contains(parameter)) {
                    AppendUniqueParameter(samples, parameter);
                }
            }
        }
    }

    return samples;
}

const pmg::EdgeBuildMetadata* FindEdgeBuildMetadata(
    const pmg::BuiltPmgArtifact& artifact,
    const std::string& source_node,
    const std::string& target_node) {
    for (const pmg::EdgeBuildMetadata& edge : artifact.metadata.edge_builds) {
        if (edge.source_node == source_node && edge.target_node == target_node) {
            return &edge;
        }
    }
    return nullptr;
}

pmg::OptimalTransition EvaluateConfiguredTransition(
    const pmg::Skeleton& skeleton,
    const pmg::MotionClip& source_clip,
    const pmg::MotionClip& target_clip,
    const pmg::PmgBuilderConfig& config) {
    if (config.transition_metric_type ==
        pmg::TransitionMetricType::kDynamicsWindow) {
        return pmg::MotionDistance::FindOptimalDynamicsTransitionForConvention(
            skeleton, source_clip, target_clip, config.distance_grid,
            config.transition_metric, config.transition_convention);
    }
    return pmg::MotionDistance::FindOptimalTransitionForConvention(
        skeleton, source_clip, target_clip, config.distance_grid,
        config.transition_convention);
}

float RootJumpAtTransition(
    const pmg::Pose& source_pose,
    const pmg::Pose& aligned_target_pose) {
    return HorizontalDistance(
        aligned_target_pose.root_position - source_pose.root_position);
}

float HeadingJumpAtTransition(
    const pmg::Pose& source_pose,
    const pmg::Pose& aligned_target_pose) {
    return std::abs(
        pmg::WrapAngleRadians(
            pmg::RootHeadingYaw(aligned_target_pose) -
            pmg::RootHeadingYaw(source_pose)));
}

float VelocityJumpAtTransition(
    const pmg::MotionClip& source_clip,
    const pmg::MotionClip& target_clip,
    const pmg::RigidTransform2D& alignment,
    int source_frame,
    int target_frame) {
    if (source_frame <= 0 || target_frame <= 0) {
        return 0.0f;
    }

    const pmg::Vec3 source_step =
        source_clip.frames[static_cast<std::size_t>(source_frame)].root_position -
        source_clip.frames[static_cast<std::size_t>(source_frame - 1)].root_position;
    const pmg::Vec3 target_step =
        alignment.ApplyPoint(
            target_clip.frames[static_cast<std::size_t>(target_frame)].root_position) -
        alignment.ApplyPoint(
            target_clip.frames[static_cast<std::size_t>(target_frame - 1)].root_position);
    const pmg::Vec3 delta =
        (target_step - source_step) * source_clip.frames_per_second;
    return HorizontalLength(delta);
}

struct TransitionContactJoints {
    int left = -1;
    int right = -1;
    int min_contact_frames = 1;
};

TransitionContactJoints ResolveTransitionContactJoints(
    const pmg::BuiltPmgArtifact& artifact,
    const std::string& node_name,
    const pmg::PmgBuilderConfig& config) {
    TransitionContactJoints result;
    const std::vector<int>& metric_joints =
        config.transition_metric.contact_joint_indices;
    if (!metric_joints.empty()) result.left = metric_joints[0];
    if (metric_joints.size() > 1) result.right = metric_joints[1];

    const auto registration = std::find_if(
        artifact.metadata.node_registrations.begin(),
        artifact.metadata.node_registrations.end(),
        [&](const auto& value) { return value.node_name == node_name; });
    if (registration != artifact.metadata.node_registrations.end()) {
        result.min_contact_frames = registration->min_contact_frames;
        for (const std::string& name : registration->contact_joints) {
            const int joint = ResolveJointIndex(artifact.skeleton, name);
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(),
                           [](unsigned char c) {
                               return static_cast<char>(std::tolower(c));
                           });
            if (lower.find("left") != std::string::npos) result.left = joint;
            if (lower.find("right") != std::string::npos) result.right = joint;
        }
    }
    if (result.left < 0) {
        result.left = ResolveJointIndex(artifact.skeleton, "LeftAnkle");
    }
    if (result.right < 0) {
        result.right = ResolveJointIndex(artifact.skeleton, "RightAnkle");
    }
    return result;
}

pmg::TransitionQualityRecord MeasureExactTransitionQuality(
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const pmg::ParameterVector& source_parameter,
    const pmg::ParameterVector& effective_target_parameter,
    const pmg::OptimalTransition& transition,
    int frames_before,
    int frames_after) {
    if (frames_before < 1 || frames_after < 1) {
        throw std::runtime_error(
            "transition quality probe requires positive before/after frames");
    }
    const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
        source_parameter, artifact.metadata.frames_per_second);
    const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
        effective_target_parameter, artifact.metadata.frames_per_second);

    std::vector<pmg::Pose> poses;
    poses.reserve(static_cast<std::size_t>(frames_before + frames_after + 1));
    const bool continuous_self = &source_node == &target_node &&
        source_parameter == effective_target_parameter;
    if (continuous_self) {
        for (int offset = -frames_before; offset <= frames_after; ++offset) {
            const int frame = std::clamp(
                transition.source_frame + offset, 0, source_clip.NumFrames() - 1);
            poses.push_back(source_clip.frames[static_cast<std::size_t>(frame)]);
        }
    } else {
        const int blend_frames = config.distance_grid.window_size;
        const int blend_first_offset = -blend_frames / 2;
        const int blend_last_offset =
            blend_first_offset + blend_frames - 1;
        for (int offset = -frames_before; offset <= frames_after; ++offset) {
            const int source_frame = std::clamp(
                transition.source_frame + offset, 0, source_clip.NumFrames() - 1);
            const int target_frame = std::clamp(
                transition.target_frame + offset, 0, target_clip.NumFrames() - 1);
            const pmg::Pose aligned_target = transition.alignment.Apply(
                target_clip.frames[static_cast<std::size_t>(target_frame)]);
            if (offset < blend_first_offset) {
                poses.push_back(
                    source_clip.frames[static_cast<std::size_t>(source_frame)]);
            } else if (offset > blend_last_offset) {
                poses.push_back(aligned_target);
            } else {
                const float linear_alpha = blend_frames == 1
                    ? 1.0f
                    : static_cast<float>(offset - blend_first_offset) /
                          static_cast<float>(blend_frames - 1);
                const float alpha = linear_alpha * linear_alpha *
                                    (3.0f - 2.0f * linear_alpha);
                poses.push_back(pmg::BlendPose(
                    source_clip.frames[static_cast<std::size_t>(source_frame)],
                    aligned_target, alpha));
            }
        }
    }

    const TransitionContactJoints joints = ResolveTransitionContactJoints(
        artifact, source_node.name, config);
    std::vector<int> contact_joints;
    if (joints.left >= 0) contact_joints.push_back(joints.left);
    if (joints.right >= 0 && joints.right != joints.left) {
        contact_joints.push_back(joints.right);
    }

    pmg::TransitionQualityContext context;
    context.frames_per_second = artifact.metadata.frames_per_second;
    context.transition_distance = transition.distance;
    context.left_foot_joint = joints.left;
    context.right_foot_joint = joints.right;
    if (!contact_joints.empty()) {
        pmg::MotionClip quality_clip;
        quality_clip.frames_per_second = artifact.metadata.frames_per_second;
        quality_clip.frames = poses;
        context.contact_settings = pmg::EstimateContactSettings(
            artifact.skeleton, quality_clip, contact_joints);
        context.contact_settings->min_contact_frames = 1;
    }

    pmg::TransitionQualityConfig quality_config;
    quality_config.frames_before = frames_before;
    quality_config.frames_after = frames_after;
    return pmg::MeasureTransitionQuality(
        artifact.skeleton, poses, frames_before, context, quality_config);
}

int ResolveNodeIndex(
    const pmg::ParametricMotionGraph& graph,
    const std::string& node_name) {
    for (int node_index = 0; node_index < graph.NumNodes(); ++node_index) {
        if (graph.Node(node_index).name == node_name) {
            return node_index;
        }
    }
    return -1;
}

int ResolveEdgeIndex(
    const pmg::BuiltPmgArtifact& artifact,
    const ReachableRegionAuditOptions& options,
    std::string& source_node,
    std::string& target_node) {
    if (!options.source_node.empty() || !options.target_node.empty()) {
        if (options.source_node.empty() || options.target_node.empty()) {
            throw std::runtime_error(
                "--audit-reachable-region requires both --source-node and "
                "--target-node");
        }
        const int source_index =
            ResolveNodeIndex(artifact.graph, options.source_node);
        const int target_index =
            ResolveNodeIndex(artifact.graph, options.target_node);
        if (source_index < 0 || target_index < 0) {
            throw std::runtime_error(
                "--audit-reachable-region: unknown source or target node");
        }
        for (int edge_index = 0; edge_index < artifact.graph.NumEdges();
             ++edge_index) {
            const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
            if (edge.source_node == source_index &&
                edge.target_node == target_index) {
                source_node = options.source_node;
                target_node = options.target_node;
                return edge_index;
            }
        }
        throw std::runtime_error(
            "--audit-reachable-region: requested edge not found");
    }

    if (artifact.graph.NumEdges() != 1) {
        throw std::runtime_error(
            "--audit-reachable-region requires --source-node and --target-node "
            "when artifact has multiple edges");
    }

    const pmg::PmgEdge& edge = artifact.graph.Edge(0);
    source_node = artifact.graph.Node(edge.source_node).name;
    target_node = artifact.graph.Node(edge.target_node).name;
    return 0;
}

ReachableRegionAuditData BuildReachableRegionAudit(
    const pmg::BuiltPmgArtifact& artifact,
    const ReachableRegionAuditOptions& options) {
    ReachableRegionAuditData data;
    int edge_index = -1;
    edge_index = ResolveEdgeIndex(
        artifact, options, data.source_node, data.target_node);
    const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
    const pmg::PmgNode& source_node = artifact.graph.Node(edge.source_node);
    const pmg::PmgNode& target_node = artifact.graph.Node(edge.target_node);
    const pmg::EdgeBuildMetadata* edge_metadata =
        FindEdgeBuildMetadata(artifact, data.source_node, data.target_node);
    if (edge_metadata == nullptr) {
        throw std::runtime_error(
            "--audit-reachable-region: missing edge build metadata");
    }
    data.config = edge_metadata->config;
    data.target_samples = BuildAuditParameterSamples(
        target_node.motion_space, options.samples_per_axis);
    for (const pmg::TransitionSample& sample : edge.samples) {
        for (const pmg::TargetTransitionPhaseSample& phase_sample :
             sample.target_phase_samples) {
            AppendUniqueParameter(data.target_samples, phase_sample.target_parameter);
        }
    }

    std::vector<int> accepted_density(
        data.target_samples.size(), 0);
    std::vector<float> worst_target_distance(
        data.target_samples.size(), 0.0f);

    constexpr float kCoverageShrinkWarnRatio = 0.5f;
    float max_support_coverage = 0.0f;
    bool any_empty = false;
    bool any_bad_accepted = false;

    for (std::size_t source_index = 0; source_index < edge.samples.size();
         ++source_index) {
        const pmg::TransitionSample& source_sample = edge.samples[source_index];
        ReachableRegionAuditRow row;
        row.source_parameter = source_sample.source_parameter;

        const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
            source_sample.source_parameter,
            artifact.metadata.frames_per_second);

        for (std::size_t target_index = 0; target_index < data.target_samples.size();
             ++target_index) {
            const pmg::ParameterVector& target_parameter =
                data.target_samples[target_index];
            const std::optional<pmg::InterpolatedTransition> interpolated =
                edge.LookupInterpolated(
                    source_sample.source_parameter, target_parameter);
            if (!interpolated.has_value()) {
                continue;
            }

            row.target_box = interpolated->target_parameter_box;
            const pmg::MotionClip target_clip =
                target_node.motion_space.GenerateClip(
                    target_parameter, artifact.metadata.frames_per_second);
            const pmg::OptimalTransition transition =
                EvaluateConfiguredTransition(
                    artifact.skeleton, source_clip, target_clip, data.config);
            if (transition.distance <= data.config.good_transition_threshold) {
                ++row.good_count;
            } else if (transition.distance >= data.config.bad_transition_threshold) {
                ++row.bad_count;
            } else {
                ++row.neutral_count;
            }

            if (!interpolated->target_parameter_box.Contains(target_parameter)) {
                continue;
            }

            ++row.accepted_target_count;
            AppendUniqueParameter(row.accepted_target_parameters, target_parameter);
            ++accepted_density[target_index];
            worst_target_distance[target_index] = std::max(
                worst_target_distance[target_index], transition.distance);
            row.worst_accepted_distance = std::max(
                row.worst_accepted_distance, transition.distance);
            const pmg::Pose source_pose =
                source_clip.frames[static_cast<std::size_t>(transition.source_frame)];
            const pmg::Pose aligned_target_pose = transition.alignment.Apply(
                target_clip.frames[static_cast<std::size_t>(transition.target_frame)]);
            row.worst_root_jump = std::max(
                row.worst_root_jump,
                RootJumpAtTransition(source_pose, aligned_target_pose));
            row.worst_heading_jump = std::max(
                row.worst_heading_jump,
                HeadingJumpAtTransition(source_pose, aligned_target_pose));
            row.worst_velocity_jump = std::max(
                row.worst_velocity_jump,
                VelocityJumpAtTransition(
                    source_clip, target_clip, transition.alignment,
                    transition.source_frame, transition.target_frame));
        }

        if (!data.target_samples.empty()) {
            row.support_coverage =
                static_cast<float>(row.accepted_target_count) /
                static_cast<float>(data.target_samples.size());
        }
        row.empty_region = row.accepted_target_count == 0;
        max_support_coverage = std::max(max_support_coverage, row.support_coverage);
        any_empty = any_empty || row.empty_region;
        any_bad_accepted =
            any_bad_accepted ||
            row.worst_accepted_distance >= data.config.bad_transition_threshold;
        data.rows.push_back(std::move(row));
    }

    data.conclusion = "PASS_REACHABLE_REGION_STABLE";
    for (ReachableRegionAuditRow& row : data.rows) {
        if (row.empty_region) {
            row.conclusion = "FAIL_EMPTY_REGION_FOR_VALID_SOURCE";
            continue;
        }
        if (row.worst_accepted_distance >= data.config.bad_transition_threshold) {
            row.conclusion = "FAIL_THRESHOLD_TOO_LOOSE";
            continue;
        }
        if (max_support_coverage > 0.0f &&
            row.support_coverage < max_support_coverage * kCoverageShrinkWarnRatio) {
            row.conclusion = "WARN_SOURCE_DEPENDENT_REGION_SHRINKAGE";
            continue;
        }
        row.conclusion = "PASS_REACHABLE_REGION_STABLE";
    }

    if (any_empty) {
        data.conclusion = "FAIL_EMPTY_REGION_FOR_VALID_SOURCE";
    } else if (any_bad_accepted) {
        data.conclusion = "FAIL_THRESHOLD_TOO_LOOSE";
    } else {
        for (const ReachableRegionAuditRow& row : data.rows) {
            if (row.conclusion == "WARN_SOURCE_DEPENDENT_REGION_SHRINKAGE") {
                data.conclusion = "WARN_SOURCE_DEPENDENT_REGION_SHRINKAGE";
                break;
            }
        }
    }

    std::filesystem::create_directories(options.output_dir);
    {
        std::ofstream density(options.output_dir + "/accepted_density.csv");
        if (!density) {
            throw std::runtime_error(
                "--audit-reachable-region: cannot write accepted density map");
        }
        density << "target_parameter,accepted_source_count,worst_accepted_distance\n";
        for (std::size_t target_index = 0; target_index < data.target_samples.size();
             ++target_index) {
            density << ParameterCsv(data.target_samples[target_index]) << ','
                    << accepted_density[target_index] << ','
                    << worst_target_distance[target_index] << "\n";
        }
    }
    for (std::size_t source_index = 0; source_index < data.rows.size();
         ++source_index) {
        std::ostringstream path;
        path << options.output_dir << "/source_"
             << std::setw(2) << std::setfill('0') << source_index << ".csv";
        std::ofstream csv(path.str());
        if (!csv) {
            throw std::runtime_error(
                "--audit-reachable-region: cannot write per-source map");
        }
        csv << "source_parameter,target_parameter,accepted,target_box_min,target_box_max,"
               "transition_distance,transition_class,root_jump,heading_jump,"
               "velocity_jump\n";
        const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
            data.rows[source_index].source_parameter,
            artifact.metadata.frames_per_second);
        for (const pmg::ParameterVector& target_parameter : data.target_samples) {
            const std::optional<pmg::InterpolatedTransition> interpolated =
                edge.LookupInterpolated(
                    data.rows[source_index].source_parameter, target_parameter);
            if (!interpolated.has_value()) {
                continue;
            }
            const pmg::MotionClip target_clip =
                target_node.motion_space.GenerateClip(
                    target_parameter, artifact.metadata.frames_per_second);
            const pmg::OptimalTransition transition =
                EvaluateConfiguredTransition(
                    artifact.skeleton, source_clip, target_clip, data.config);
            const bool accepted =
                interpolated->target_parameter_box.Contains(target_parameter);
            const pmg::Pose source_pose =
                source_clip.frames[static_cast<std::size_t>(transition.source_frame)];
            const pmg::Pose aligned_target_pose = transition.alignment.Apply(
                target_clip.frames[static_cast<std::size_t>(transition.target_frame)]);
            csv << ParameterCsv(data.rows[source_index].source_parameter) << ','
                << ParameterCsv(target_parameter) << ','
                << (accepted ? "true" : "false") << ','
                << ParameterCsv(interpolated->target_parameter_box.min_corner) << ','
                << ParameterCsv(interpolated->target_parameter_box.max_corner) << ','
                << transition.distance << ','
                << PoseSeamClass(transition.distance, data.config) << ','
                << RootJumpAtTransition(source_pose, aligned_target_pose) << ','
                << HeadingJumpAtTransition(source_pose, aligned_target_pose) << ','
                << VelocityJumpAtTransition(
                       source_clip, target_clip, transition.alignment,
                       transition.source_frame, transition.target_frame)
                << "\n";
        }
    }
    return data;
}

void WriteReachableRegionAuditCsv(
    const std::string& path,
    const ReachableRegionAuditData& data) {
    std::ofstream csv(path);
    if (!csv) {
        throw std::runtime_error(
            "--audit-reachable-region: cannot write CSV");
    }
    csv << std::setprecision(9)
        << "source_node,target_node,source_parameter,target_box_min,target_box_max,"
           "accepted_target_parameters,accepted_target_count,support_coverage,"
           "good_count,neutral_count,bad_count,worst_accepted_distance,"
           "worst_root_jump,worst_heading_jump,worst_velocity_jump,empty_region,"
           "conclusion\n";
    for (const ReachableRegionAuditRow& row : data.rows) {
        csv << data.source_node << ',' << data.target_node << ','
            << ParameterCsv(row.source_parameter) << ','
            << ParameterCsv(row.target_box.min_corner) << ','
            << ParameterCsv(row.target_box.max_corner) << ','
            << ParameterListCsv(row.accepted_target_parameters) << ','
            << row.accepted_target_count << ',' << row.support_coverage << ','
            << row.good_count << ',' << row.neutral_count << ','
            << row.bad_count << ',' << row.worst_accepted_distance << ','
            << row.worst_root_jump << ',' << row.worst_heading_jump << ','
            << row.worst_velocity_jump << ','
            << (row.empty_region ? "true" : "false") << ','
            << row.conclusion << "\n";
    }
}

void WriteReachableRegionAuditMarkdown(
    const std::string& path,
    const ReachableRegionAuditOptions& options,
    const ReachableRegionAuditData& data) {
    std::ofstream md(path);
    if (!md) {
        throw std::runtime_error(
            "--audit-reachable-region: cannot write markdown");
    }
    const int empty_region_count = static_cast<int>(std::count_if(
        data.rows.begin(), data.rows.end(), [](const auto& row) {
            return row.empty_region;
        }));
    md << "# Reachable Target Region Audit\n\n";
    md << "## Purpose\n\n";
    md << "Report source-parameter-dependent target reachability for one PMG "
          "edge under current offline metric and stored thresholds. Report only; "
          "no graph, threshold, or runtime behavior change.\n\n";
    md << "## Inputs\n\n";
    md << "- Artifact: `" << options.pmg_path << "`\n";
    md << "- Edge: `" << data.source_node << " -> " << data.target_node << "`\n";
    md << "- Metric: `"
       << pmg::TransitionMetricTypeName(data.config.transition_metric_type)
       << "`\n";
    md << "- TGOOD/TBAD: `" << data.config.good_transition_threshold << " / "
       << data.config.bad_transition_threshold << "`\n";
    md << "- Source samples: " << data.rows.size() << "\n";
    md << "- Target samples: " << data.target_samples.size() << "\n";
    md << "- Empty accepted regions: " << empty_region_count << "\n";
    md << "- Final conclusion: `" << data.conclusion << "`\n\n";
    md << "## Contract\n\n";
    md << "- Parameters use node authored units.\n";
    md << "- Root jump uses floor-plane distance in artifact units.\n";
    md << "- Heading jump uses radians after point-cloud alignment.\n";
    md << "- Velocity jump uses floor-plane root-velocity delta magnitude in artifact units/s.\n";
    md << "- Coverage = accepted target sample count / audited target sample count.\n\n";
    md << "## Source summary\n\n";
    md << "| Source p | Box min | Box max | Accepted | Coverage | GOOD | NEUTRAL | BAD | Worst D | Worst root jump | Worst heading jump | Worst velocity jump | Conclusion |\n";
    md << "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n";
    for (const ReachableRegionAuditRow& row : data.rows) {
        md << "| " << ParameterMd(row.source_parameter) << " | "
           << ParameterMd(row.target_box.min_corner) << " | "
           << ParameterMd(row.target_box.max_corner) << " | "
           << row.accepted_target_count << " | " << row.support_coverage << " | "
           << row.good_count << " | " << row.neutral_count << " | "
           << row.bad_count << " | " << row.worst_accepted_distance << " | "
           << row.worst_root_jump << " | " << row.worst_heading_jump << " | "
           << row.worst_velocity_jump << " | `" << row.conclusion << "` |\n";
    }
    md << "\n## Artifacts\n\n";
    md << "- Summary CSV: `" << options.output_csv << "`\n";
    md << "- Markdown report: `" << options.output_md << "`\n";
    md << "- Maps directory: `" << options.output_dir << "`\n";
}

ReachableRegionAuditOptions ParseReachableRegionAuditOptions(
    int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--audit-reachable-region needs <graph.pmg>");
    }
    ReachableRegionAuditOptions options;
    options.pmg_path = argv[2];
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
        } else if (option == "--output-dir") {
            options.output_dir = require_value("--output-dir");
        } else if (option == "--source-node") {
            options.source_node = require_value("--source-node");
        } else if (option == "--target-node") {
            options.target_node = require_value("--target-node");
        } else if (option == "--samples") {
            options.samples_per_axis = std::stoi(require_value("--samples"));
        } else {
            throw std::runtime_error(
                "unknown audit-reachable-region option '" + option + "'");
        }
    }
    if (options.output_csv.empty()) {
        throw std::runtime_error(
            "--audit-reachable-region requires --output-csv");
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-reachable-region requires --output-md");
    }
    if (options.output_dir.empty()) {
        throw std::runtime_error(
            "--audit-reachable-region requires --output-dir");
    }
    return options;
}

int ReachableRegionAuditCommand(const ReachableRegionAuditOptions& options) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(options.pmg_path);
    const ReachableRegionAuditData data =
        BuildReachableRegionAudit(artifact, options);
    WriteReachableRegionAuditCsv(options.output_csv, data);
    WriteReachableRegionAuditMarkdown(options.output_md, options, data);
    std::cout << "reachable_region_rows=" << data.rows.size() << "\n";
    std::cout << "reachable_region_csv=" << options.output_csv << "\n";
    std::cout << "reachable_region_md=" << options.output_md << "\n";
    std::cout << "reachable_region_maps=" << options.output_dir << "\n";
    std::cout << "reachable_region_conclusion=" << data.conclusion << "\n";
    return 0;
}

struct TransitionMontageRow {
    std::string category;
    int category_rank = 0;
    std::string montage_id;
    std::string replay_id;
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector requested_target_parameter;
    pmg::ParameterVector effective_target_parameter;
    float source_phase = 0.0f;
    float target_phase = 0.0f;
    int source_frame = -1;
    int target_frame = -1;
    float transition_distance = 0.0f;
    std::string transition_class;
    float root_jump = 0.0f;
    float heading_jump = 0.0f;
    float velocity_jump = 0.0f;
    int blend_frames = 0;
    std::string notes;
};

struct TransitionMontageAuditData {
    std::string source_node;
    std::string target_node;
    pmg::PmgBuilderConfig config;
    std::string manifest_csv_path;
    std::vector<TransitionMontageRow> rows;
    int unique_transition_count = 0;
    int accepted_count = 0;
    int rejected_count = 0;
    int rejected_jog_walk_count = 0;
    std::string conclusion;
};

struct TransitionAcceptanceConsistencyRow {
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector requested_target_parameter;
    pmg::ParameterVector effective_target_parameter;
    pmg::ParameterAabb interpolated_target_box;
    bool accepted_by_box = false;
    float transition_distance = 0.0f;
    std::string metric_class;
    bool acceptance_violation = false;
    float distance_over_tbad = 0.0f;
    bool source_coverage_available = false;
    float source_coverage = 0.0f;
    float source_phase = 0.0f;
    int source_frame = 0;
    float target_phase = 0.0f;
    int target_frame = 0;
    float root_jump = 0.0f;
    float heading_jump = 0.0f;
    float velocity_jump = 0.0f;
    std::optional<pmg::TransitionQualityRecord> quality;
    bool quality_gate_accepts = false;
    std::string quality_reject_reason = "not_evaluated";
    std::string nearest_source_anchor_label;
    std::string nearest_target_anchor_label;
    std::string nearest_good_evidence;
    std::string nearest_bad_evidence;
    std::string overreach_origin;
    bool jog_walk_pair = false;
    std::string notes;
};

struct TransitionAcceptanceConsistencyAuditData {
    std::string source_node;
    std::string target_node;
    pmg::PmgBuilderConfig config;
    std::vector<TransitionAcceptanceConsistencyRow> rows;
    int accepted_bad_count = 0;
    int near_threshold_accepted_count = 0;
    int accepted_bad_after_quality_gate = 0;
    int accepted_near_threshold_after_quality_gate = 0;
    int box_accepted_quality_rejected = 0;
    std::map<std::string, int> quality_reject_reasons;
    int jog_walk_row_count = 0;
    int shrinkage_row_count = 0;
    bool known_bad_case_found = false;
    std::string consistency_conclusion;
    std::string root_cause_conclusion;
};

constexpr float kTransitionMontageAnchorTolerance = 1.0e-5f;
constexpr float kTransitionMontageProjectedNoteTolerance = 1.0e-4f;
constexpr int kWorstAcceptedSamples = 8;
constexpr int kNearThresholdSamples = 8;
constexpr int kRejectedJogWalkSamples = 8;
constexpr int kAnchorToAnchorSamples = 12;
constexpr int kOutsideRequestSamples = 8;
constexpr int kShrinkageSamples = 8;

bool SameParameterWithin(
    const pmg::ParameterVector& left,
    const pmg::ParameterVector& right,
    float tolerance) {
    if (left.size() != right.size()) {
        return false;
    }
    for (std::size_t axis = 0; axis < left.size(); ++axis) {
        if (std::abs(left[axis] - right[axis]) > tolerance) {
            return false;
        }
    }
    return true;
}

std::string NearestAnchorLabel(
    const pmg::ParameterSupport& support,
    const pmg::ParameterVector& parameter) {
    int best_index = -1;
    float best_distance = std::numeric_limits<float>::infinity();
    for (std::size_t index = 0; index < support.Vertices().size(); ++index) {
        const float distance =
            pmg::SquaredDistance(parameter, support.Vertices()[index]);
        if (distance < best_distance) {
            best_distance = distance;
            best_index = static_cast<int>(index);
        }
    }
    return best_index >= 0 ? "anchor_" + std::to_string(best_index)
                           : "anchor_unknown";
}

bool IsAnchorParameter(
    const pmg::ParameterSupport& support,
    const pmg::ParameterVector& parameter) {
    for (const pmg::ParameterVector& vertex : support.Vertices()) {
        if (SameParameterWithin(
                parameter, vertex, kTransitionMontageAnchorTolerance)) {
            return true;
        }
    }
    return false;
}

float ParameterAxisOrZero(const pmg::ParameterVector& parameter, int axis) {
    return axis >= 0 && axis < static_cast<int>(parameter.size())
               ? parameter[static_cast<std::size_t>(axis)]
               : 0.0f;
}

bool IsJogWalkPair(
    const pmg::ParameterVector& source_parameter,
    const pmg::ParameterVector& target_parameter) {
    const float source_speed_axis = ParameterAxisOrZero(source_parameter, 1);
    const float target_speed_axis = ParameterAxisOrZero(target_parameter, 1);
    return std::max(source_speed_axis, target_speed_axis) >= 0.75f &&
           std::min(source_speed_axis, target_speed_axis) <= 0.25f;
}

struct TransitionAuditCandidate {
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector requested_target_parameter;
    pmg::ParameterVector effective_target_parameter;
    pmg::ParameterAabb interpolated_target_box;
    pmg::OptimalTransition transition;
    bool accepted = false;
    bool source_is_anchor = false;
    bool requested_target_is_anchor = false;
    bool requested_target_projected = false;
    bool effective_target_projected = false;
    bool jog_walk_pair = false;
    bool source_coverage_available = false;
    float source_coverage = 0.0f;
    std::string transition_class;
    std::string notes;
};

TransitionAuditCandidate BuildTransitionAuditCandidate(
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgEdge& edge,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const ReachableRegionAuditData& reachable_data,
    const pmg::ParameterVector& source_parameter,
    const pmg::ParameterVector& requested_target_parameter,
    const pmg::ParameterSupport* source_support,
    const pmg::ParameterSupport* target_support) {
    const std::optional<pmg::InterpolatedTransition> interpolated =
        edge.LookupInterpolated(source_parameter, requested_target_parameter);
    if (!interpolated.has_value()) {
        throw std::runtime_error(
            "transition montage audit: edge lookup returned no transition");
    }

    TransitionAuditCandidate candidate;
    candidate.source_parameter = source_parameter;
    candidate.requested_target_parameter = requested_target_parameter;
    candidate.effective_target_parameter =
        interpolated->target_parameter_box.Clamp(requested_target_parameter);
    candidate.interpolated_target_box = interpolated->target_parameter_box;
    if (target_support != nullptr) {
        candidate.effective_target_parameter =
            target_support->ProjectInside(
                requested_target_parameter,
                interpolated->target_parameter_box);
    }
    candidate.accepted =
        interpolated->target_parameter_box.Contains(requested_target_parameter);
    candidate.source_is_anchor =
        source_support != nullptr &&
        IsAnchorParameter(*source_support, source_parameter);
    candidate.requested_target_is_anchor =
        target_support != nullptr &&
        IsAnchorParameter(*target_support, requested_target_parameter);
    candidate.requested_target_projected =
        target_support != nullptr &&
        !target_support->Contains(requested_target_parameter);
    candidate.effective_target_projected =
        !SameParameterWithin(
            candidate.requested_target_parameter,
            candidate.effective_target_parameter,
            kTransitionMontageProjectedNoteTolerance);
    candidate.jog_walk_pair =
        IsJogWalkPair(source_parameter, requested_target_parameter);

    const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
        source_parameter, artifact.metadata.frames_per_second);
    const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
        requested_target_parameter, artifact.metadata.frames_per_second);
    candidate.transition = EvaluateConfiguredTransition(
        artifact.skeleton, source_clip, target_clip, config);
    candidate.transition_class =
        candidate.accepted
            ? PoseSeamClass(candidate.transition.distance, config)
            : "REJECTED";

    for (const ReachableRegionAuditRow& row : reachable_data.rows) {
        if (SameParameterWithin(
                row.source_parameter, source_parameter,
                kTransitionMontageAnchorTolerance)) {
            candidate.source_coverage_available = true;
            candidate.source_coverage = row.support_coverage;
            break;
        }
    }

    std::ostringstream notes;
    notes << (candidate.accepted ? "accepted" : "rejected");
    if (candidate.source_is_anchor && source_support != nullptr) {
        notes << "; source " << NearestAnchorLabel(*source_support, source_parameter);
    }
    if (candidate.requested_target_is_anchor && target_support != nullptr) {
        notes << "; target "
              << NearestAnchorLabel(*target_support, requested_target_parameter);
    }
    if (candidate.jog_walk_pair) {
        notes << "; jog/walk pair";
    }
    if (candidate.requested_target_projected) {
        notes << "; requested target outside support";
    }
    if (candidate.effective_target_projected) {
        notes << "; runtime target projects to "
              << ParameterMd(candidate.effective_target_parameter);
    }
    if (candidate.source_coverage_available) {
        notes << "; source coverage " << candidate.source_coverage;
    }
    candidate.notes = notes.str();
    return candidate;
}

std::vector<TransitionAuditCandidate> BuildTransitionAuditCandidates(
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgEdge& edge,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const ReachableRegionAuditData& reachable_data,
    const pmg::ParameterSupport* source_support,
    const pmg::ParameterSupport* target_support,
    int samples_per_axis) {
    std::vector<pmg::ParameterVector> source_samples =
        BuildAuditParameterSamples(source_node.motion_space, samples_per_axis);
    std::vector<pmg::ParameterVector> target_samples =
        BuildAuditParameterSamples(target_node.motion_space, samples_per_axis);
    if (target_support != nullptr && target_support->Dimension() == 2) {
        AppendUniqueParameter(target_samples, {0.8f, 0.8f});
    }

    std::vector<TransitionAuditCandidate> candidates;
    for (const pmg::ParameterVector& source_parameter : source_samples) {
        for (const pmg::ParameterVector& target_parameter : target_samples) {
            candidates.push_back(BuildTransitionAuditCandidate(
                artifact, edge, source_node, target_node, config,
                reachable_data, source_parameter, target_parameter,
                source_support, target_support));
        }
    }
    return candidates;
}

TransitionMontageRow ToTransitionMontageRow(
    const TransitionAuditCandidate& candidate,
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const std::string& category,
    int category_rank,
    int manifest_index) {
    const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
        candidate.source_parameter, artifact.metadata.frames_per_second);
    const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
        candidate.requested_target_parameter, artifact.metadata.frames_per_second);
    const pmg::Pose source_pose = source_clip.frames[static_cast<std::size_t>(
        candidate.transition.source_frame)];
    const pmg::Pose aligned_target_pose = candidate.transition.alignment.Apply(
        target_clip.frames[static_cast<std::size_t>(
            candidate.transition.target_frame)]);

    TransitionMontageRow row;
    row.category = category;
    row.category_rank = category_rank;
    {
        std::ostringstream id;
        id << 'M' << std::setw(3) << std::setfill('0') << manifest_index;
        row.montage_id = id.str();
    }
    row.replay_id = row.montage_id;
    row.source_parameter = candidate.source_parameter;
    row.requested_target_parameter = candidate.requested_target_parameter;
    row.effective_target_parameter = candidate.effective_target_parameter;
    row.source_phase = candidate.transition.source_phase;
    row.target_phase = candidate.transition.target_phase;
    row.source_frame = candidate.transition.source_frame;
    row.target_frame = candidate.transition.target_frame;
    row.transition_distance = candidate.transition.distance;
    row.transition_class = candidate.transition_class;
    row.root_jump = RootJumpAtTransition(source_pose, aligned_target_pose);
    row.heading_jump =
        HeadingJumpAtTransition(source_pose, aligned_target_pose);
    row.velocity_jump = VelocityJumpAtTransition(
        source_clip, target_clip, candidate.transition.alignment,
        candidate.transition.source_frame, candidate.transition.target_frame);
    row.blend_frames = config.distance_grid.window_size;
    row.notes = candidate.notes;
    return row;
}

template <typename Predicate, typename RankKey>
void AppendRankedTransitionRows(
    const std::vector<TransitionAuditCandidate>& candidates,
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const std::string& category,
    int max_rows,
    Predicate predicate,
    RankKey rank_key,
    int& next_manifest_index,
    std::vector<TransitionMontageRow>& out_rows) {
    std::vector<const TransitionAuditCandidate*> ranked;
    for (const TransitionAuditCandidate& candidate : candidates) {
        if (predicate(candidate)) {
            ranked.push_back(&candidate);
        }
    }
    std::sort(
        ranked.begin(), ranked.end(),
        [&](const TransitionAuditCandidate* left,
            const TransitionAuditCandidate* right) {
            const auto left_key = rank_key(*left);
            const auto right_key = rank_key(*right);
            if (left_key != right_key) {
                return left_key > right_key;
            }
            if (left->transition.distance != right->transition.distance) {
                return left->transition.distance > right->transition.distance;
            }
            return ParameterMd(left->source_parameter) <
                   ParameterMd(right->source_parameter);
        });
    const int count =
        std::min(max_rows, static_cast<int>(ranked.size()));
    for (int index = 0; index < count; ++index) {
        out_rows.push_back(ToTransitionMontageRow(
            *ranked[static_cast<std::size_t>(index)], artifact,
            source_node, target_node, config, category, index + 1,
            next_manifest_index++));
    }
}

std::string TransitionMontageConclusion(
    const std::vector<TransitionAuditCandidate>& candidates,
    float bad_threshold) {
    int accepted_bad = 0;
    int accepted_near_threshold = 0;
    int rejected_jog_walk = 0;
    for (const TransitionAuditCandidate& candidate : candidates) {
        if (!candidate.accepted) {
            if (candidate.jog_walk_pair) {
                ++rejected_jog_walk;
            }
            continue;
        }
        if (candidate.transition.distance >= bad_threshold) {
            ++accepted_bad;
        } else if (candidate.transition.distance >= 0.9f * bad_threshold) {
            ++accepted_near_threshold;
        }
    }
    if (accepted_bad > 0) {
        return "FAIL_VISIBLE_TRANSITION_POP";
    }
    if (rejected_jog_walk >= 8) {
        return "FAIL_THRESHOLD_REJECTS_TOO_MUCH";
    }
    if (rejected_jog_walk > 0) {
        return "WARN_JOG_TRANSITION_WEAK";
    }
    if (accepted_near_threshold > 0) {
        return "WARN_MINOR_POP";
    }
    return "PASS_TRANSITIONS_VISUALLY_ACCEPTABLE";
}

TransitionMontageAuditData BuildTransitionMontageAudit(
    const pmg::BuiltPmgArtifact& artifact,
    const TransitionMontageAuditOptions& options) {
    TransitionMontageAuditData data;
    ReachableRegionAuditOptions reachable_options;
    reachable_options.pmg_path = options.pmg_path;
    reachable_options.output_csv = options.output_dir + "/_unused.csv";
    reachable_options.output_md = options.output_dir + "/_unused.md";
    reachable_options.output_dir = options.output_dir + "/_unused_maps";
    reachable_options.source_node = options.source_node;
    reachable_options.target_node = options.target_node;
    reachable_options.samples_per_axis = options.samples_per_axis;

    int edge_index = ResolveEdgeIndex(
        artifact, reachable_options, data.source_node, data.target_node);
    const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
    const pmg::PmgNode& source_node = artifact.graph.Node(edge.source_node);
    const pmg::PmgNode& target_node = artifact.graph.Node(edge.target_node);
    const pmg::EdgeBuildMetadata* edge_metadata =
        FindEdgeBuildMetadata(artifact, data.source_node, data.target_node);
    if (edge_metadata == nullptr) {
        throw std::runtime_error(
            "--audit-transition-montage: missing edge build metadata");
    }
    data.config = edge_metadata->config;
    const ReachableRegionAuditData reachable_data =
        BuildReachableRegionAudit(artifact, reachable_options);

    std::filesystem::create_directories(options.output_dir);
    std::filesystem::remove_all(reachable_options.output_dir);
    const pmg::ParameterSupport* source_support =
        source_node.motion_space.HasExplicitParameterSupport()
            ? &*source_node.motion_space.ExplicitSupport()
            : nullptr;
    const pmg::ParameterSupport* target_support =
        target_node.motion_space.HasExplicitParameterSupport()
            ? &*target_node.motion_space.ExplicitSupport()
            : nullptr;

    const std::vector<TransitionAuditCandidate> candidates =
        BuildTransitionAuditCandidates(
            artifact, edge, source_node, target_node, data.config,
            reachable_data, source_support, target_support,
            options.samples_per_axis);

    data.unique_transition_count = static_cast<int>(candidates.size());
    data.accepted_count = static_cast<int>(std::count_if(
        candidates.begin(), candidates.end(), [](const auto& candidate) {
            return candidate.accepted;
        }));
    data.rejected_count = data.unique_transition_count - data.accepted_count;
    data.rejected_jog_walk_count = static_cast<int>(std::count_if(
        candidates.begin(), candidates.end(), [](const auto& candidate) {
            return !candidate.accepted && candidate.jog_walk_pair;
        }));
    data.conclusion = TransitionMontageConclusion(
        candidates, data.config.bad_transition_threshold);
    data.manifest_csv_path =
        options.output_dir + "/transition_montage_manifest.csv";

    int next_manifest_index = 1;
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "worst_accepted_default", kWorstAcceptedSamples,
        [](const auto& candidate) { return candidate.accepted; },
        [](const auto& candidate) { return candidate.transition.distance; },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "near_threshold_accepted", kNearThresholdSamples,
        [](const auto& candidate) { return candidate.accepted; },
        [&](const auto& candidate) {
            return -std::abs(
                candidate.transition.distance -
                data.config.bad_transition_threshold);
        },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "rejected_high_d_jog_walk", kRejectedJogWalkSamples,
        [](const auto& candidate) {
            return !candidate.accepted && candidate.jog_walk_pair;
        },
        [](const auto& candidate) { return candidate.transition.distance; },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "anchor_to_anchor", kAnchorToAnchorSamples,
        [](const auto& candidate) {
            return candidate.source_is_anchor &&
                   candidate.requested_target_is_anchor;
        },
        [](const auto& candidate) { return candidate.transition.distance; },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "outside_request_projected", kOutsideRequestSamples,
        [](const auto& candidate) {
            return candidate.requested_target_projected;
        },
        [](const auto& candidate) { return candidate.transition.distance; },
        next_manifest_index, data.rows);
    AppendRankedTransitionRows(
        candidates, artifact, source_node, target_node, data.config,
        "source_dependent_shrinkage", kShrinkageSamples,
        [](const auto& candidate) {
            return candidate.source_coverage_available &&
                   (candidate.source_coverage <= 0.35f ||
                    SameParameterWithin(
                        candidate.source_parameter, {0.0f, 1.0f},
                        kTransitionMontageAnchorTolerance));
        },
        [](const auto& candidate) {
            return !candidate.accepted
                       ? candidate.transition.distance + 1000.0f
                       : candidate.transition.distance;
        },
        next_manifest_index, data.rows);
    return data;
}

void WriteTransitionMontageManifestCsv(
    const TransitionMontageAuditData& data) {
    std::ofstream csv(data.manifest_csv_path);
    if (!csv) {
        throw std::runtime_error(
            "--audit-transition-montage: cannot write manifest CSV");
    }
    csv << std::setprecision(9)
        << "category,category_rank,montage_id,replay_id,source_parameter,"
           "requested_target_parameter,effective_target_parameter,"
           "source_phase,source_frame,target_phase,target_frame,"
           "transition_metric_d,transition_class,root_jump,heading_jump,"
           "velocity_jump,blend_frames,notes\n";
    for (const TransitionMontageRow& row : data.rows) {
        csv << row.category << ',' << row.category_rank << ','
            << row.montage_id << ',' << row.replay_id << ','
            << ParameterCsv(row.source_parameter) << ','
            << ParameterCsv(row.requested_target_parameter) << ','
            << ParameterCsv(row.effective_target_parameter) << ','
            << row.source_phase << ',' << row.source_frame << ','
            << row.target_phase << ',' << row.target_frame << ','
            << row.transition_distance << ',' << row.transition_class << ','
            << row.root_jump << ',' << row.heading_jump << ','
            << row.velocity_jump << ',' << row.blend_frames << ",\""
            << row.notes << "\"\n";
    }
}

void WriteTransitionMontageMarkdown(
    const TransitionMontageAuditOptions& options,
    const TransitionMontageAuditData& data) {
    std::ofstream md(options.output_md);
    if (!md) {
        throw std::runtime_error(
            "--audit-transition-montage: cannot write markdown");
    }
    md << "# Transition Montage Audit\n\n";
    md << "## Purpose\n\n";
    md << "Report-only transition replay manifest for current PMG edge. "
          "No PMG artifact, threshold, or runtime behavior change. "
          "Conclusion uses current metric D plus root/heading/velocity seam "
          "diagnostics as proxy for later manual replay review.\n\n";
    md << "## Inputs\n\n";
    md << "- Artifact: `" << options.pmg_path << "`\n";
    md << "- Edge: `" << data.source_node << " -> " << data.target_node
       << "`\n";
    md << "- TGOOD/TBAD: `" << data.config.good_transition_threshold
       << " / " << data.config.bad_transition_threshold << "`\n";
    md << "- Blend frames: `" << data.config.distance_grid.window_size
       << "`\n";
    md << "- Unique source/target evaluations: " << data.unique_transition_count
       << "\n";
    md << "- Accepted / rejected requests: " << data.accepted_count << " / "
       << data.rejected_count << "\n";
    md << "- Rejected jog/walk requests: " << data.rejected_jog_walk_count
       << "\n";
    md << "- Conclusion: `" << data.conclusion << "`\n\n";
    md << "## Contract\n\n";
    md << "- `transition_metric_d` uses current offline build metric units.\n";
    md << "- `source_phase` / `target_phase` come from measured transition search.\n";
    md << "- `requested_target_parameter` is audited request.\n";
    md << "- `effective_target_parameter` is current runtime clamp/project result.\n";
    md << "- `REJECTED` means request fell outside interpolated target box for current source sample.\n";
    md << "- Manifest ids are deterministic for identical artifact + sampling.\n\n";

    const std::array<std::string, 6> categories = {
        "worst_accepted_default",
        "near_threshold_accepted",
        "rejected_high_d_jog_walk",
        "anchor_to_anchor",
        "outside_request_projected",
        "source_dependent_shrinkage"};
    for (const std::string& category : categories) {
        md << "## " << category << "\n\n";
        md << "| Rank | Montage | Source p | Req target p | Eff target p | Src phase/frame | Tgt phase/frame | D | Class | Root jump | Heading jump | Velocity jump | Notes |\n";
        md << "|---:|---|---|---|---|---|---|---:|---|---:|---:|---:|---|\n";
        for (const TransitionMontageRow& row : data.rows) {
            if (row.category != category) {
                continue;
            }
            md << "| " << row.category_rank << " | `" << row.montage_id
               << "` | " << ParameterMd(row.source_parameter) << " | "
               << ParameterMd(row.requested_target_parameter) << " | "
               << ParameterMd(row.effective_target_parameter) << " | "
               << row.source_phase << " / " << row.source_frame << " | "
               << row.target_phase << " / " << row.target_frame << " | "
               << row.transition_distance << " | `" << row.transition_class
               << "` | " << row.root_jump << " | " << row.heading_jump
               << " | " << row.velocity_jump << " | " << row.notes
               << " |\n";
        }
        md << "\n";
    }
    md << "## Artifacts\n\n";
    md << "- Markdown report: `" << options.output_md << "`\n";
    md << "- Manifest CSV: `" << data.manifest_csv_path << "`\n";
    md << "- Montage directory: `" << options.output_dir << "`\n";
}

struct ContactTransitionRow {
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector target_parameter;
    float transition_distance = 0.0f;
    int contact_mismatch_count = 0;
    float max_contact_foot_velocity = 0.0f;
    float total_skate_distance = 0.0f;
    float left_contact_confidence = 0.0f;
    float right_contact_confidence = 0.0f;
};

bool FrameInContact(
    const std::vector<pmg::ContactInterval>& contacts, int joint, int frame) {
    return std::any_of(contacts.begin(), contacts.end(), [&](const auto& contact) {
        return contact.joint_index == joint && frame >= contact.first_frame &&
               frame <= contact.last_frame;
    });
}

std::vector<ContactTransitionRow> BuildContactTransitionRows(
    const pmg::BuiltPmgArtifact& artifact,
    const ContactTransitionAuditOptions& options) {
    ReachableRegionAuditOptions reachable_options;
    reachable_options.source_node = options.source_node;
    reachable_options.target_node = options.target_node;
    std::string source_name;
    std::string target_name;
    const int edge_index = ResolveEdgeIndex(
        artifact, reachable_options, source_name, target_name);
    const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
    const pmg::PmgNode& source_node = artifact.graph.Node(edge.source_node);
    const pmg::PmgNode& target_node = artifact.graph.Node(edge.target_node);
    const pmg::EdgeBuildMetadata* metadata =
        FindEdgeBuildMetadata(artifact, source_name, target_name);
    if (metadata == nullptr) {
        throw std::runtime_error(
            "--audit-contact-transitions: missing edge build metadata");
    }
    std::vector<int> contact_joints =
        metadata->config.transition_metric.contact_joint_indices;
    if (contact_joints.empty()) {
        const auto registration = std::find_if(
            artifact.metadata.node_registrations.begin(),
            artifact.metadata.node_registrations.end(),
            [&](const auto& value) { return value.node_name == source_name; });
        if (registration != artifact.metadata.node_registrations.end()) {
            for (const std::string& joint_name : registration->contact_joints) {
                const int joint = ResolveJointIndex(artifact.skeleton, joint_name);
                if (joint >= 0) {
                    contact_joints.push_back(joint);
                }
            }
        }
    }
    if (contact_joints.empty()) {
        for (const char* joint_name : {"LeftAnkle", "RightAnkle"}) {
            const int joint = ResolveJointIndex(artifact.skeleton, joint_name);
            if (joint >= 0) {
                contact_joints.push_back(joint);
            }
        }
    }
    if (contact_joints.size() < 2) {
        throw std::runtime_error(
            "--audit-contact-transitions: edge needs two contact joints");
    }

    std::vector<ContactTransitionRow> rows;
    const auto source_samples = BuildAuditParameterSamples(
        source_node.motion_space, options.samples_per_axis);
    const auto target_samples = BuildAuditParameterSamples(
        target_node.motion_space, options.samples_per_axis);
    for (const auto& source_parameter : source_samples) {
        for (const auto& target_parameter : target_samples) {
            const auto lookup = edge.LookupInterpolated(
                source_parameter, target_parameter);
            if (!lookup.has_value() ||
                !lookup->target_parameter_box.Contains(target_parameter)) {
                continue;
            }
            const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
                source_parameter, artifact.metadata.frames_per_second);
            const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
                target_parameter, artifact.metadata.frames_per_second);
            const pmg::OptimalTransition transition = EvaluateConfiguredTransition(
                artifact.skeleton, source_clip, target_clip, metadata->config);
            const pmg::ContactDetectionSettings source_settings =
                pmg::EstimateContactSettings(
                    artifact.skeleton, source_clip, contact_joints);
            const pmg::ContactDetectionSettings target_settings =
                pmg::EstimateContactSettings(
                    artifact.skeleton, target_clip, contact_joints);
            const auto source_contacts = pmg::DetectContacts(
                artifact.skeleton, source_clip, contact_joints, source_settings);
            const auto target_contacts = pmg::DetectContacts(
                artifact.skeleton, target_clip, contact_joints, target_settings);

            ContactTransitionRow row;
            row.source_parameter = source_parameter;
            row.target_parameter = target_parameter;
            row.transition_distance = transition.distance;
            std::array<int, 2> contact_frames{};
            std::array<int, 2> blend_frames{};
            std::array<pmg::Vec3, 2> previous_positions{};
            const int frame_count = metadata->config.distance_grid.window_size;
            for (int blend_frame = 0; blend_frame < frame_count; ++blend_frame) {
                const int source_frame = std::clamp(
                    transition.source_frame + blend_frame, 0,
                    source_clip.NumFrames() - 1);
                const int target_frame = std::clamp(
                    transition.target_frame - frame_count + 1 + blend_frame, 0,
                    target_clip.NumFrames() - 1);
                const float linear_alpha = frame_count == 1
                    ? 1.0f
                    : static_cast<float>(blend_frame) /
                          static_cast<float>(frame_count - 1);
                const float alpha = linear_alpha * linear_alpha *
                                    (3.0f - 2.0f * linear_alpha);
                const pmg::Pose blended = pmg::BlendPose(
                    source_clip.frames[source_frame],
                    transition.alignment.Apply(target_clip.frames[target_frame]),
                    alpha);
                const auto positions = pmg::ComputeJointWorldPositions(
                    artifact.skeleton, blended);
                for (int foot = 0; foot < 2; ++foot) {
                    const int joint = contact_joints[foot];
                    const bool source_contact = FrameInContact(
                        source_contacts, joint, source_frame);
                    const bool target_contact = FrameInContact(
                        target_contacts, joint, target_frame);
                    if (source_contact != target_contact) {
                        ++row.contact_mismatch_count;
                    }
                    const bool in_contact = source_contact || target_contact;
                    ++blend_frames[foot];
                    if (in_contact) {
                        ++contact_frames[foot];
                    }
                    if (blend_frame > 0 && in_contact) {
                        const float step = HorizontalLength(
                            positions[joint] - previous_positions[foot]);
                        row.total_skate_distance += step;
                        row.max_contact_foot_velocity = std::max(
                            row.max_contact_foot_velocity,
                            step * artifact.metadata.frames_per_second);
                    }
                    previous_positions[foot] = positions[joint];
                }
            }
            row.left_contact_confidence = static_cast<float>(contact_frames[0]) /
                                          static_cast<float>(blend_frames[0]);
            row.right_contact_confidence = static_cast<float>(contact_frames[1]) /
                                           static_cast<float>(blend_frames[1]);
            rows.push_back(row);
        }
    }
    return rows;
}

std::string ContactTransitionConclusion(
    const std::vector<ContactTransitionRow>& rows) {
    const int mismatched = static_cast<int>(std::count_if(
        rows.begin(), rows.end(), [](const auto& row) {
            return row.contact_mismatch_count > 0;
        }));
    if (mismatched > static_cast<int>(rows.size() / 2)) {
        return "WARN_CONTACT_MISMATCH_COMMON";
    }
    if (mismatched > 0) {
        return "WARN_FOOT_SLIDING_VISIBLE";
    }
    return "PASS_CONTACT_ARTIFACT_LOW";
}

void WriteContactTransitionAudit(
    const ContactTransitionAuditOptions& options,
    const std::vector<ContactTransitionRow>& rows) {
    std::ofstream csv(options.output_csv);
    std::ofstream md(options.output_md);
    if (!csv || !md) {
        throw std::runtime_error(
            "--audit-contact-transitions: cannot write output");
    }
    csv << std::setprecision(9)
        << "source_parameter,target_parameter,transition_metric_d,"
           "contact_mismatch_count,max_contact_foot_velocity,"
           "total_skate_distance,left_contact_confidence,"
           "right_contact_confidence\n";
    for (const auto& row : rows) {
        csv << ParameterCsv(row.source_parameter) << ','
            << ParameterCsv(row.target_parameter) << ','
            << row.transition_distance << ',' << row.contact_mismatch_count << ','
            << row.max_contact_foot_velocity << ',' << row.total_skate_distance
            << ',' << row.left_contact_confidence << ','
            << row.right_contact_confidence << '\n';
    }
    std::vector<ContactTransitionRow> ranked = rows;
    std::sort(ranked.begin(), ranked.end(), [](const auto& left, const auto& right) {
        return left.total_skate_distance > right.total_skate_distance;
    });
    md << "# Contact Transition Audit\n\n"
       << "Report-only contact/foot-slide diagnostics. Distances use motion "
          "corpus units; velocity uses corpus units/s; confidence is contact "
          "frame fraction over blend. Contact is estimated independently per "
          "generated clip. No runtime or transition rejection change.\n\n"
       << "- Evaluated accepted transitions: " << rows.size() << "\n"
       << "- Conclusion: `" << ContactTransitionConclusion(rows) << "`\n\n"
       << "| Rank | Source | Target | D | Mismatch frames | Max foot velocity | Skate distance | Left confidence | Right confidence |\n"
       << "|---:|---|---|---:|---:|---:|---:|---:|---:|\n";
    const int count = std::min(20, static_cast<int>(ranked.size()));
    for (int index = 0; index < count; ++index) {
        const auto& row = ranked[index];
        md << "| " << index + 1 << " | " << ParameterMd(row.source_parameter)
           << " | " << ParameterMd(row.target_parameter) << " | "
           << row.transition_distance << " | " << row.contact_mismatch_count
           << " | " << row.max_contact_foot_velocity << " | "
           << row.total_skate_distance << " | " << row.left_contact_confidence
           << " | " << row.right_contact_confidence << " |\n";
    }
}

constexpr float kAcceptanceNearThresholdRatio = 0.9f;
constexpr float kShrinkageCoverageWarnThreshold = 0.35f;
constexpr float kKnownBadCaseTolerance = 1.0e-5f;

bool IsKnownAcceptedBadCase(
    const pmg::ParameterVector& source_parameter,
    const pmg::ParameterVector& target_parameter) {
    const bool original_b1 = SameParameterWithin(
        source_parameter, {0.1875f, 0.375f}, kKnownBadCaseTolerance) &&
        SameParameterWithin(
            target_parameter, {0.025f, 0.875f}, kKnownBadCaseTolerance);
    const bool fail_vis_001 = SameParameterWithin(
        source_parameter, {-0.04f, 0.8f}, kKnownBadCaseTolerance) &&
        SameParameterWithin(
            target_parameter, {0.5f, 0.0f}, kKnownBadCaseTolerance);
    return original_b1 || fail_vis_001;
}

std::string BuildAcceptanceConsistencyNotes(
    const TransitionAuditCandidate& candidate,
    float bad_threshold) {
    std::vector<std::string> notes;
    if (candidate.accepted &&
        candidate.transition.distance >= bad_threshold) {
        notes.push_back(
            "accepted by interpolated target box but measured D is BAD");
        if (!candidate.effective_target_projected) {
            notes.push_back(
                "likely box overreach: acceptance gates box membership, not D");
        }
    } else if (candidate.accepted &&
               candidate.transition.distance >=
                   kAcceptanceNearThresholdRatio * bad_threshold) {
        notes.push_back("accepted near TBAD; small box change may flip class");
    }
    if (candidate.source_coverage_available &&
        candidate.source_coverage <= kShrinkageCoverageWarnThreshold) {
        notes.push_back("source-dependent shrinkage from reachable-region audit");
    }
    if (candidate.jog_walk_pair) {
        notes.push_back("jog/walk speed-mismatch case");
    }
    if (candidate.effective_target_projected) {
        notes.push_back("runtime projects request inside current box/support");
    }
    if (IsKnownAcceptedBadCase(
            candidate.source_parameter,
            candidate.requested_target_parameter)) {
        notes.push_back("known B1 failing case");
    }
    if (notes.empty()) {
        notes.push_back("box and measured D agree");
    }

    std::ostringstream out;
    for (std::size_t index = 0; index < notes.size(); ++index) {
        if (index > 0) {
            out << "; ";
        }
        out << notes[index];
    }
    return out.str();
}

TransitionAcceptanceConsistencyRow BuildAcceptanceConsistencyRow(
    const TransitionAuditCandidate& candidate,
    const pmg::BuiltPmgArtifact& artifact,
    const pmg::PmgNode& source_node,
    const pmg::PmgNode& target_node,
    const pmg::PmgBuilderConfig& config,
    const pmg::TransitionQualityGateConfig& quality_gate,
    const pmg::ParameterSupport* source_support,
    const pmg::ParameterSupport* target_support) {
    const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
        candidate.source_parameter, artifact.metadata.frames_per_second);
    const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
        candidate.requested_target_parameter, artifact.metadata.frames_per_second);
    const pmg::Pose source_pose = source_clip.frames[static_cast<std::size_t>(
        candidate.transition.source_frame)];
    const pmg::Pose aligned_target_pose = candidate.transition.alignment.Apply(
        target_clip.frames[static_cast<std::size_t>(
            candidate.transition.target_frame)]);

    TransitionAcceptanceConsistencyRow row;
    row.source_parameter = candidate.source_parameter;
    row.requested_target_parameter = candidate.requested_target_parameter;
    row.effective_target_parameter = candidate.effective_target_parameter;
    row.interpolated_target_box = candidate.interpolated_target_box;
    row.accepted_by_box = candidate.accepted;
    row.transition_distance = candidate.transition.distance;
    row.metric_class = candidate.transition_class;
    row.acceptance_violation =
        candidate.accepted &&
        candidate.transition.distance >= config.bad_transition_threshold;
    row.distance_over_tbad =
        candidate.transition.distance - config.bad_transition_threshold;
    row.source_coverage_available = candidate.source_coverage_available;
    row.source_coverage = candidate.source_coverage;
    row.source_phase = candidate.transition.source_phase;
    row.source_frame = candidate.transition.source_frame;
    row.target_phase = candidate.transition.target_phase;
    row.target_frame = candidate.transition.target_frame;
    row.root_jump = RootJumpAtTransition(source_pose, aligned_target_pose);
    row.heading_jump =
        HeadingJumpAtTransition(source_pose, aligned_target_pose);
    row.velocity_jump = VelocityJumpAtTransition(
        source_clip, target_clip, candidate.transition.alignment,
        candidate.transition.source_frame, candidate.transition.target_frame);
    if (candidate.accepted) {
        row.quality = MeasureExactTransitionQuality(
            artifact, source_node, target_node, config,
            candidate.source_parameter, candidate.effective_target_parameter,
            candidate.transition, 3, 3);
        const pmg::TransitionQualityGateDecision decision =
            pmg::EvaluateTransitionQualityGate(*row.quality, quality_gate);
        const bool no_runtime_transition = &source_node == &target_node &&
            candidate.source_parameter == candidate.effective_target_parameter;
        row.quality_gate_accepts = no_runtime_transition || decision.accepted;
        row.quality_reject_reason =
            no_runtime_transition
                ? "none"
                : pmg::TransitionQualityGateReasonName(decision.reason);
    }
    row.nearest_source_anchor_label =
        source_support != nullptr
            ? NearestAnchorLabel(*source_support, candidate.source_parameter)
            : "";
    row.nearest_target_anchor_label =
        target_support != nullptr
            ? NearestAnchorLabel(
                  *target_support, candidate.requested_target_parameter)
            : "";
    row.jog_walk_pair = candidate.jog_walk_pair;
    row.notes = BuildAcceptanceConsistencyNotes(
        candidate, config.bad_transition_threshold);
    return row;
}

std::string NearestMetricEvidence(
    const TransitionAcceptanceConsistencyRow& row,
    const std::vector<TransitionAuditCandidate>& candidates,
    float good_threshold,
    float bad_threshold,
    bool find_good) {
    const TransitionAuditCandidate* nearest = nullptr;
    float nearest_distance = std::numeric_limits<float>::infinity();
    for (const TransitionAuditCandidate& candidate : candidates) {
        if (!SameParameterWithin(
                candidate.source_parameter, row.source_parameter,
                kTransitionMontageAnchorTolerance)) {
            continue;
        }
        const bool matches = find_good
                                 ? candidate.transition.distance <= good_threshold
                                 : candidate.transition.distance >= bad_threshold;
        const float parameter_distance = pmg::SquaredDistance(
            candidate.requested_target_parameter,
            row.requested_target_parameter);
        if (matches && parameter_distance < nearest_distance) {
            nearest = &candidate;
            nearest_distance = parameter_distance;
        }
    }
    if (nearest == nullptr) {
        return "none";
    }
    std::ostringstream evidence;
    evidence << "p=" << ParameterMd(nearest->requested_target_parameter)
             << ", D=" << nearest->transition.distance;
    return evidence.str();
}

std::string OverreachOrigin(
    const TransitionAcceptanceConsistencyRow& row,
    const pmg::PmgEdge& edge) {
    if (!row.acceptance_violation) {
        return "none";
    }
    float nearest_source_distance = std::numeric_limits<float>::infinity();
    for (const pmg::TransitionSample& sample : edge.samples) {
        nearest_source_distance = std::min(
            nearest_source_distance,
            pmg::SquaredDistance(
                sample.source_parameter, row.source_parameter));
    }
    for (const pmg::TransitionSample& sample : edge.samples) {
        const float source_distance = pmg::SquaredDistance(
            sample.source_parameter, row.source_parameter);
        if (std::abs(source_distance - nearest_source_distance) <=
                kTransitionMontageAnchorTolerance &&
            sample.target_parameter_box.Contains(
                row.requested_target_parameter)) {
            return "box_construction";
        }
    }
    return "interpolation";
}

template <typename Predicate, typename RankKey>
void WriteAcceptanceConsistencySection(
    std::ofstream& md,
    const std::string& title,
    const std::vector<TransitionAcceptanceConsistencyRow>& rows,
    Predicate predicate,
    RankKey rank_key) {
    std::vector<const TransitionAcceptanceConsistencyRow*> ranked;
    for (const TransitionAcceptanceConsistencyRow& row : rows) {
        if (predicate(row)) {
            ranked.push_back(&row);
        }
    }
    std::sort(
        ranked.begin(), ranked.end(),
        [&](const TransitionAcceptanceConsistencyRow* left,
            const TransitionAcceptanceConsistencyRow* right) {
            const auto left_key = rank_key(*left);
            const auto right_key = rank_key(*right);
            if (left_key != right_key) {
                return left_key > right_key;
            }
            if (left->transition_distance != right->transition_distance) {
                return left->transition_distance > right->transition_distance;
            }
            return ParameterMd(left->source_parameter) <
                   ParameterMd(right->source_parameter);
        });

    md << "## " << title << "\n\n";
    if (ranked.empty()) {
        md << "No rows.\n\n";
        return;
    }
    md << "| Rank | Source p | Req target p | Target box | Box | Quality | Reject reason | D | Class | D-TBAD | Nearest GOOD | Nearest BAD | Origin | Source coverage | Notes |\n";
    md << "|---:|---|---|---|---|---|---|---:|---|---:|---|---|---|---:|---|\n";
    for (std::size_t index = 0; index < ranked.size(); ++index) {
        const TransitionAcceptanceConsistencyRow& row =
            *ranked[index];
        md << "| " << (index + 1) << " | "
           << ParameterMd(row.source_parameter) << " | "
           << ParameterMd(row.requested_target_parameter) << " | "
           << ParameterMd(row.interpolated_target_box.min_corner) << " .. "
           << ParameterMd(row.interpolated_target_box.max_corner) << " | "
           << (row.accepted_by_box ? "`true`" : "`false`") << " | "
           << (row.quality_gate_accepts ? "`accept`" : "`reject`") << " | `"
           << row.quality_reject_reason << "` | "
           << row.transition_distance << " | `" << row.metric_class
           << "` | " << row.distance_over_tbad << " | "
           << row.nearest_good_evidence << " | "
           << row.nearest_bad_evidence << " | `"
           << row.overreach_origin << "` | ";
        if (row.source_coverage_available) {
            md << row.source_coverage;
        } else {
            md << "n/a";
        }
        md << " | " << row.notes << " |\n";
    }
    md << "\n";
}

TransitionAcceptanceConsistencyAuditData BuildTransitionAcceptanceConsistencyAudit(
    const pmg::BuiltPmgArtifact& artifact,
    const TransitionAcceptanceConsistencyAuditOptions& options) {
    TransitionAcceptanceConsistencyAuditData data;
    ReachableRegionAuditOptions reachable_options;
    reachable_options.pmg_path = options.pmg_path;
    const std::filesystem::path output_csv_path(options.output_csv);
    const std::filesystem::path temp_root =
        output_csv_path.has_parent_path()
            ? output_csv_path.parent_path()
            : std::filesystem::path(".");
    reachable_options.output_csv =
        (temp_root / "_acceptance_consistency_reachable_unused.csv").string();
    reachable_options.output_md =
        (temp_root / "_acceptance_consistency_reachable_unused.md").string();
    reachable_options.output_dir =
        (temp_root / "_acceptance_consistency_reachable_unused_maps").string();
    reachable_options.source_node = options.source_node;
    reachable_options.target_node = options.target_node;
    reachable_options.samples_per_axis = options.samples_per_axis;

    int edge_index = ResolveEdgeIndex(
        artifact, reachable_options, data.source_node, data.target_node);
    const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
    const pmg::PmgNode& source_node = artifact.graph.Node(edge.source_node);
    const pmg::PmgNode& target_node = artifact.graph.Node(edge.target_node);
    const pmg::EdgeBuildMetadata* edge_metadata =
        FindEdgeBuildMetadata(artifact, data.source_node, data.target_node);
    if (edge_metadata == nullptr) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency: missing edge build metadata");
    }
    data.config = edge_metadata->config;
    const ReachableRegionAuditData reachable_data =
        BuildReachableRegionAudit(artifact, reachable_options);
    std::filesystem::remove(reachable_options.output_csv);
    std::filesystem::remove(reachable_options.output_md);
    std::filesystem::remove_all(reachable_options.output_dir);
    const pmg::ParameterSupport* source_support =
        source_node.motion_space.HasExplicitParameterSupport()
            ? &*source_node.motion_space.ExplicitSupport()
            : nullptr;
    const pmg::ParameterSupport* target_support =
        target_node.motion_space.HasExplicitParameterSupport()
            ? &*target_node.motion_space.ExplicitSupport()
            : nullptr;
    const std::vector<TransitionAuditCandidate> candidates =
        BuildTransitionAuditCandidates(
            artifact, edge, source_node, target_node, data.config,
            reachable_data, source_support, target_support,
            options.samples_per_axis);

    bool any_box_overreach = false;
    for (const TransitionAuditCandidate& candidate : candidates) {
        TransitionAcceptanceConsistencyRow row =
            BuildAcceptanceConsistencyRow(
                candidate, artifact, source_node, target_node, data.config,
                options.quality_gate,
                source_support, target_support);
        row.nearest_good_evidence = NearestMetricEvidence(
            row, candidates, data.config.good_transition_threshold,
            data.config.bad_transition_threshold, true);
        row.nearest_bad_evidence = NearestMetricEvidence(
            row, candidates, data.config.good_transition_threshold,
            data.config.bad_transition_threshold, false);
        row.overreach_origin = OverreachOrigin(row, edge);
        data.accepted_bad_count += row.acceptance_violation ? 1 : 0;
        data.accepted_bad_after_quality_gate +=
            row.acceptance_violation && row.quality_gate_accepts ? 1 : 0;
        data.box_accepted_quality_rejected +=
            row.accepted_by_box && !row.quality_gate_accepts ? 1 : 0;
        if (row.accepted_by_box && !row.quality_gate_accepts) {
            ++data.quality_reject_reasons[row.quality_reject_reason];
        }
        data.near_threshold_accepted_count +=
            row.accepted_by_box &&
                    row.transition_distance <
                        data.config.bad_transition_threshold &&
                    row.transition_distance >=
                        kAcceptanceNearThresholdRatio *
                            data.config.bad_transition_threshold
                ? 1
                : 0;
        data.accepted_near_threshold_after_quality_gate +=
            row.accepted_by_box && row.quality_gate_accepts &&
                    row.transition_distance < data.config.bad_transition_threshold &&
                    row.transition_distance >=
                        kAcceptanceNearThresholdRatio *
                            data.config.bad_transition_threshold
                ? 1
                : 0;
        data.jog_walk_row_count += row.jog_walk_pair ? 1 : 0;
        data.shrinkage_row_count +=
            row.source_coverage_available &&
                    row.source_coverage <=
                        kShrinkageCoverageWarnThreshold
                ? 1
                : 0;
        data.known_bad_case_found =
            data.known_bad_case_found ||
            IsKnownAcceptedBadCase(
                row.source_parameter, row.requested_target_parameter);
        any_box_overreach =
            any_box_overreach ||
            (row.acceptance_violation &&
             SameParameterWithin(
                 row.requested_target_parameter,
                 row.effective_target_parameter,
                 kTransitionMontageProjectedNoteTolerance));
        data.rows.push_back(std::move(row));
    }

    if (data.accepted_bad_count > 0) {
        data.consistency_conclusion = "FAIL_OVERREACH_REMAINS";
    } else if (data.shrinkage_row_count > 0) {
        data.consistency_conclusion = "PASS_BUT_COVERAGE_SHRUNK";
    } else {
        data.consistency_conclusion = "PASS_NO_ACCEPTED_BAD_TRANSITIONS";
    }

    if (any_box_overreach) {
        data.root_cause_conclusion = "FAIL_EDGE_BOX_OVERREACH";
    } else {
        data.root_cause_conclusion = data.consistency_conclusion;
    }
    return data;
}

void WriteTransitionAcceptanceConsistencyCsv(
    const TransitionAcceptanceConsistencyAuditOptions& options,
    const TransitionAcceptanceConsistencyAuditData& data) {
    std::ofstream csv(options.output_csv);
    if (!csv) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency: cannot write CSV");
    }
    csv << std::setprecision(9)
        << "source_parameter,requested_target_parameter,effective_target_parameter,"
           "interpolated_target_box_min,interpolated_target_box_max,"
           "accepted_by_box,transition_metric_d,metric_class,acceptance_violation,"
           "distance_over_tbad,source_coverage,source_phase,source_frame,"
           "target_phase,target_frame,root_jump,heading_jump,velocity_jump,"
           "root_speed_ratio,yaw_rate_ratio,left_foot_drift,right_foot_drift,"
           "left_foot_height_before,left_foot_height_after,"
           "right_foot_height_before,right_foot_height_after,"
           "left_contact_before,left_contact_after,right_contact_before,"
           "right_contact_after,quality_gate_accepts,quality_reject_reason,"
           "nearest_source_anchor_label,nearest_target_anchor_label,"
           "nearest_good_evidence,nearest_bad_evidence,overreach_origin,notes\n";
    for (const TransitionAcceptanceConsistencyRow& row : data.rows) {
        csv << ParameterCsv(row.source_parameter) << ','
            << ParameterCsv(row.requested_target_parameter) << ','
            << ParameterCsv(row.effective_target_parameter) << ','
            << ParameterCsv(row.interpolated_target_box.min_corner) << ','
            << ParameterCsv(row.interpolated_target_box.max_corner) << ','
            << (row.accepted_by_box ? "true" : "false") << ','
            << row.transition_distance << ',' << row.metric_class << ','
            << (row.acceptance_violation ? "true" : "false") << ','
            << row.distance_over_tbad << ',';
        if (row.source_coverage_available) {
            csv << row.source_coverage;
        }
        csv << ',' << row.source_phase << ',' << row.source_frame << ','
            << row.target_phase << ',' << row.target_frame << ','
            << row.root_jump << ',' << row.heading_jump << ','
            << row.velocity_jump << ',';
        if (row.quality.has_value()) {
            const pmg::TransitionQualityRecord& quality = *row.quality;
            csv << quality.root_speed_ratio << ',' << quality.yaw_rate_ratio << ','
                << quality.left_foot_drift << ',' << quality.right_foot_drift << ','
                << quality.left_foot_height_before << ','
                << quality.left_foot_height_after << ','
                << quality.right_foot_height_before << ','
                << quality.right_foot_height_after << ','
                << pmg::TransitionContactStateName(quality.left_contact_before)
                << ',' << pmg::TransitionContactStateName(quality.left_contact_after)
                << ',' << pmg::TransitionContactStateName(quality.right_contact_before)
                << ',' << pmg::TransitionContactStateName(quality.right_contact_after);
        } else {
            csv << ",,,,,,,,,,,";
        }
        csv << ',' << (row.quality_gate_accepts ? "true" : "false") << ','
            << row.quality_reject_reason << ",\""
            << row.nearest_source_anchor_label << "\",\""
            << row.nearest_target_anchor_label << "\",\""
            << row.nearest_good_evidence << "\",\""
            << row.nearest_bad_evidence << "\","
            << row.overreach_origin << ",\""
            << row.notes << "\"\n";
    }
}

void WriteTransitionAcceptanceConsistencyMarkdown(
    const TransitionAcceptanceConsistencyAuditOptions& options,
    const TransitionAcceptanceConsistencyAuditData& data) {
    std::ofstream md(options.output_md);
    if (!md) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency: cannot write markdown");
    }
    md << "# Transition Acceptance Consistency Audit\n\n";
    md << "## Purpose\n\n";
    md << "Gate interpolated edge acceptance against sampled transition "
          "quality under current PMG artifact. Any accepted row with "
          "D >= TBAD fails command.\n\n";
    md << "## Inputs\n\n";
    md << "- Artifact: `" << options.pmg_path << "`\n";
    md << "- Edge: `" << data.source_node << " -> " << data.target_node
       << "`\n";
    md << "- TGOOD/TBAD: `" << data.config.good_transition_threshold
       << " / " << data.config.bad_transition_threshold << "`\n";
    md << "- Evaluated source/target rows: " << data.rows.size() << "\n";
    md << "- Accepted BAD transitions: " << data.accepted_bad_count << "\n";
    md << "- Accepted BAD after quality gate: "
       << data.accepted_bad_after_quality_gate << "\n";
    md << "- Near-threshold accepted transitions: "
       << data.near_threshold_accepted_count << "\n";
    md << "- Near-threshold accepted after quality gate: "
       << data.accepted_near_threshold_after_quality_gate << "\n";
    md << "- Box-accepted, quality-rejected: "
       << data.box_accepted_quality_rejected << "\n";
    md << "- Quality reject reasons: ";
    if (data.quality_reject_reasons.empty()) {
        md << "`none`";
    } else {
        bool first = true;
        for (const auto& [reason, count] : data.quality_reject_reasons) {
            if (!first) md << ", ";
            md << '`' << reason << "=" << count << '`';
            first = false;
        }
    }
    md << "\n";
    md << "- Known bad case found: "
       << (data.known_bad_case_found ? "`yes`" : "`no`") << "\n";
    md << "- Consistency conclusion: `" << data.consistency_conclusion
       << "`\n";
    md << "- Root-cause conclusion: `" << data.root_cause_conclusion
       << "`\n\n";
    md << "## Root Cause\n\n";
    md << "- Acceptance means requested target lies inside interpolated "
          "target box.\n";
    md << "- Measured `D` is evaluated independently at requested target.\n";
    md << "- Each row reports interpolated box, nearest sampled GOOD/BAD "
          "evidence at same source parameter, and overreach origin.\n";
    md << "- `box_construction` means target is inside a stored sampled box; "
          "`interpolation` means only interpolated lookup introduced it.\n\n";

    WriteAcceptanceConsistencySection(
        md, "Accepted BAD transitions", data.rows,
        [](const auto& row) { return row.acceptance_violation; },
        [](const auto& row) { return row.distance_over_tbad; });
    WriteAcceptanceConsistencySection(
        md, "Near-threshold accepted transitions", data.rows,
        [&](const auto& row) {
            return row.accepted_by_box &&
                   row.transition_distance < data.config.bad_transition_threshold &&
                   row.distance_over_tbad >=
                       -(1.0f - kAcceptanceNearThresholdRatio) *
                           data.config.bad_transition_threshold;
        },
        [](const auto& row) { return -std::abs(row.distance_over_tbad); });
    WriteAcceptanceConsistencySection(
        md, "Known bad case", data.rows,
        [](const auto& row) {
            return IsKnownAcceptedBadCase(
                row.source_parameter, row.requested_target_parameter);
        },
        [](const auto& row) { return row.transition_distance; });
    WriteAcceptanceConsistencySection(
        md, "Source-dependent shrinkage focus", data.rows,
        [](const auto& row) {
            return (row.source_coverage_available &&
                    row.source_coverage <=
                        kShrinkageCoverageWarnThreshold) ||
                   SameParameterWithin(
                       row.source_parameter, {0.0f, 1.0f},
                       kKnownBadCaseTolerance);
        },
        [](const auto& row) { return row.transition_distance; });
    WriteAcceptanceConsistencySection(
        md, "Jog/walk focus", data.rows,
        [](const auto& row) { return row.jog_walk_pair; },
        [](const auto& row) { return row.transition_distance; });
    md << "## Artifacts\n\n";
    md << "- CSV: `" << options.output_csv << "`\n";
    md << "- Markdown: `" << options.output_md << "`\n";
}

struct TransitionProbeResult {
    pmg::ParameterVector source_parameter;
    pmg::ParameterVector requested_target_parameter;
    pmg::ParameterVector effective_target_parameter;
    pmg::ParameterAabb target_box;
    bool accepted_by_box = false;
    pmg::OptimalTransition transition;
    std::string metric_class;
    float root_jump = 0.0f;
    float heading_jump = 0.0f;
    float velocity_jump = 0.0f;
    pmg::TransitionQualityRecord quality;
    pmg::TransitionQualityGateDecision quality_decision;
    bool final_accepted = false;
    std::string reject_reason;
};

bool ContactMismatch(
    pmg::TransitionContactState before,
    pmg::TransitionContactState after) {
    return before != pmg::TransitionContactState::kUnknown &&
           after != pmg::TransitionContactState::kUnknown && before != after;
}

TransitionProbeResult BuildTransitionProbe(
    const pmg::BuiltPmgArtifact& artifact,
    const TransitionProbeOptions& options) {
    ReachableRegionAuditOptions edge_options;
    edge_options.source_node = options.source_node;
    edge_options.target_node = options.target_node;
    std::string source_name;
    std::string target_name;
    const int edge_index = ResolveEdgeIndex(
        artifact, edge_options, source_name, target_name);
    const pmg::PmgEdge& edge = artifact.graph.Edge(edge_index);
    const pmg::PmgNode& source_node = artifact.graph.Node(edge.source_node);
    const pmg::PmgNode& target_node = artifact.graph.Node(edge.target_node);
    const pmg::EdgeBuildMetadata* metadata =
        FindEdgeBuildMetadata(artifact, source_name, target_name);
    if (metadata == nullptr) {
        throw std::runtime_error("--probe-transition: missing edge build metadata");
    }
    const auto lookup = edge.LookupInterpolated(
        options.source_parameter, options.target_parameter);
    if (!lookup.has_value()) {
        throw std::runtime_error("--probe-transition: edge lookup returned no transition");
    }

    TransitionProbeResult result;
    result.source_parameter = options.source_parameter;
    result.requested_target_parameter = options.target_parameter;
    result.target_box = lookup->target_parameter_box;
    result.accepted_by_box = result.target_box.Contains(options.target_parameter);
    result.effective_target_parameter = result.target_box.Clamp(options.target_parameter);
    if (target_node.motion_space.HasExplicitParameterSupport()) {
        result.effective_target_parameter =
            target_node.motion_space.ExplicitSupport()->ProjectInside(
                options.target_parameter, result.target_box);
    }

    const pmg::MotionClip source_clip = source_node.motion_space.GenerateClip(
        options.source_parameter, artifact.metadata.frames_per_second);
    const pmg::MotionClip target_clip = target_node.motion_space.GenerateClip(
        result.effective_target_parameter, artifact.metadata.frames_per_second);
    result.transition = EvaluateConfiguredTransition(
        artifact.skeleton, source_clip, target_clip, metadata->config);
    result.metric_class = PoseSeamClass(
        result.transition.distance, metadata->config);
    const pmg::Pose& source_pose = source_clip.frames[
        static_cast<std::size_t>(result.transition.source_frame)];
    const pmg::Pose aligned_target_pose = result.transition.alignment.Apply(
        target_clip.frames[static_cast<std::size_t>(result.transition.target_frame)]);
    result.root_jump = RootJumpAtTransition(source_pose, aligned_target_pose);
    result.heading_jump = HeadingJumpAtTransition(source_pose, aligned_target_pose);
    result.velocity_jump = VelocityJumpAtTransition(
        source_clip, target_clip, result.transition.alignment,
        result.transition.source_frame, result.transition.target_frame);
    result.quality = MeasureExactTransitionQuality(
        artifact, source_node, target_node, metadata->config,
        options.source_parameter, result.effective_target_parameter,
        result.transition, options.frames_before, options.frames_after);
    result.quality_decision = pmg::EvaluateTransitionQualityGate(
        result.quality, options.quality_gate);
    if (&source_node == &target_node &&
        options.source_parameter == result.effective_target_parameter) {
        // RuntimeController does not schedule an exact same-node/parameter move.
        result.quality_decision = {};
    }
    result.final_accepted = result.accepted_by_box && result.quality_decision.accepted;
    result.reject_reason = !result.accepted_by_box
                               ? "outside_target_box"
                               : pmg::TransitionQualityGateReasonName(
                                     result.quality_decision.reason);
    return result;
}

void WriteTransitionProbeCsv(
    const std::string& path,
    const TransitionProbeResult& row) {
    std::ofstream csv(path);
    if (!csv) throw std::runtime_error("--probe-transition: cannot write CSV");
    csv << std::setprecision(9)
        << "source_parameter,requested_target_parameter,effective_target_parameter,"
           "target_box_min,target_box_max,accepted_by_box,transition_metric_d,"
           "metric_class,source_phase,source_frame,target_phase,target_frame,"
           "root_jump,heading_jump,velocity_jump,root_speed_ratio,yaw_rate_ratio,"
           "left_foot_drift,right_foot_drift,left_contact_before,left_contact_after,"
           "right_contact_before,right_contact_after,left_foot_height_before,"
           "left_foot_height_after,left_foot_height_delta,right_foot_height_before,"
           "right_foot_height_after,right_foot_height_delta,"
           "left_contact_mismatch,right_contact_mismatch,quality_gate_decision,"
           "reject_reason\n";
    const auto& q = row.quality;
    csv << ParameterCsv(row.source_parameter) << ','
        << ParameterCsv(row.requested_target_parameter) << ','
        << ParameterCsv(row.effective_target_parameter) << ','
        << ParameterCsv(row.target_box.min_corner) << ','
        << ParameterCsv(row.target_box.max_corner) << ','
        << (row.accepted_by_box ? "true" : "false") << ','
        << row.transition.distance << ',' << row.metric_class << ','
        << row.transition.source_phase << ',' << row.transition.source_frame << ','
        << row.transition.target_phase << ',' << row.transition.target_frame << ','
        << row.root_jump << ',' << row.heading_jump << ',' << row.velocity_jump << ','
        << q.root_speed_ratio << ',' << q.yaw_rate_ratio << ','
        << q.left_foot_drift << ',' << q.right_foot_drift << ','
        << pmg::TransitionContactStateName(q.left_contact_before) << ','
        << pmg::TransitionContactStateName(q.left_contact_after) << ','
        << pmg::TransitionContactStateName(q.right_contact_before) << ','
        << pmg::TransitionContactStateName(q.right_contact_after) << ','
        << q.left_foot_height_before << ',' << q.left_foot_height_after << ','
        << q.left_foot_height_delta << ',' << q.right_foot_height_before << ','
        << q.right_foot_height_after << ',' << q.right_foot_height_delta << ','
        << (ContactMismatch(q.left_contact_before, q.left_contact_after) ? "true" : "false") << ','
        << (ContactMismatch(q.right_contact_before, q.right_contact_after) ? "true" : "false") << ','
        << (row.final_accepted ? "accept" : "reject") << ','
        << row.reject_reason << '\n';
}

void WriteTransitionProbeMarkdown(
    const std::string& path,
    const TransitionProbeOptions& options,
    const TransitionProbeResult& row) {
    std::ofstream md(path);
    if (!md) throw std::runtime_error("--probe-transition: cannot write markdown");
    const auto& q = row.quality;
    md << "# Exact Transition Probe\n\n"
       << "- Artifact: `" << options.pmg_path << "`\n"
       << "- Source parameter: " << ParameterMd(row.source_parameter) << "\n"
       << "- Requested/effective target: "
       << ParameterMd(row.requested_target_parameter) << " / "
       << ParameterMd(row.effective_target_parameter) << "\n"
       << "- Target box: " << ParameterMd(row.target_box.min_corner) << " .. "
       << ParameterMd(row.target_box.max_corner) << "\n"
       << "- Box accepted: `" << (row.accepted_by_box ? "true" : "false") << "`\n"
       << "- D/class: " << row.transition.distance << " / `" << row.metric_class << "`\n"
       << "- Source/target phase (frame): " << row.transition.source_phase << " ("
       << row.transition.source_frame << ") / " << row.transition.target_phase << " ("
       << row.transition.target_frame << ")\n"
       << "- Root/heading/velocity jump: " << row.root_jump << " / "
       << row.heading_jump << " / " << row.velocity_jump << "\n"
       << "- Root speed/yaw rate ratio: " << q.root_speed_ratio << " / "
       << q.yaw_rate_ratio << "\n"
       << "- Left/right foot drift: " << q.left_foot_drift << " / "
       << q.right_foot_drift << "\n"
       << "- Left foot contact before/after, height before/after: `"
       << pmg::TransitionContactStateName(q.left_contact_before) << "` / `"
       << pmg::TransitionContactStateName(q.left_contact_after) << "`, "
       << q.left_foot_height_before << " / " << q.left_foot_height_after << "\n"
       << "- Right foot contact before/after, height before/after: `"
       << pmg::TransitionContactStateName(q.right_contact_before) << "` / `"
       << pmg::TransitionContactStateName(q.right_contact_after) << "`, "
       << q.right_foot_height_before << " / " << q.right_foot_height_after << "\n"
       << "- Quality decision: `" << (row.final_accepted ? "accept" : "reject")
       << "`\n- Reject reason: `" << row.reject_reason << "`\n";
}

TransitionMontageAuditOptions ParseTransitionMontageAuditOptions(
    int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error(
            "--audit-transition-montage needs <graph.pmg>");
    }
    TransitionMontageAuditOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else if (option == "--output-dir") {
            options.output_dir = require_value("--output-dir");
        } else if (option == "--source-node") {
            options.source_node = require_value("--source-node");
        } else if (option == "--target-node") {
            options.target_node = require_value("--target-node");
        } else if (option == "--samples") {
            options.samples_per_axis = std::stoi(require_value("--samples"));
        } else {
            throw std::runtime_error(
                "unknown audit-transition-montage option '" + option + "'");
        }
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-transition-montage requires --output-md");
    }
    if (options.output_dir.empty()) {
        throw std::runtime_error(
            "--audit-transition-montage requires --output-dir");
    }
    if (options.samples_per_axis < 2) {
        throw std::runtime_error(
            "--audit-transition-montage: --samples must be at least 2");
    }
    return options;
}

TransitionProbeOptions ParseTransitionProbeOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--probe-transition needs <graph.pmg>");
    }
    TransitionProbeOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            return argv[++index];
        };
        auto require_parameter_2d = [&](const char* name) {
            const float x = std::stof(require_value(name));
            const float y = std::stof(require_value(name));
            return pmg::ParameterVector{x, y};
        };
        if (option == "--source-param") {
            options.source_parameter = require_parameter_2d("--source-param");
        } else if (option == "--target-param") {
            options.target_parameter = require_parameter_2d("--target-param");
        } else if (option == "--frames-before") {
            options.frames_before = std::stoi(require_value("--frames-before"));
        } else if (option == "--frames-after") {
            options.frames_after = std::stoi(require_value("--frames-after"));
        } else if (option == "--output-md") {
            options.output_md = require_value("--output-md");
        } else if (option == "--output-csv") {
            options.output_csv = require_value("--output-csv");
        } else if (option == "--source-node") {
            options.source_node = require_value("--source-node");
        } else if (option == "--target-node") {
            options.target_node = require_value("--target-node");
        } else if (option == "--expect-decision") {
            options.expected_decision = require_value("--expect-decision");
        } else if (option == "--max-root-speed-ratio") {
            options.quality_gate.max_root_speed_ratio =
                std::stof(require_value("--max-root-speed-ratio"));
        } else if (option == "--max-yaw-rate-ratio") {
            options.quality_gate.max_yaw_rate_ratio =
                std::stof(require_value("--max-yaw-rate-ratio"));
        } else if (option == "--max-contact-drift") {
            options.quality_gate.max_contact_drift =
                std::stof(require_value("--max-contact-drift"));
        } else if (option == "--max-foot-height-delta") {
            options.quality_gate.max_foot_height_delta =
                std::stof(require_value("--max-foot-height-delta"));
        } else if (option == "--allow-contact-mismatch") {
            options.quality_gate.reject_contact_mismatch = false;
        } else if (option == "--reject-contact-mismatch") {
            options.quality_gate.reject_contact_mismatch = true;
        } else {
            throw std::runtime_error(
                "unknown probe-transition option '" + option + "'");
        }
    }
    if (options.source_parameter.empty() || options.target_parameter.empty()) {
        throw std::runtime_error(
            "--probe-transition requires --source-param and --target-param");
    }
    if (options.frames_before < 1 || options.frames_after < 1) {
        throw std::runtime_error(
            "--probe-transition frame counts must be positive");
    }
    if (!options.expected_decision.empty() &&
        options.expected_decision != "accept" &&
        options.expected_decision != "reject") {
        throw std::runtime_error(
            "--expect-decision must be accept or reject");
    }
    return options;
}

ContactTransitionAuditOptions ParseContactTransitionAuditOptions(
    int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error(
            "--audit-contact-transitions needs <graph.pmg>");
    }
    ContactTransitionAuditOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        if (index + 1 >= argc) {
            throw std::runtime_error(option + " requires a value");
        }
        const std::string value = argv[++index];
        if (option == "--output-csv") options.output_csv = value;
        else if (option == "--output-md") options.output_md = value;
        else if (option == "--source-node") options.source_node = value;
        else if (option == "--target-node") options.target_node = value;
        else if (option == "--samples") options.samples_per_axis = std::stoi(value);
        else throw std::runtime_error(
            "unknown audit-contact-transitions option '" + option + "'");
    }
    if (options.output_csv.empty() || options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-contact-transitions requires --output-csv and --output-md");
    }
    if (options.samples_per_axis < 2) {
        throw std::runtime_error(
            "--audit-contact-transitions: --samples must be at least 2");
    }
    return options;
}

TransitionAcceptanceConsistencyAuditOptions
ParseTransitionAcceptanceConsistencyAuditOptions(
    int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency needs <graph.pmg>");
    }
    TransitionAcceptanceConsistencyAuditOptions options;
    options.pmg_path = argv[2];
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
        } else if (option == "--source-node") {
            options.source_node = require_value("--source-node");
        } else if (option == "--target-node") {
            options.target_node = require_value("--target-node");
        } else if (option == "--samples") {
            options.samples_per_axis = std::stoi(require_value("--samples"));
        } else if (option == "--max-root-speed-ratio") {
            options.quality_gate.max_root_speed_ratio =
                std::stof(require_value("--max-root-speed-ratio"));
        } else if (option == "--max-yaw-rate-ratio") {
            options.quality_gate.max_yaw_rate_ratio =
                std::stof(require_value("--max-yaw-rate-ratio"));
        } else if (option == "--max-contact-drift") {
            options.quality_gate.max_contact_drift =
                std::stof(require_value("--max-contact-drift"));
        } else if (option == "--max-foot-height-delta") {
            options.quality_gate.max_foot_height_delta =
                std::stof(require_value("--max-foot-height-delta"));
        } else if (option == "--allow-contact-mismatch") {
            options.quality_gate.reject_contact_mismatch = false;
        } else if (option == "--reject-contact-mismatch") {
            options.quality_gate.reject_contact_mismatch = true;
        } else {
            throw std::runtime_error(
                "unknown audit-transition-acceptance-consistency option '" +
                option + "'");
        }
    }
    if (options.output_csv.empty()) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency requires --output-csv");
    }
    if (options.output_md.empty()) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency requires --output-md");
    }
    if (options.samples_per_axis < 2) {
        throw std::runtime_error(
            "--audit-transition-acceptance-consistency: --samples must be at least 2");
    }
    return options;
}

int TransitionMontageAuditCommand(
    const TransitionMontageAuditOptions& options) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(options.pmg_path);
    const TransitionMontageAuditData data =
        BuildTransitionMontageAudit(artifact, options);
    WriteTransitionMontageManifestCsv(data);
    WriteTransitionMontageMarkdown(options, data);
    std::cout << "transition_montage_rows=" << data.rows.size() << "\n";
    std::cout << "transition_montage_md=" << options.output_md << "\n";
    std::cout << "transition_montage_manifest=" << data.manifest_csv_path
              << "\n";
    std::cout << "transition_montage_conclusion=" << data.conclusion << "\n";
    return 0;
}

int ContactTransitionAuditCommand(
    const ContactTransitionAuditOptions& options) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(options.pmg_path);
    const auto rows = BuildContactTransitionRows(artifact, options);
    WriteContactTransitionAudit(options, rows);
    std::cout << "contact_transition_rows=" << rows.size() << "\n";
    std::cout << "contact_transition_conclusion="
              << ContactTransitionConclusion(rows) << "\n";
    return 0;
}

int TransitionProbeCommand(const TransitionProbeOptions& options) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(options.pmg_path);
    const TransitionProbeResult row = BuildTransitionProbe(artifact, options);
    if (!options.output_csv.empty()) WriteTransitionProbeCsv(options.output_csv, row);
    if (!options.output_md.empty()) {
        WriteTransitionProbeMarkdown(options.output_md, options, row);
    }
    const auto& q = row.quality;
    std::cout << std::setprecision(9)
              << "source_parameter=" << ParameterMd(row.source_parameter) << '\n'
              << "requested_target_parameter="
              << ParameterMd(row.requested_target_parameter) << '\n'
              << "effective_target_parameter="
              << ParameterMd(row.effective_target_parameter) << '\n'
              << "target_box_min=" << ParameterMd(row.target_box.min_corner) << '\n'
              << "target_box_max=" << ParameterMd(row.target_box.max_corner) << '\n'
              << "accepted_by_box=" << (row.accepted_by_box ? "true" : "false") << '\n'
              << "transition_metric_d=" << row.transition.distance << '\n'
              << "metric_class=" << row.metric_class << '\n'
              << "source_phase=" << row.transition.source_phase << '\n'
              << "source_frame=" << row.transition.source_frame << '\n'
              << "target_phase=" << row.transition.target_phase << '\n'
              << "target_frame=" << row.transition.target_frame << '\n'
              << "root_jump=" << row.root_jump << '\n'
              << "heading_jump=" << row.heading_jump << '\n'
              << "velocity_jump=" << row.velocity_jump << '\n'
              << "root_speed_ratio=" << q.root_speed_ratio << '\n'
              << "yaw_rate_ratio=" << q.yaw_rate_ratio << '\n'
              << "left_foot_drift=" << q.left_foot_drift << '\n'
              << "right_foot_drift=" << q.right_foot_drift << '\n'
              << "left_contact_before="
              << pmg::TransitionContactStateName(q.left_contact_before) << '\n'
              << "left_contact_after="
              << pmg::TransitionContactStateName(q.left_contact_after) << '\n'
              << "right_contact_before="
              << pmg::TransitionContactStateName(q.right_contact_before) << '\n'
              << "right_contact_after="
              << pmg::TransitionContactStateName(q.right_contact_after) << '\n'
              << "left_contact_mismatch="
              << (ContactMismatch(q.left_contact_before, q.left_contact_after)
                      ? "true" : "false") << '\n'
              << "right_contact_mismatch="
              << (ContactMismatch(q.right_contact_before, q.right_contact_after)
                      ? "true" : "false") << '\n'
              << "left_foot_height_before=" << q.left_foot_height_before << '\n'
              << "left_foot_height_after=" << q.left_foot_height_after << '\n'
              << "left_foot_height_delta=" << q.left_foot_height_delta << '\n'
              << "right_foot_height_before=" << q.right_foot_height_before << '\n'
              << "right_foot_height_after=" << q.right_foot_height_after << '\n'
              << "right_foot_height_delta=" << q.right_foot_height_delta << '\n'
              << "final_quality_gate_decision="
              << (row.final_accepted ? "accept" : "reject") << '\n'
              << "reject_reason=" << row.reject_reason << '\n';
    const std::string actual_decision = row.final_accepted ? "accept" : "reject";
    return options.expected_decision.empty() ||
                   options.expected_decision == actual_decision
               ? 0
               : 1;
}

int TransitionAcceptanceConsistencyAuditCommand(
    const TransitionAcceptanceConsistencyAuditOptions& options) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(options.pmg_path);
    const TransitionAcceptanceConsistencyAuditData data =
        BuildTransitionAcceptanceConsistencyAudit(artifact, options);
    WriteTransitionAcceptanceConsistencyCsv(options, data);
    WriteTransitionAcceptanceConsistencyMarkdown(options, data);
    std::cout << "transition_acceptance_consistency_rows="
              << data.rows.size() << "\n";
    std::cout << "transition_acceptance_consistency_csv="
              << options.output_csv << "\n";
    std::cout << "transition_acceptance_consistency_md="
              << options.output_md << "\n";
    std::cout << "transition_acceptance_consistency_conclusion="
              << data.consistency_conclusion << "\n";
    std::cout << "transition_acceptance_consistency_root_cause="
              << data.root_cause_conclusion << "\n";
    std::cout << "accepted_bad_by_box=" << data.accepted_bad_count << "\n";
    std::cout << "accepted_bad_after_quality_gate="
              << data.accepted_bad_after_quality_gate << "\n";
    std::cout << "box_accepted_quality_rejected="
              << data.box_accepted_quality_rejected << "\n";
    return data.accepted_bad_after_quality_gate == 0 ? 0 : 1;
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

SimplexAuditOptions ParseParameterResponseAuditOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error(
            "Usage: pmg_cli --audit-parameter-response graph.pmg "
            "--output-csv out.csv --output-md out.md [--samples N]");
    }
    SimplexAuditOptions options;
    options.pmg_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
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
    int node_idx = options.node_name.empty() && artifact.graph.NumNodes() == 1 ? 0 : -1;
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
    
    if (options.samples_per_axis < 2) {
        throw std::runtime_error("--samples must be at least 2");
    }
    std::ofstream csv(options.output_csv);
    if (!csv) throw std::runtime_error("parameter response audit: cannot write CSV");
    csv << "sample_type,requested_p0,requested_p1,projected_p0,projected_p1,"
           "frame_count,dx,dz,path_length,average_speed,signed_heading_change,"
           "signed_curvature,endpoint_heading,finite_valid,nearest_anchor,"
           "triangle_index,barycentric_weights,example_weights,projected\n";
    
    struct Sample { std::string type; pmg::ParameterVector parameter; };
    std::vector<Sample> samples;
    auto add = [&](const std::string& type, const pmg::ParameterVector& parameter) {
        for (const Sample& sample : samples) {
            if (sample.parameter.size() == parameter.size() &&
                std::equal(parameter.begin(), parameter.end(), sample.parameter.begin(),
                    [](float a, float b) { return std::abs(a - b) < 1.0e-5f; })) return;
        }
        samples.push_back({type, parameter});
    };
    const auto& vertices = support.Vertices();
    for (const auto& vertex : vertices) add("anchor", vertex);
    if (support.GetType() == pmg::ParameterSupport::Type::kTriangulated2D) {
        std::set<std::pair<int, int>> edges;
        for (const auto& triangle : support.Triangles()) {
            pmg::ParameterVector centroid(2, 0.0f);
            for (int vertex : triangle) {
                centroid[0] += vertices[vertex][0] / 3.0f;
                centroid[1] += vertices[vertex][1] / 3.0f;
            }
            add("triangle_centroid", centroid);
            for (int edge = 0; edge < 3; ++edge) {
                int a = triangle[edge], b = triangle[(edge + 1) % 3];
                if (a > b) std::swap(a, b);
                if (edges.insert({a, b}).second) {
                    add("edge_midpoint", {(vertices[a][0] + vertices[b][0]) * 0.5f,
                                          (vertices[a][1] + vertices[b][1]) * 0.5f});
                }
            }
        }
    }
    if (domain.Dimension() == 1) {
        for (int i = 0; i < options.samples_per_axis; ++i) {
            float t = i / std::max(1.0f, float(options.samples_per_axis - 1));
            pmg::ParameterVector pt = {domain.Bounds().min_corner[0] + t * (domain.Bounds().max_corner[0] - domain.Bounds().min_corner[0])};
            if (support.Contains(pt)) add("grid", pt);
        }
    } else if (domain.Dimension() == 2) {
        for (int i = 0; i < options.samples_per_axis; ++i) {
            float ty = i / std::max(1.0f, float(options.samples_per_axis - 1));
            float y = domain.Bounds().min_corner[1] + ty * (domain.Bounds().max_corner[1] - domain.Bounds().min_corner[1]);
            for (int j = 0; j < options.samples_per_axis; ++j) {
                float tx = j / std::max(1.0f, float(options.samples_per_axis - 1));
                float x = domain.Bounds().min_corner[0] + tx * (domain.Bounds().max_corner[0] - domain.Bounds().min_corner[0]);
                pmg::ParameterVector pt = {x, y};
                if (support.Contains(pt)) add("grid", pt);
            }
        }
    } else {
        throw std::runtime_error("Only 1D and 2D spaces supported");
    }
    if (domain.Dimension() == 2) add("outside_request", {0.8f, 0.8f});

    auto list = [](const std::vector<float>& values) {
        std::ostringstream out;
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i) out << ';';
            out << values[i];
        }
        return out.str();
    };
    bool all_valid = true;
    for (const Sample& sample : samples) {
        const auto& p = sample.parameter;
        const bool projected = !support.Contains(p);
        const pmg::ParameterVector sp = support.Project(p);
        const pmg::MotionClip clip = space.GenerateClip(sp, artifact.metadata.frames_per_second);
        clip.RequireNotEmpty("parameter response audit");
        const pmg::Vec3 start = clip.frames.front().root_position;
        const pmg::Vec3 end = clip.frames.back().root_position;
        float path_length = 0.0f;
        for (std::size_t frame = 1; frame < clip.frames.size(); ++frame) {
            const pmg::Vec3 step = clip.frames[frame].root_position - clip.frames[frame - 1].root_position;
            path_length += std::hypot(step.x, step.z);
        }
        const float duration = clip.DurationSeconds();
        const float heading = pmg::MeasureParameterMetric(pmg::ParameterMetric::kTurnRate, clip) * duration;
        const float endpoint_heading = std::atan2(
            pmg::Rotate(clip.frames.back().local_rotations.front(), {0, 0, 1}).x,
            pmg::Rotate(clip.frames.back().local_rotations.front(), {0, 0, 1}).z);
        const float curvature = path_length > 1.0e-4f ? heading / path_length : 0.0f;
        bool finite = std::isfinite(path_length) && std::isfinite(heading) &&
                      std::isfinite(end.x) && std::isfinite(end.z);
        all_valid = all_valid && finite;
        int nearest = 0;
        float nearest_distance = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            float distance = 0.0f;
            for (std::size_t axis = 0; axis < sp.size(); ++axis) distance += std::pow(sp[axis] - vertices[i][axis], 2.0f);
            if (distance < nearest_distance) { nearest_distance = distance; nearest = static_cast<int>(i); }
        }
        const auto barycentric = support.BarycentricWeights(sp);
        int triangle_index = -1;
        for (std::size_t i = 0; i < support.Triangles().size(); ++i) {
            const auto& triangle = support.Triangles()[i];
            if (barycentric[triangle[0]] >= -1.0e-5f && barycentric[triangle[1]] >= -1.0e-5f && barycentric[triangle[2]] >= -1.0e-5f) { triangle_index = static_cast<int>(i); break; }
        }
        csv << sample.type << ',' << p[0] << ',' << (p.size() > 1 ? p[1] : 0) << ','
            << sp[0] << ',' << (sp.size() > 1 ? sp[1] : 0) << ',' << clip.NumFrames() << ','
            << end.x - start.x << ',' << end.z - start.z << ',' << path_length << ','
            << (duration > 0 ? path_length / duration : 0) << ',' << heading << ',' << curvature << ','
            << endpoint_heading << ',' << (finite ? "true" : "false") << ",anchor_" << nearest << ','
            << triangle_index << ",\"" << list(barycentric) << "\",\""
            << list(space.ComputeLocalBlendWeights(sp)) << "\"," << (projected ? "true" : "false") << '\n';
    }

    std::ofstream md(options.output_md);
    if (!md) throw std::runtime_error("parameter response audit: cannot write markdown");
    md << "# Parameter Response Audit: " << artifact.graph.Node(node_idx).name << "\n\n"
       << "## Contract\n\nRoot coordinates use artifact units in the x/z floor plane; heading uses radians; "
          "signed curvature is heading change / path length. Samples include anchors, triangle centroids, "
          "unique edge midpoints, support grid points, and outside request (0.8, 0.8).\n\n"
       << "## Result\n\n- Samples: " << samples.size() << "\n- Finite/valid: " << (all_valid ? "yes" : "no")
       << "\n- Conclusion: " << (all_valid ? "PASS_PARAMETER_RESPONSE" : "FAIL_INTERPOLATION_DISCONTINUITY")
       << "\n\nDetailed measurements: `" << options.output_csv << "`.\n";
    std::cout << "parameter_response_rows=" << samples.size() << "\n"
              << "parameter_response_csv=" << options.output_csv << "\n"
              << "parameter_response_md=" << options.output_md << "\n";
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
    if (command == "--audit-parameter-response") {
        return SimplexAuditCommand(ParseParameterResponseAuditOptions(argc, argv));
    }
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
    if (command == "--audit-registration-phase-alignment" && argc >= 3) {
        return RegistrationPhaseAlignmentAuditCommand(
            ParseRegistrationPhaseAlignmentAuditOptions(argc, argv));
    }
    if (command == "--audit-reachable-region" && argc >= 3) {
        return ReachableRegionAuditCommand(
            ParseReachableRegionAuditOptions(argc, argv));
    }
    if (command == "--audit-transition-montage" && argc >= 3) {
        return TransitionMontageAuditCommand(
            ParseTransitionMontageAuditOptions(argc, argv));
    }
    if (command == "--probe-transition" && argc >= 3) {
        return TransitionProbeCommand(ParseTransitionProbeOptions(argc, argv));
    }
    if (command == "--audit-contact-transitions" && argc >= 3) {
        return ContactTransitionAuditCommand(
            ParseContactTransitionAuditOptions(argc, argv));
    }
    if (command == "--audit-transition-acceptance-consistency" && argc >= 3) {
        return TransitionAcceptanceConsistencyAuditCommand(
            ParseTransitionAcceptanceConsistencyAuditOptions(argc, argv));
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
