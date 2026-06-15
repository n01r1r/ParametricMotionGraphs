#include "PmgCommandModules.h"

#include "pmg/AlignmentStrategy.h"
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
#include "pmg/legacy/FrameCountClipGeneration.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
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
    return std::nullopt;
}

}  // namespace pmgcli
