#include "PmgCommandModules.h"

#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/FootLocking.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/GraphIo.h"
#include "pmg/GraphSpec.h"
#include "pmg/MotionDistance.h"
#include "pmg/MotionRegistration.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/RuntimeController.h"
#include "pmg/SkeletonCompatibility.h"

#include <algorithm>
#include <cctype>
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


float Percentile(std::vector<float> values, float p) {
    if (values.empty()) {
        return 0.0f;
    }
    std::sort(values.begin(), values.end());
    const float position = p * static_cast<float>(values.size() - 1);
    const std::size_t low = static_cast<std::size_t>(std::floor(position));
    const std::size_t high = static_cast<std::size_t>(std::ceil(position));
    const float fraction = position - static_cast<float>(low);
    return values[low] * (1.0f - fraction) + values[high] * fraction;
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

// ---------------------------------------------------------------------------
// Phase 4: paper applications, validated numerically.
//   --random-walk  random transitions through the PMG; popping metric
//   --goto         goal-directed locomotion via semantic control
//                  (target position -> desired heading -> curvature parameter)
// ---------------------------------------------------------------------------

// Load a complete artifact (V4+), or build the same artifact in-memory from a
// GraphSpec for compatibility with existing commands and tests.
pmg::BuiltPmgArtifact LoadOrBuildRuntimeArtifact(
    const std::string& input_path,
    const std::string& cycle_joint,
    const std::string& contact_joints_csv,
    int min_contact_frames,
    const pmg::PmgBuilderConfig& builder_config) {
    if (std::filesystem::path(input_path).extension() == ".pmg") {
        pmg::BuiltPmgArtifact artifact =
            pmg::LoadPmgArtifactText(input_path);
        if (artifact.skeleton.NumJoints() == 0) {
            throw std::runtime_error(
                "runtime requires a complete (V4+) artifact containing its Skeleton");
        }
        if (artifact.metadata.frames_per_second <= 0.0f) {
            throw std::runtime_error(
                "runtime artifact has invalid frame metadata");
        }
        return artifact;
    }

    pmg::ArtifactBuildConfig config;
    config.default_edge_config = builder_config;
    config.default_cycle_joint = cycle_joint;
    config.default_contact_joints = SplitCommaList(contact_joints_csv);
    config.default_min_contact_frames = min_contact_frames;
    config.default_dtw_refine = true;
    return pmg::BuildPmgArtifactFromSpec(
        pmg::LoadGraphSpec(input_path), config);
}

struct PopStats {
    float median_step = 0.0f;
    float max_step = 0.0f;
    float Ratio() const {
        return median_step > 1.0e-6f ? max_step / median_step : 0.0f;
    }
};

PopStats MeasurePopping(const pmg::Skeleton& skeleton, const std::vector<pmg::Pose>& poses) {
    PopStats stats;
    std::vector<float> deltas;
    deltas.reserve(poses.size());
    for (std::size_t frame = 1; frame < poses.size(); ++frame) {
        deltas.push_back(MeanJointDistance(skeleton, poses[frame - 1], poses[frame]));
    }
    if (deltas.empty()) {
        return stats;
    }
    stats.max_step = *std::max_element(deltas.begin(), deltas.end());
    stats.median_step = Percentile(deltas, 0.5f);
    return stats;
}

struct RandomWalkOptions {
    std::string spec_path;
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    std::string cycle_joint;
    int min_contact_frames = 3;
    pmg::PmgBuilderConfig builder;
    float seconds = 30.0f;
    unsigned int seed = 99u;
    // Hold one constant parameter instead of randomizing: calibration mode
    // that reports the achieved world turn rate at that parameter.
    float hold_parameter = std::numeric_limits<float>::quiet_NaN();
    // Assert thresholds; negative = report only.
    int min_transitions = -1;
    float max_pop_ratio = -1.0f;
    // Override runtime blend placement (default: artifact-derived / centered).
    std::optional<pmg::TransitionWindowConvention> blend_placement;
    // Ablation: override the target pre-roll policy (default: clamp at clip
    // start). kWrapCyclicClip lets a cyclic self-edge pre-roll into the previous
    // cycle tail instead of clamping at phase 0.
    std::optional<pmg::TransitionPreRollPolicy> preroll_policy;
    // Per-frame motion trace for offline plotting (path, pop-over-time,
    // before/after). Empty = no dump.
    std::string dump_motion_csv;
};

// Paper application A: stream random transitions through the graph and verify
// no frame-to-frame popping (worst step vs the median walking step).
int RandomWalkCommand(const RandomWalkOptions& options) {
    const pmg::BuiltPmgArtifact artifact = LoadOrBuildRuntimeArtifact(
        options.spec_path, options.cycle_joint, options.contact_joints_csv,
        options.min_contact_frames, options.builder);

    // The metric convention lives in the artifact; the runtime blend placement
    // defaults to PMG centered (§5.2.1) and can be overridden for the A/B.
    pmg::RuntimeControllerConfig runtime_config =
        pmg::RuntimeControllerConfigFromArtifact(artifact);
    if (options.blend_placement.has_value()) {
        runtime_config.convention = *options.blend_placement;
    }
    if (options.preroll_policy.has_value()) {
        runtime_config.preroll_policy = *options.preroll_policy;
    }
    std::cout << "preroll_policy="
              << (runtime_config.preroll_policy ==
                          pmg::TransitionPreRollPolicy::kWrapCyclicClip
                      ? "wrap"
                      : "clamp")
              << "\n";
    std::cout << "metric_convention="
              << pmg::TransitionWindowConventionName(
                     artifact.metadata.edge_builds.empty()
                         ? pmg::TransitionWindowConvention::kKovarDirectional
                         : artifact.metadata.edge_builds.front()
                               .config.transition_convention)
              << " blend_placement="
              << pmg::TransitionWindowConventionName(runtime_config.convention)
              << "\n";
    pmg::PointCloudAlignment alignment(
        artifact.skeleton, runtime_config.transition_blend_frames);
    pmg::RuntimeController controller(
        artifact.graph, alignment, runtime_config);

    std::mt19937 rng(options.seed);
    const bool hold = !std::isnan(options.hold_parameter);

    const pmg::ParameterDomain start_domain =
        artifact.graph.Node(0).motion_space.Domain();
    controller.Start(0,
                     hold ? pmg::ParameterVector{options.hold_parameter}
                          : start_domain.SampleUniform(rng),
                     artifact.metadata.frames_per_second);

    pmg::RuntimeControlRequest request;
    if (hold) {
        request.desired_node = 0;
        request.desired_parameter = {options.hold_parameter};
    } else {
        request = pmg::ChooseRandomOutgoingTransition(
                      artifact.graph, controller.CurrentNode(), rng)
                      .request;
    }

    const float dt = 1.0f / artifact.metadata.frames_per_second;
    const int total_frames = static_cast<int>(
        options.seconds * artifact.metadata.frames_per_second);
    int last_transition_count = 0;

    std::vector<pmg::Pose> poses;
    poses.reserve(static_cast<std::size_t>(total_frames));
    poses.push_back(controller.CurrentPose());

    // Per-frame trace for --dump-motion-csv.
    struct MotionRow {
        float time_seconds;
        float root_x;
        float root_z;
        float facing_degrees;
        float step;
        int transitioning;
        int node;
    };
    const bool dump_motion = !options.dump_motion_csv.empty();
    std::vector<MotionRow> motion_rows;
    constexpr float kRadToDeg = 57.2957795f;
    if (dump_motion) {
        motion_rows.reserve(static_cast<std::size_t>(total_frames + 1));
        const pmg::Pose& first = poses.front();
        motion_rows.push_back({0.0f, first.root_position.x, first.root_position.z,
                               pmg::PoseFacingYaw(first) * kRadToDeg, 0.0f,
                               controller.IsTransitioning() ? 1 : 0,
                               controller.CurrentNode()});
    }

    for (int frame = 0; frame < total_frames; ++frame) {
        controller.Update(dt, request);
        poses.push_back(controller.CurrentPose());

        if (dump_motion) {
            const pmg::Pose& cur = poses.back();
            motion_rows.push_back(
                {static_cast<float>(frame + 1) * dt, cur.root_position.x,
                 cur.root_position.z, pmg::PoseFacingYaw(cur) * kRadToDeg,
                 MeanJointDistance(artifact.skeleton,
                                   poses[poses.size() - 2], cur),
                 controller.IsTransitioning() ? 1 : 0,
                 controller.CurrentNode()});
        }

        if (!hold && controller.CompletedTransitions() != last_transition_count) {
            last_transition_count = controller.CompletedTransitions();
            request = pmg::ChooseRandomOutgoingTransition(
                          artifact.graph, controller.CurrentNode(), rng)
                          .request;
        }
    }

    if (dump_motion) {
        std::ofstream csv(options.dump_motion_csv);
        if (!csv) {
            throw std::runtime_error(
                "--dump-motion-csv: cannot open '" + options.dump_motion_csv +
                "'");
        }
        csv << "frame,time_seconds,root_x,root_z,facing_degrees,step,"
               "transitioning,node\n";
        for (std::size_t i = 0; i < motion_rows.size(); ++i) {
            const MotionRow& r = motion_rows[i];
            csv << i << ',' << r.time_seconds << ',' << r.root_x << ','
                << r.root_z << ',' << r.facing_degrees << ',' << r.step << ','
                << r.transitioning << ',' << r.node << '\n';
        }
        std::cout << "motion_csv=" << options.dump_motion_csv << " rows="
                  << motion_rows.size() << "\n";
    }

    if (hold) {
        // Achieved world turn rate: unwrapped heading change over the run.
        float unwrapped_heading = 0.0f;
        float previous_heading = pmg::PoseFacingYaw(poses.front());
        for (std::size_t frame = 1; frame < poses.size(); ++frame) {
            const float heading = pmg::PoseFacingYaw(poses[frame]);
            unwrapped_heading +=
                pmg::WrapAngleRadians(heading - previous_heading);
            previous_heading = heading;
        }
        const float elapsed = static_cast<float>(poses.size() - 1) /
                              artifact.metadata.frames_per_second;
        std::cout << "hold_parameter=" << options.hold_parameter << "\n";
        std::cout << "achieved_turn_rate=" << unwrapped_heading / elapsed << " rad/s\n";
    }

    const PopStats stats = MeasurePopping(artifact.skeleton, poses);
    std::cout << "frames=" << poses.size() << "\n";
    std::cout << "transitions=" << controller.CompletedTransitions() << "\n";
    std::cout << "median_step=" << stats.median_step << "\n";
    std::cout << "max_step=" << stats.max_step << "\n";
    std::cout << "pop_ratio=" << stats.Ratio() << "\n";

    bool failed = false;
    if (options.min_transitions >= 0 &&
        controller.CompletedTransitions() < options.min_transitions) {
        std::cout << "ASSERT FAIL: transitions=" << controller.CompletedTransitions()
                  << " < " << options.min_transitions << "\n";
        failed = true;
    }
    if (options.max_pop_ratio >= 0.0f && stats.Ratio() > options.max_pop_ratio) {
        std::cout << "ASSERT FAIL: pop_ratio=" << stats.Ratio() << " > "
                  << options.max_pop_ratio << "\n";
        failed = true;
    }
    std::cout << (failed ? "RESULT=FAIL" : "RESULT=PASS") << "\n";
    return failed ? 1 : 0;
}

struct GotoOptions {
    std::string spec_path;
    float target_x = 0.0f;
    float target_z = 0.0f;
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    std::string cycle_joint;
    int min_contact_frames = 3;
    pmg::PmgBuilderConfig builder;
    float seconds = 60.0f;
    bool trace = false;
    std::optional<float> final_facing_degrees;
    float facing_tolerance_degrees = 15.0f;
    // Assert thresholds; negative = report only.
    float tolerance = -1.0f;
    float max_pop_ratio = -1.0f;
};

// Paper application B/C: goal-directed locomotion through semantic control.
// The user-level intent is a target position; it is converted to a desired
// heading, the heading error to a turn rate, and the turn rate to the walk
// space's curvature parameter via the examples' measured turn rates. The
// raw parameter never appears in the interface.
int GotoCommand(const GotoOptions& options) {
    const pmg::BuiltPmgArtifact artifact = LoadOrBuildRuntimeArtifact(
        options.spec_path, options.cycle_joint, options.contact_joints_csv,
        options.min_contact_frames, options.builder);

    const pmg::ParametricMotionSpace& walk_space =
        artifact.graph.Node(0).motion_space;
    const std::vector<float> parameter_min = walk_space.MinParameter();
    const std::vector<float> parameter_max = walk_space.MaxParameter();
    pmg::ParameterVector start_parameter(parameter_min.size(), 0.0f);
    for (std::size_t axis = 0; axis < start_parameter.size(); ++axis) {
        start_parameter[axis] =
            0.5f * (parameter_min[axis] + parameter_max[axis]);
    }
    const pmg::RuntimeControllerConfig runtime_config =
        pmg::RuntimeControllerConfigFromArtifact(artifact);
    pmg::GoalDirectedLocomotionConfig steering_config;
    steering_config.runtime = runtime_config;
    pmg::GoalDirectedLocomotion steering(
        artifact.graph, artifact.skeleton, 0,
        artifact.metadata.frames_per_second, steering_config);
    const pmg::SteeringCalibration& calibration = steering.Calibration();
    for (std::size_t axis = 0; axis < calibration.axes.size(); ++axis) {
        const pmg::SteeringAxis& axis_calibration = calibration.axes[axis];
        const char* metric_name =
            axis_calibration.metric == pmg::ParameterMetric::kTurnRate
                ? "turn_rate"
                : axis_calibration.metric == pmg::ParameterMetric::kTravelSpeed
                      ? "travel_speed"
                      : "none";
        for (std::size_t sample = 0;
             sample < axis_calibration.parameters.size(); ++sample) {
            std::cout << "achieved[axis=" << axis << " " << metric_name
                      << " param=" << axis_calibration.parameters[sample]
                      << "]=" << axis_calibration.achieved_metric[sample]
                      << "\n";
        }
    }
    std::cout << "travel_heading_offset_deg="
              << calibration.travel_heading_offset * 180.0f / pmg::kPi
              << "\n";

    pmg::PointCloudAlignment alignment(
        artifact.skeleton, runtime_config.transition_blend_frames);
    pmg::RuntimeController controller(
        artifact.graph, alignment, runtime_config);
    controller.Start(0, start_parameter,
                     artifact.metadata.frames_per_second);

    const float dt = 1.0f / artifact.metadata.frames_per_second;
    const int total_frames = static_cast<int>(
        options.seconds * artifact.metadata.frames_per_second);

    std::vector<pmg::Pose> poses;
    poses.reserve(static_cast<std::size_t>(total_frames));
    float min_distance = std::numeric_limits<float>::max();
    float reached_at_seconds = -1.0f;
    pmg::GoalRequest goal;
    goal.target_position = {options.target_x, 0.0f, options.target_z};
    if (options.final_facing_degrees.has_value()) {
        goal.final_facing_yaw =
            *options.final_facing_degrees * pmg::kPi / 180.0f;
    }

    for (int frame = 0; frame < total_frames; ++frame) {
        const pmg::Pose pose = controller.CurrentPose();
        poses.push_back(pose);

        const float dx = options.target_x - pose.root_position.x;
        const float dz = options.target_z - pose.root_position.z;
        const float distance = std::sqrt(dx * dx + dz * dz);
        min_distance = std::min(min_distance, distance);
        if (options.tolerance >= 0.0f &&
            steering.Reached(
                pose, goal, options.tolerance,
                options.facing_tolerance_degrees * pmg::kPi / 180.0f)) {
            reached_at_seconds = static_cast<float>(frame) * dt;
            break;
        }

        const pmg::RuntimeControlRequest request =
            steering.RequestForPose(pose, goal);

        if (options.trace && frame % 30 == 0) {
            std::cout << "t=" << static_cast<float>(frame) * dt
                      << " pos=(" << pose.root_position.x << ", " << pose.root_position.z
                      << ") dist=" << distance << " param=(";
            for (std::size_t axis = 0;
                 axis < request.desired_parameter.size(); ++axis) {
                std::cout << (axis == 0 ? "" : ", ")
                          << request.desired_parameter[axis];
            }
            std::cout << ")\n";
        }

        controller.Update(dt, request);
    }

    const PopStats stats = MeasurePopping(artifact.skeleton, poses);
    std::cout << "target=(" << options.target_x << ", " << options.target_z << ")\n";
    std::cout << "min_distance=" << min_distance << "\n";
    std::cout << "reached=" << (reached_at_seconds >= 0.0f ? 1 : 0) << "\n";
    if (reached_at_seconds >= 0.0f) {
        std::cout << "reached_at_seconds=" << reached_at_seconds << "\n";
    }
    std::cout << "transitions=" << controller.CompletedTransitions() << "\n";
    std::cout << "median_step=" << stats.median_step << "\n";
    std::cout << "max_step=" << stats.max_step << "\n";
    std::cout << "pop_ratio=" << stats.Ratio() << "\n";

    bool failed = false;
    if (options.tolerance >= 0.0f && reached_at_seconds < 0.0f) {
        std::cout << "ASSERT FAIL: target not reached within tolerance "
                  << options.tolerance << " (min_distance=" << min_distance << ")\n";
        failed = true;
    }
    if (options.max_pop_ratio >= 0.0f && stats.Ratio() > options.max_pop_ratio) {
        std::cout << "ASSERT FAIL: pop_ratio=" << stats.Ratio() << " > "
                  << options.max_pop_ratio << "\n";
        failed = true;
    }
    std::cout << (failed ? "RESULT=FAIL" : "RESULT=PASS") << "\n";
    return failed ? 1 : 0;
}

void ParseSharedRuntimeOption(
    const std::string& option,
    const std::function<std::string(const char*)>& require_value,
    std::string& contact_joints_csv,
    std::string& cycle_joint,
    int& min_contact_frames,
    pmg::PmgBuilderConfig& builder,
    bool& handled) {
    handled = true;
    if (option == "--contact-joints") {
        contact_joints_csv = require_value("--contact-joints");
    } else if (option == "--cycle-joint") {
        cycle_joint = require_value("--cycle-joint");
    } else if (option == "--min-contact-frames") {
        min_contact_frames = std::stoi(require_value("--min-contact-frames"));
    } else if (option == "--tgood") {
        builder.good_transition_threshold = std::stof(require_value("--tgood"));
    } else if (option == "--tbad") {
        builder.bad_transition_threshold = std::stof(require_value("--tbad"));
    } else if (option == "--source-samples") {
        builder.source_sample_count = std::stoi(require_value("--source-samples"));
    } else if (option == "--target-samples") {
        builder.target_sample_count = std::stoi(require_value("--target-samples"));
    } else if (option == "--seed") {
        builder.seed = static_cast<unsigned int>(std::stoul(require_value("--seed")));
    } else {
        handled = false;
    }
}

RandomWalkOptions ParseRandomWalkOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--random-walk needs <spec>");
    }
    RandomWalkOptions options;
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
        bool handled = false;
        ParseSharedRuntimeOption(option, require_value, options.contact_joints_csv,
                                 options.cycle_joint, options.min_contact_frames,
                                 options.builder, handled);
        if (handled) {
            continue;
        }
        if (option == "--seconds") {
            options.seconds = std::stof(require_value("--seconds"));
        } else if (option == "--walk-seed") {
            options.seed = static_cast<unsigned int>(std::stoul(require_value("--walk-seed")));
        } else if (option == "--hold-param") {
            options.hold_parameter = std::stof(require_value("--hold-param"));
        } else if (option == "--min-transitions") {
            options.min_transitions = std::stoi(require_value("--min-transitions"));
        } else if (option == "--max-pop-ratio") {
            options.max_pop_ratio = std::stof(require_value("--max-pop-ratio"));
        } else if (option == "--transition-convention") {
            const std::string value =
                require_value("--transition-convention");
            if (value == "centered" || value == "pmg_centered") {
                options.builder.transition_convention =
                    pmg::TransitionWindowConvention::kPmgCentered;
            } else if (value == "directional" ||
                       value == "kovar_directional") {
                options.builder.transition_convention =
                    pmg::TransitionWindowConvention::kKovarDirectional;
            } else {
                throw std::runtime_error(
                    "--transition-convention expects 'directional' or "
                    "'centered'");
            }
        } else if (option == "--blend-placement") {
            const std::string value = require_value("--blend-placement");
            if (value == "centered" || value == "pmg_centered") {
                options.blend_placement =
                    pmg::TransitionWindowConvention::kPmgCentered;
            } else if (value == "directional" ||
                       value == "kovar_directional") {
                options.blend_placement =
                    pmg::TransitionWindowConvention::kKovarDirectional;
            } else {
                throw std::runtime_error(
                    "--blend-placement expects 'directional' or 'centered'");
            }
        } else if (option == "--preroll-policy") {
            const std::string value = require_value("--preroll-policy");
            if (value == "clamp") {
                options.preroll_policy =
                    pmg::TransitionPreRollPolicy::kClampAtClipStart;
            } else if (value == "wrap") {
                options.preroll_policy =
                    pmg::TransitionPreRollPolicy::kWrapCyclicClip;
            } else {
                throw std::runtime_error(
                    "--preroll-policy expects 'clamp' or 'wrap'");
            }
        } else if (option == "--dump-motion-csv") {
            options.dump_motion_csv = require_value("--dump-motion-csv");
        } else {
            throw std::runtime_error("unknown random-walk option '" + option + "'");
        }
    }
    return options;
}

GotoOptions ParseGotoOptions(int argc, char** argv) {
    if (argc < 5) {
        throw std::runtime_error("--goto needs <spec> <x> <z>");
    }
    GotoOptions options;
    options.spec_path = argv[2];
    options.target_x = std::stof(argv[3]);
    options.target_z = std::stof(argv[4]);
    for (int index = 5; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        bool handled = false;
        ParseSharedRuntimeOption(option, require_value, options.contact_joints_csv,
                                 options.cycle_joint, options.min_contact_frames,
                                 options.builder, handled);
        if (handled) {
            continue;
        }
        if (option == "--seconds") {
            options.seconds = std::stof(require_value("--seconds"));
        } else if (option == "--tolerance") {
            options.tolerance = std::stof(require_value("--tolerance"));
        } else if (option == "--max-pop-ratio") {
            options.max_pop_ratio = std::stof(require_value("--max-pop-ratio"));
        } else if (option == "--facing-degrees") {
            options.final_facing_degrees =
                std::stof(require_value("--facing-degrees"));
        } else if (option == "--facing-tolerance-degrees") {
            options.facing_tolerance_degrees =
                std::stof(require_value("--facing-tolerance-degrees"));
        } else if (option == "--trace") {
            options.trace = true;
        } else {
            throw std::runtime_error("unknown goto option '" + option + "'");
        }
    }
    return options;
}

}  // namespace

namespace pmgcli {

std::optional<int> TryRunRuntimeCommand(int argc, char** argv) {
    const std::string command = argc > 1 ? argv[1] : "";
    if (command == "--random-walk" && argc >= 3) {
        return RandomWalkCommand(ParseRandomWalkOptions(argc, argv));
    }
    if (command == "--goto" && argc >= 5) {
        return GotoCommand(ParseGotoOptions(argc, argv));
    }
    return std::nullopt;
}

}  // namespace pmgcli
