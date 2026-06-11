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

struct GraphSpecBuildInputs {
    pmg::Skeleton skeleton;
    std::map<std::string, pmg::ParametricMotionSpace> spaces;
};

void PrintParameterVector(const pmg::ParameterVector& parameter) {
    std::cout << "[";
    for (std::size_t i = 0; i < parameter.size(); ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << parameter[i];
    }
    std::cout << "]";
}

void PrintAabb(const pmg::ParameterAabb& box) {
    if (box.IsEmpty()) {
        std::cout << "<empty>";
        return;
    }
    std::cout << "min=";
    PrintParameterVector(box.min_corner);
    std::cout << " max=";
    PrintParameterVector(box.max_corner);
}

const pmg::GraphSpecNode& FindSpecNodeForCli(
    const pmg::GraphSpec& spec,
    const std::string& name) {
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        if (node.name == name) {
            return node;
        }
    }
    throw std::runtime_error("GraphSpec CLI: unknown node '" + name + "'");
}

GraphSpecBuildInputs LoadSpecInputsForCli(
    const pmg::GraphSpec& spec,
    float skeleton_offset_tolerance = 1.0e-4f) {
    if (spec.nodes.empty()) {
        throw std::runtime_error("GraphSpec CLI: spec contains no nodes");
    }

    GraphSpecBuildInputs inputs;
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        inputs.spaces.emplace(node.name,
                              pmg::ParametricMotionSpace(node.name, node.parameter_dimension));
    }

    std::optional<pmg::Skeleton> reference_skeleton;
    for (const pmg::GraphSpecExample& example : spec.examples) {
        const auto space_it = inputs.spaces.find(example.node_name);
        if (space_it == inputs.spaces.end()) {
            throw std::runtime_error("GraphSpec CLI: example references unknown node '" +
                                     example.node_name + "'");
        }

        const pmg::BvhData data = pmg::BvhLoader::Load(example.bvh_path);
        if (!reference_skeleton.has_value()) {
            reference_skeleton = data.skeleton;
        } else {
            pmg::RequireSkeletonCompatible(*reference_skeleton, data.skeleton,
                                           "GraphSpec CLI", skeleton_offset_tolerance);
        }
        space_it->second.AddExample(example.parameter, data.clip);
    }

    if (!reference_skeleton.has_value()) {
        throw std::runtime_error("GraphSpec CLI: spec contains no examples");
    }
    inputs.skeleton = *reference_skeleton;
    return inputs;
}

int ValidateGraphSpecCommand(const std::string& spec_path) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(spec_path);
    std::cout << "graph spec: " << spec_path << "\n";
    std::cout << "nodes: " << spec.nodes.size() << "\n";
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        int example_count = 0;
        for (const pmg::GraphSpecExample& example : spec.examples) {
            if (example.node_name == node.name) {
                ++example_count;
            }
        }
        std::cout << "  " << node.name << " dim=" << node.parameter_dimension
                  << " examples=" << example_count << "\n";
    }

    const GraphSpecBuildInputs inputs = LoadSpecInputsForCli(spec);
    std::cout << "skeleton: joints=" << inputs.skeleton.NumJoints()
              << " compatible=yes\n";
    for (const auto& item : inputs.spaces) {
        const pmg::ParametricMotionSpace& space = item.second;
        std::cout << "space " << item.first
                  << ": dim=" << space.ParameterDimension()
                  << " examples=" << space.NumExamples();
        if (space.NumExamples() > 0) {
            std::cout << " min=";
            PrintParameterVector(space.MinParameter());
            std::cout << " max=";
            PrintParameterVector(space.MaxParameter());
        }
        std::cout << "\n";
    }
    std::cout << "edges requested: " << spec.edges.size() << "\n";
    for (const pmg::GraphSpecEdge& edge : spec.edges) {
        (void)FindSpecNodeForCli(spec, edge.source_node);
        (void)FindSpecNodeForCli(spec, edge.target_node);
        std::cout << "  " << edge.source_node << " -> " << edge.target_node << "\n";
    }
    return 0;
}

void PrintEdgeBuildReport(const pmg::EdgeBuildReport& report) {
    std::cout << "edge_created=" << (report.edge_created ? "yes" : "no") << "\n";
    if (!report.edge_created) {
        std::cout << "reject_reason=" << report.reject_reason << "\n";
    }
    std::cout << "source sample reports: " << report.source_reports.size() << "\n";
    for (std::size_t i = 0; i < report.source_reports.size(); ++i) {
        const pmg::SourceSampleBuildReport& source = report.source_reports[i];
        std::cout << "  [" << i << "] source=";
        PrintParameterVector(source.source_parameter);
        std::cout << " accepted=" << (source.accepted ? "yes" : "no") << "\n";
        std::cout << "      D min=" << source.min_distance
                  << " p25=" << source.p25_distance
                  << " median=" << source.median_distance
                  << " max=" << source.max_distance << "\n";
        std::cout << "      counts GOOD=" << source.good_count
                  << " NEUTRAL=" << source.neutral_count
                  << " BAD=" << source.bad_count << "\n";
        std::cout << "      box before: ";
        PrintAabb(source.target_box_before_shrink);
        std::cout << "\n      box after : ";
        PrintAabb(source.target_box_after_shrink);
        std::cout << "\n";
        if (!source.accepted) {
            std::cout << "      reject_reason=" << source.reject_reason << "\n";
        }
    }
}

int DiagnoseGraphEdgeCommand(
    const std::string& spec_path,
    const std::string& source_node,
    const std::string& target_node,
    const pmg::PmgBuilderConfig& config) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(spec_path);
    const GraphSpecBuildInputs inputs = LoadSpecInputsForCli(spec);

    const auto source_it = inputs.spaces.find(source_node);
    const auto target_it = inputs.spaces.find(target_node);
    if (source_it == inputs.spaces.end()) {
        throw std::runtime_error("--diagnose-graph-edge: unknown source node '" + source_node + "'");
    }
    if (target_it == inputs.spaces.end()) {
        throw std::runtime_error("--diagnose-graph-edge: unknown target node '" + target_node + "'");
    }

    std::cout << "diagnose edge: " << source_node << " -> " << target_node << "\n";
    std::cout << "config: TGOOD=" << config.good_transition_threshold
              << " TBAD=" << config.bad_transition_threshold
              << " source_samples=" << config.source_sample_count
              << " target_samples=" << config.target_sample_count << "\n";

    const pmg::EdgeBuildResult result = pmg::PmgBuilder::BuildEdgeWithReport(
        inputs.skeleton, 0, 1, source_it->second, target_it->second, config);
    std::cout << "transition_samples=" << result.edge.samples.size() << "\n";
    PrintEdgeBuildReport(result.report);
    return result.edge.samples.empty() ? 1 : 0;
}

pmg::PmgBuilderConfig ParseBuilderConfigOptions(int argc, char** argv, int first_option_index) {
    pmg::PmgBuilderConfig config;
    for (int index = first_option_index; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--tgood") {
            config.good_transition_threshold = std::stof(require_value("--tgood"));
        } else if (option == "--tbad") {
            config.bad_transition_threshold = std::stof(require_value("--tbad"));
        } else if (option == "--source-samples") {
            config.source_sample_count = std::stoi(require_value("--source-samples"));
        } else if (option == "--target-samples") {
            config.target_sample_count = std::stoi(require_value("--target-samples"));
        } else if (option == "--seed") {
            config.seed = static_cast<unsigned int>(std::stoul(require_value("--seed")));
        } else {
            throw std::runtime_error("unknown builder option '" + option + "'");
        }
    }
    return config;
}

std::string JsonEscape(const std::string& text) {
    std::string escaped;
    for (const char character : text) {
        if (character == '\\' || character == '"') {
            escaped.push_back('\\');
        }
        escaped.push_back(character);
    }
    return escaped;
}

bool BoxesDiffer(const pmg::ParameterAabb& before,
                 const pmg::ParameterAabb& after) {
    return before.min_corner != after.min_corner ||
           before.max_corner != after.max_corner;
}

struct ArtifactRuntimeMetrics {
    int simulated_frames = 0;
    int transitions = 0;
    double runtime_frames_per_second = 0.0;
    float target_min_distance = std::numeric_limits<float>::quiet_NaN();
};

ArtifactRuntimeMetrics BenchmarkArtifact(
    const pmg::BuiltPmgArtifact& artifact) {
    ArtifactRuntimeMetrics metrics;
    const pmg::RuntimeControllerConfig runtime_config =
        pmg::RuntimeControllerConfigFromArtifact(artifact);
    pmg::PointCloudAlignment alignment(
        artifact.skeleton, runtime_config.transition_blend_frames);
    pmg::RuntimeController controller(
        artifact.graph, alignment, runtime_config);
    const pmg::ParameterDomain domain =
        artifact.graph.Node(0).motion_space.Domain();
    pmg::ParameterVector start_parameter = domain.Bounds().min_corner;
    for (std::size_t dimension = 0; dimension < start_parameter.size(); ++dimension) {
        start_parameter[dimension] =
            0.5f * (domain.Bounds().min_corner[dimension] +
                    domain.Bounds().max_corner[dimension]);
    }
    controller.Start(0, start_parameter, artifact.metadata.frames_per_second);

    std::mt19937 random(12345u);
    pmg::RuntimeControlRequest request =
        pmg::ChooseRandomOutgoingTransition(
            artifact.graph, controller.CurrentNode(), random)
            .request;
    const float delta_seconds = 1.0f / artifact.metadata.frames_per_second;
    metrics.simulated_frames =
        static_cast<int>(10.0f * artifact.metadata.frames_per_second);
    int previous_transitions = 0;
    const auto runtime_start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < metrics.simulated_frames; ++frame) {
        controller.Update(delta_seconds, request);
        if (controller.CompletedTransitions() != previous_transitions) {
            previous_transitions = controller.CompletedTransitions();
            if (artifact.graph.OutgoingEdgeIndices(
                    controller.CurrentNode()).empty()) {
                metrics.simulated_frames = frame + 1;
                break;
            }
            request =
                pmg::ChooseRandomOutgoingTransition(
                    artifact.graph, controller.CurrentNode(), random)
                    .request;
        }
    }
    const double runtime_seconds =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - runtime_start)
            .count();
    metrics.transitions = controller.CompletedTransitions();
    metrics.runtime_frames_per_second =
        runtime_seconds > 0.0
            ? static_cast<double>(metrics.simulated_frames) / runtime_seconds
            : 0.0;

    try {
        pmg::GoalDirectedLocomotionConfig steering_config;
        steering_config.runtime = runtime_config;
        pmg::GoalDirectedLocomotion steering(
            artifact.graph, artifact.skeleton, 0,
            artifact.metadata.frames_per_second, steering_config);
        pmg::PointCloudAlignment goal_alignment(
            artifact.skeleton, runtime_config.transition_blend_frames);
        pmg::RuntimeController goal_controller(
            artifact.graph, goal_alignment, runtime_config);
        goal_controller.Start(
            0, start_parameter, artifact.metadata.frames_per_second);
        pmg::GoalRequest goal;
        goal.target_position = {10.0f, 0.0f, 10.0f};
        metrics.target_min_distance =
            std::numeric_limits<float>::infinity();
        const int goal_frames =
            static_cast<int>(60.0f * artifact.metadata.frames_per_second);
        for (int frame = 0; frame < goal_frames; ++frame) {
            const pmg::Pose pose = goal_controller.CurrentPose();
            const float delta_x = goal.target_position.x - pose.root_position.x;
            const float delta_z = goal.target_position.z - pose.root_position.z;
            metrics.target_min_distance = std::min(
                metrics.target_min_distance,
                std::sqrt(delta_x * delta_x + delta_z * delta_z));
            goal_controller.Update(
                delta_seconds, steering.RequestForPose(pose, goal));
        }
    } catch (const std::exception&) {
        // Goal-directed control is only defined for a one-dimensional
        // steerable node with a usable self-transition.
    }
    return metrics;
}

void WriteArtifactReports(
    const pmg::BuiltPmgArtifact& artifact,
    const std::filesystem::path& artifact_path,
    double build_seconds) {
    const std::filesystem::path output_directory =
        artifact_path.has_parent_path() ? artifact_path.parent_path()
                                        : std::filesystem::current_path();
    const std::filesystem::path tables_directory =
        output_directory / "tables";
    std::filesystem::create_directories(tables_directory);
    const std::uintmax_t artifact_bytes =
        std::filesystem::file_size(artifact_path);

    int source_report_count = 0;
    int good_count = 0;
    int neutral_count = 0;
    int bad_count = 0;
    int shrunken_box_count = 0;
    for (const pmg::EdgeBuildMetadata& edge : artifact.metadata.edge_builds) {
        for (const pmg::SourceSampleBuildReport& source :
             edge.report.source_reports) {
            ++source_report_count;
            good_count += source.good_count;
            neutral_count += source.neutral_count;
            bad_count += source.bad_count;
            if (BoxesDiffer(
                    source.target_box_before_shrink,
                    source.target_box_after_shrink)) {
                ++shrunken_box_count;
            }
        }
    }
    const ArtifactRuntimeMetrics runtime = BenchmarkArtifact(artifact);

    {
        std::ofstream config(output_directory / "config.json");
        config << "{\n"
               << "  \"format\": \"PMG_GRAPH_V6\",\n"
               << "  \"units\": \"" << JsonEscape(artifact.metadata.units) << "\",\n"
               << "  \"generated_frame_count\": "
               << artifact.metadata.generated_frame_count << ",\n"
               << "  \"frames_per_second\": "
               << artifact.metadata.frames_per_second << ",\n"
               << "  \"source_bvh_paths\": [";
        for (std::size_t index = 0;
             index < artifact.metadata.source_bvh_paths.size(); ++index) {
            if (index > 0) {
                config << ", ";
            }
            config << '"' << JsonEscape(
                artifact.metadata.source_bvh_paths[index]) << '"';
        }
        config << "],\n  \"registrations\": [\n";
        for (std::size_t index = 0;
             index < artifact.metadata.node_registrations.size(); ++index) {
            const pmg::NodeRegistrationMetadata& registration =
                artifact.metadata.node_registrations[index];
            config << "    {\"node\": \"" << JsonEscape(registration.node_name)
                   << "\", \"cycle_joint\": \""
                   << JsonEscape(registration.cycle_joint)
                   << "\", \"contact_joints\": [";
            for (std::size_t joint = 0;
                 joint < registration.contact_joints.size(); ++joint) {
                if (joint > 0) {
                    config << ", ";
                }
                config << '"' << JsonEscape(
                    registration.contact_joints[joint]) << '"';
            }
            config << "], \"min_contact_frames\": "
                   << registration.min_contact_frames
                   << ", \"dtw_refine\": "
                   << (registration.dtw_refine ? "true" : "false") << "}";
            config << (index + 1 ==
                               artifact.metadata.node_registrations.size()
                           ? "\n"
                           : ",\n");
        }
        config << "  ],\n  \"edges\": [\n";
        for (std::size_t index = 0;
             index < artifact.metadata.edge_builds.size(); ++index) {
            const pmg::EdgeBuildMetadata& edge =
                artifact.metadata.edge_builds[index];
            config << "    {\"source\": \"" << JsonEscape(edge.source_node)
                   << "\", \"target\": \"" << JsonEscape(edge.target_node)
                   << "\", \"tgood\": "
                   << edge.config.good_transition_threshold
                   << ", \"tbad\": " << edge.config.bad_transition_threshold
                   << ", \"source_samples\": "
                   << edge.config.source_sample_count
                   << ", \"target_samples\": "
                   << edge.config.target_sample_count
                   << ", \"seed\": " << edge.config.seed << "}";
            config << (index + 1 == artifact.metadata.edge_builds.size()
                           ? "\n"
                           : ",\n");
        }
        config << "  ]\n}\n";
    }
    {
        std::ofstream metrics_file(output_directory / "metrics.json");
        metrics_file << "{\n"
                     << "  \"build_seconds\": " << build_seconds << ",\n"
                     << "  \"artifact_bytes\": " << artifact_bytes << ",\n"
                     << "  \"nodes\": " << artifact.graph.NumNodes() << ",\n"
                     << "  \"edges\": " << artifact.graph.NumEdges() << ",\n"
                     << "  \"source_reports\": " << source_report_count << ",\n"
                     << "  \"good_samples\": " << good_count << ",\n"
                     << "  \"neutral_samples\": " << neutral_count << ",\n"
                     << "  \"bad_samples\": " << bad_count << ",\n"
                     << "  \"shrunken_boxes\": " << shrunken_box_count << ",\n"
                     << "  \"runtime_simulated_frames\": "
                     << runtime.simulated_frames << ",\n"
                     << "  \"runtime_transitions\": " << runtime.transitions << ",\n"
                     << "  \"runtime_frames_per_second\": "
                     << runtime.runtime_frames_per_second << ",\n"
                     << "  \"target_10_10_min_distance\": ";
        if (std::isfinite(runtime.target_min_distance)) {
            metrics_file << runtime.target_min_distance;
        } else {
            metrics_file << "null";
        }
        metrics_file << "\n}\n";
    }
    {
        std::ofstream table(tables_directory / "edge_samples.csv");
        table << "edge,source_parameter,good,neutral,bad,min_distance,"
                 "median_distance,box_shrunk,accepted,reject_reason\n";
        for (const pmg::EdgeBuildMetadata& edge :
             artifact.metadata.edge_builds) {
            for (const pmg::SourceSampleBuildReport& source :
                 edge.report.source_reports) {
                table << edge.source_node << "->" << edge.target_node << ",\"";
                for (std::size_t dimension = 0;
                     dimension < source.source_parameter.size(); ++dimension) {
                    if (dimension > 0) {
                        table << ' ';
                    }
                    table << source.source_parameter[dimension];
                }
                table << "\"," << source.good_count << ','
                      << source.neutral_count << ',' << source.bad_count << ','
                      << source.min_distance << ',' << source.median_distance
                      << ',' << static_cast<int>(BoxesDiffer(
                                     source.target_box_before_shrink,
                                     source.target_box_after_shrink))
                      << ',' << static_cast<int>(source.accepted) << ",\""
                      << source.reject_reason << "\"\n";
            }
        }
    }
    {
        std::ofstream report(output_directory / "report.md");
        report << "# PMG Paper-Core Build Report\n\n"
               << "## Purpose\n\n"
               << "Offline-built PMG artifact validated through the same "
                  "complete artifact used by online runtime playback.\n\n"
               << "## Inputs\n\n"
               << "- Source BVHs: " << artifact.metadata.source_bvh_paths.size()
               << "\n- Units: " << artifact.metadata.units
               << "\n- Seed/config: `config.json`\n\n"
               << "## Outputs\n\n"
               << "- Artifact: `" << artifact_path.filename().string()
               << "` (" << artifact_bytes << " bytes)\n"
               << "- Edge table: `tables/edge_samples.csv`\n\n"
               << "## Metrics\n\n"
               << "- Build time: " << build_seconds << " s\n"
               << "- Nodes / edges: " << artifact.graph.NumNodes() << " / "
               << artifact.graph.NumEdges() << "\n"
               << "- GOOD / NEUTRAL / BAD: " << good_count << " / "
               << neutral_count << " / " << bad_count << "\n"
               << "- Shrunken target boxes: " << shrunken_box_count << "\n"
               << "- Runtime transitions (10 s simulated): "
               << runtime.transitions << "\n"
               << "- Runtime throughput: "
               << runtime.runtime_frames_per_second << " frames/s\n"
               << "- Goal (10,10) minimum distance: "
               << (std::isfinite(runtime.target_min_distance)
                       ? std::to_string(runtime.target_min_distance)
                       : std::string("not applicable"))
               << "\n\n"
               << "## Limitations\n\n"
               << "- This validates renderer-conditioned PMG behavior on the "
                  "included BVH corpus; it does not reproduce the paper's "
                  "boxing/platform datasets.\n"
               << "- Foot locking is an optional post-process and is not part "
                  "of the PMG artifact runtime contract.\n";
    }
}


int BuildGraphCommand(const std::string& spec_path,
                      const std::string& output_path,
                      const pmg::PmgBuilderConfig& config) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(spec_path);
    pmg::ArtifactBuildConfig artifact_config;
    artifact_config.default_edge_config = config;
    const auto build_start = std::chrono::steady_clock::now();
    const pmg::BuiltPmgArtifact artifact =
        pmg::BuildPmgArtifactFromSpec(spec, artifact_config);
    const double build_seconds =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - build_start)
            .count();
    const std::filesystem::path artifact_path(output_path);
    if (artifact_path.has_parent_path()) {
        std::filesystem::create_directories(artifact_path.parent_path());
    }
    pmg::SavePmgArtifactText(artifact, output_path);
    WriteArtifactReports(artifact, artifact_path, build_seconds);
    std::cout << "wrote graph: " << output_path << "\n";
    std::cout << "format=PMG_GRAPH_V6\n";
    std::cout << "nodes=" << artifact.graph.NumNodes()
              << " edges=" << artifact.graph.NumEdges() << "\n";
    std::cout << "skeleton_joints=" << artifact.skeleton.NumJoints() << "\n";
    std::cout << "generated_frame_count="
              << artifact.metadata.generated_frame_count << "\n";
    std::cout << "frames_per_second="
              << artifact.metadata.frames_per_second << "\n";
    return 0;
}

int InspectGraphCommand(const std::string& graph_path) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(graph_path);
    const pmg::ParametricMotionGraph& graph = artifact.graph;
    std::cout << "graph: " << graph_path << "\n";
    std::cout << "skeleton_joints: " << artifact.skeleton.NumJoints() << "\n";
    std::cout << "runtime: frames=" << artifact.metadata.generated_frame_count
              << " fps=" << artifact.metadata.frames_per_second << "\n";
    std::cout << "nodes: " << graph.NumNodes() << "\n";
    for (int node_index = 0; node_index < graph.NumNodes(); ++node_index) {
        const pmg::PmgNode& node = graph.Node(node_index);
        std::cout << "  [" << node_index << "] " << node.name
                  << " dim=" << node.motion_space.ParameterDimension()
                  << " examples=" << node.motion_space.NumExamples() << "\n";
    }
    std::cout << "edges: " << graph.NumEdges() << "\n";
    for (int edge_index = 0; edge_index < graph.NumEdges(); ++edge_index) {
        const pmg::PmgEdge& edge = graph.Edge(edge_index);
        std::cout << "  [" << edge_index << "] "
                  << edge.source_node << " -> " << edge.target_node
                  << " samples=" << edge.samples.size() << "\n";
    }
    return 0;
}

}  // namespace

namespace pmgcli {

std::optional<int> TryRunGraphCommand(int argc, char** argv) {
    const std::string command = argc > 1 ? argv[1] : "";
    if (command == "--build-graph" && argc >= 4) {
        const pmg::PmgBuilderConfig config =
            ParseBuilderConfigOptions(argc, argv, 4);
        return BuildGraphCommand(argv[2], argv[3], config);
    }
    if (command == "--validate-graph-spec" && argc == 3) {
        return ValidateGraphSpecCommand(argv[2]);
    }
    if (command == "--diagnose-graph-edge" && argc >= 5) {
        const pmg::PmgBuilderConfig config =
            ParseBuilderConfigOptions(argc, argv, 5);
        return DiagnoseGraphEdgeCommand(argv[2], argv[3], argv[4], config);
    }
    if (command == "--inspect-graph" && argc == 3) {
        return InspectGraphCommand(argv[2]);
    }
    return std::nullopt;
}

}  // namespace pmgcli
