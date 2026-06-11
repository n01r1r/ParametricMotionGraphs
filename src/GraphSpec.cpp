#include "pmg/GraphSpec.h"

#include "pmg/BvhLoader.h"
#include "pmg/ContactDetection.h"
#include "pmg/MotionRegistration.h"
#include "pmg/SkeletonCompatibility.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace pmg {

namespace {

std::string StripComment(const std::string& line) {
    const std::size_t hash = line.find('#');
    if (hash == std::string::npos) {
        return line;
    }
    return line.substr(0, hash);
}

const GraphSpecNode& FindSpecNode(const GraphSpec& spec, const std::string& name) {
    for (const GraphSpecNode& node : spec.nodes) {
        if (node.name == name) {
            return node;
        }
    }
    throw std::runtime_error("GraphSpec: unknown node '" + name + "'");
}

int FindNodeIndex(const ParametricMotionGraph& graph, const std::string& name) {
    for (int index = 0; index < graph.NumNodes(); ++index) {
        if (graph.Node(index).name == name) {
            return index;
        }
    }
    throw std::runtime_error("BuildGraphFromSpec: unknown graph node '" + name + "'");
}

std::string ResolveRelativePath(const std::filesystem::path& base_directory,
                                const std::string& path_text) {
    const std::filesystem::path path(path_text);
    if (path.is_absolute()) {
        return path.string();
    }
    return (base_directory / path).lexically_normal().string();
}

std::vector<std::string> SplitCommaList(const std::string& text) {
    std::vector<std::string> values;
    std::istringstream stream(text);
    std::string value;
    while (std::getline(stream, value, ',')) {
        if (!value.empty()) {
            values.push_back(value);
        }
    }
    return values;
}

int FindJointIndex(const Skeleton& skeleton, const std::string& name,
                   const std::string& context) {
    for (int joint_index = 0; joint_index < skeleton.NumJoints(); ++joint_index) {
        if (skeleton.joints[joint_index].name == name) {
            return joint_index;
        }
    }
    throw std::runtime_error(context + ": unknown joint '" + name + "'");
}

}  // namespace

GraphSpec LoadGraphSpec(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("LoadGraphSpec: failed to open '" + path + "'");
    }

    const std::filesystem::path base_directory =
        std::filesystem::absolute(std::filesystem::path(path)).parent_path();

    GraphSpec spec;
    std::map<std::string, int> node_dimensions;
    std::string raw_line;
    int line_number = 0;
    while (std::getline(input, raw_line)) {
        ++line_number;
        std::istringstream line(StripComment(raw_line));
        std::string keyword;
        if (!(line >> keyword)) {
            continue;
        }

        if (keyword == "node") {
            GraphSpecNode node;
            if (!(line >> node.name >> node.parameter_dimension)) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": expected node <name> <parameter_dimension>");
            }
            if (node.parameter_dimension <= 0) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": parameter_dimension must be positive");
            }
            if (node_dimensions.count(node.name) != 0) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": duplicate node '" + node.name + "'");
            }
            node_dimensions[node.name] = node.parameter_dimension;
            spec.nodes.push_back(node);
            continue;
        }

        if (keyword == "example") {
            GraphSpecExample example;
            if (!(line >> example.node_name)) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": expected example <node_name> ...");
            }
            const auto dimension_it = node_dimensions.find(example.node_name);
            if (dimension_it == node_dimensions.end()) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": example references unknown node '" +
                                         example.node_name + "'");
            }
            example.parameter.resize(static_cast<std::size_t>(dimension_it->second));
            for (float& value : example.parameter) {
                if (!(line >> value)) {
                    throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                             ": missing parameter value");
                }
            }
            std::string bvh_path;
            if (!(line >> bvh_path)) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": missing BVH path");
            }
            example.bvh_path = ResolveRelativePath(base_directory, bvh_path);
            spec.examples.push_back(example);
            continue;
        }

        if (keyword == "registration") {
            std::string node_name;
            std::string cycle_joint;
            std::string contact_joint_csv;
            int min_contact_frames = 0;
            int dtw_refine = 0;
            if (!(line >> node_name >> cycle_joint >> contact_joint_csv >>
                  min_contact_frames >> dtw_refine)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected registration <node> <cycle_joint|-> "
                    "<contact_joint_csv|-> <min_frames> <dtw:0|1>");
            }
            auto node_it = std::find_if(
                spec.nodes.begin(), spec.nodes.end(),
                [&](const GraphSpecNode& node) { return node.name == node_name; });
            if (node_it == spec.nodes.end()) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": registration references unknown node '" +
                                         node_name + "'");
            }
            if (node_it->has_registration_config) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": duplicate registration for node '" +
                                         node_name + "'");
            }
            if (min_contact_frames <= 0 || (dtw_refine != 0 && dtw_refine != 1)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": registration requires positive min_frames and dtw 0 or 1");
            }
            node_it->has_registration_config = true;
            node_it->cycle_joint = cycle_joint == "-" ? "" : cycle_joint;
            node_it->contact_joints =
                contact_joint_csv == "-" ? std::vector<std::string>{}
                                          : SplitCommaList(contact_joint_csv);
            node_it->min_contact_frames = min_contact_frames;
            node_it->dtw_refine = dtw_refine != 0;
            continue;
        }

        if (keyword == "edge") {
            GraphSpecEdge edge;
            if (!(line >> edge.source_node >> edge.target_node)) {
                throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                         ": expected edge <source_node> <target_node>");
            }
            (void)FindSpecNode(spec, edge.source_node);
            (void)FindSpecNode(spec, edge.target_node);
            spec.edges.push_back(edge);
            continue;
        }

        if (keyword == "edge_config") {
            GraphSpecEdge configured_edge;
            if (!(line >> configured_edge.source_node >> configured_edge.target_node >>
                  configured_edge.build_config.good_transition_threshold >>
                  configured_edge.build_config.bad_transition_threshold >>
                  configured_edge.build_config.source_sample_count >>
                  configured_edge.build_config.target_sample_count >>
                  configured_edge.build_config.seed)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected edge_config <source> <target> <tgood> <tbad> "
                    "<source_samples> <target_samples> <seed>");
            }
            (void)FindSpecNode(spec, configured_edge.source_node);
            (void)FindSpecNode(spec, configured_edge.target_node);
            auto edge_it = std::find_if(
                spec.edges.begin(), spec.edges.end(),
                [&](const GraphSpecEdge& edge) {
                    return edge.source_node == configured_edge.source_node &&
                           edge.target_node == configured_edge.target_node;
                });
            if (edge_it == spec.edges.end()) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": edge_config requires a preceding matching edge");
            }
            if (edge_it->has_build_config) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": duplicate edge_config");
            }
            configured_edge.has_build_config = true;
            edge_it->has_build_config = true;
            edge_it->build_config = configured_edge.build_config;
            continue;
        }

        throw std::runtime_error("LoadGraphSpec line " + std::to_string(line_number) +
                                 ": unknown keyword '" + keyword + "'");
    }

    if (spec.nodes.empty()) {
        throw std::runtime_error("LoadGraphSpec: spec contains no nodes");
    }
    return spec;
}

ParametricMotionGraph BuildGraphFromSpec(
    const GraphSpec& spec,
    const PmgBuilderConfig& builder_config,
    float skeleton_offset_tolerance) {
    if (spec.nodes.empty()) {
        throw std::runtime_error("BuildGraphFromSpec: spec contains no nodes");
    }

    std::map<std::string, ParametricMotionSpace> spaces;
    for (const GraphSpecNode& node : spec.nodes) {
        spaces.emplace(node.name, ParametricMotionSpace(node.name, node.parameter_dimension));
    }

    std::optional<Skeleton> reference_skeleton;
    for (const GraphSpecExample& example : spec.examples) {
        const auto node_it = spaces.find(example.node_name);
        if (node_it == spaces.end()) {
            throw std::runtime_error("BuildGraphFromSpec: example references unknown node '" +
                                     example.node_name + "'");
        }

        const BvhData bvh = BvhLoader::Load(example.bvh_path);
        if (!reference_skeleton.has_value()) {
            reference_skeleton = bvh.skeleton;
        } else {
            RequireSkeletonCompatible(*reference_skeleton, bvh.skeleton,
                                      "BuildGraphFromSpec", skeleton_offset_tolerance);
        }
        node_it->second.AddExample(example.parameter, bvh.clip);
    }

    if (!reference_skeleton.has_value()) {
        throw std::runtime_error("BuildGraphFromSpec: spec contains no examples");
    }

    ParametricMotionGraph graph;
    for (const GraphSpecNode& node : spec.nodes) {
        const ParametricMotionSpace& space = spaces.at(node.name);
        if (space.NumExamples() == 0) {
            throw std::runtime_error("BuildGraphFromSpec: node '" + node.name +
                                     "' has no examples");
        }
        graph.AddNode(node.name, space);
    }

    for (const GraphSpecEdge& edge_spec : spec.edges) {
        const int source_index = FindNodeIndex(graph, edge_spec.source_node);
        const int target_index = FindNodeIndex(graph, edge_spec.target_node);
        EdgeBuildResult edge_result = PmgBuilder::BuildEdgeWithReport(
            *reference_skeleton,
            source_index,
            target_index,
            graph.Node(source_index).motion_space,
            graph.Node(target_index).motion_space,
            builder_config);
        if (edge_result.edge.samples.empty()) {
            throw std::runtime_error(
                "BuildGraphFromSpec: edge '" + edge_spec.source_node + " -> " +
                edge_spec.target_node + "' produced no valid transition samples: " +
                edge_result.report.reject_reason);
        }
        graph.AddEdge(std::move(edge_result.edge));
    }

    return graph;
}

BuiltPmgArtifact BuildPmgArtifactFromSpec(
    const GraphSpec& spec,
    const ArtifactBuildConfig& config) {
    if (spec.nodes.empty()) {
        throw std::runtime_error("BuildPmgArtifactFromSpec: spec contains no nodes");
    }

    BuiltPmgArtifact artifact;
    std::map<std::string, ParametricMotionSpace> spaces;
    std::optional<Skeleton> reference_skeleton;

    for (const GraphSpecNode& node : spec.nodes) {
        ParametricMotionSpace space(node.name, node.parameter_dimension);
        NodeRegistrationMetadata registration;
        registration.node_name = node.name;
        registration.cycle_joint =
            node.has_registration_config ? node.cycle_joint : config.default_cycle_joint;
        registration.contact_joints =
            node.has_registration_config ? node.contact_joints : config.default_contact_joints;
        registration.min_contact_frames =
            node.has_registration_config ? node.min_contact_frames
                                         : config.default_min_contact_frames;
        registration.dtw_refine =
            node.has_registration_config ? node.dtw_refine : config.default_dtw_refine;

        for (const GraphSpecExample& example : spec.examples) {
            if (example.node_name != node.name) {
                continue;
            }
            BvhData bvh = BvhLoader::Load(example.bvh_path);
            if (!reference_skeleton.has_value()) {
                reference_skeleton = bvh.skeleton;
            } else {
                RequireSkeletonCompatible(
                    *reference_skeleton, bvh.skeleton, "BuildPmgArtifactFromSpec",
                    config.skeleton_offset_tolerance);
            }

            MotionClip clip = std::move(bvh.clip);
            if (!registration.cycle_joint.empty()) {
                const int cycle_joint = FindJointIndex(
                    *reference_skeleton, registration.cycle_joint,
                    "BuildPmgArtifactFromSpec cycle joint");
                const ContactDetectionSettings cycle_settings =
                    EstimateContactSettings(*reference_skeleton, clip, {cycle_joint});
                clip = ExtractFirstCycle(
                    *reference_skeleton, clip, cycle_joint, cycle_settings);
            }
            space.AddExample(example.parameter, std::move(clip));
            artifact.metadata.source_bvh_paths.push_back(example.bvh_path);
        }
        if (space.NumExamples() == 0) {
            throw std::runtime_error("BuildPmgArtifactFromSpec: node '" + node.name +
                                     "' has no examples");
        }

        if (!registration.contact_joints.empty()) {
            std::vector<int> contact_joint_indices;
            for (const std::string& joint_name : registration.contact_joints) {
                contact_joint_indices.push_back(FindJointIndex(
                    *reference_skeleton, joint_name,
                    "BuildPmgArtifactFromSpec contact joint"));
            }
            ContactDetectionSettings settings = EstimateContactSettings(
                *reference_skeleton, space.Examples().front().clip,
                contact_joint_indices);
            settings.min_contact_frames = registration.min_contact_frames;
            RegisterSpaceByContacts(
                space, *reference_skeleton, contact_joint_indices, settings);
            if (registration.dtw_refine) {
                RefineRegistrationByDtw(space, *reference_skeleton, {});
            }
        } else if (registration.dtw_refine) {
            throw std::runtime_error(
                "BuildPmgArtifactFromSpec: node '" + node.name +
                "' enables DTW refinement without contact joints");
        }

        artifact.metadata.node_registrations.push_back(registration);
        spaces.emplace(node.name, std::move(space));
    }

    if (!reference_skeleton.has_value()) {
        throw std::runtime_error("BuildPmgArtifactFromSpec: spec contains no examples");
    }
    artifact.skeleton = *reference_skeleton;

    std::map<std::string, int> node_indices;
    for (const GraphSpecNode& node : spec.nodes) {
        node_indices.emplace(
            node.name, artifact.graph.AddNode(node.name, spaces.at(node.name)));
    }

    artifact.metadata.generated_frame_count =
        config.default_edge_config.generated_frame_count;
    artifact.metadata.frames_per_second =
        config.default_edge_config.generated_frames_per_second;
    if (artifact.metadata.generated_frame_count <= 1 ||
        artifact.metadata.frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "BuildPmgArtifactFromSpec: generated runtime frame settings are invalid");
    }

    for (const GraphSpecEdge& edge_spec : spec.edges) {
        PmgBuilderConfig edge_config =
            edge_spec.has_build_config ? edge_spec.build_config
                                       : config.default_edge_config;
        edge_config.generated_frame_count = artifact.metadata.generated_frame_count;
        edge_config.generated_frames_per_second = artifact.metadata.frames_per_second;

        const int source_index = node_indices.at(edge_spec.source_node);
        const int target_index = node_indices.at(edge_spec.target_node);
        EdgeBuildResult result = PmgBuilder::BuildEdgeWithReport(
            artifact.skeleton, source_index, target_index,
            artifact.graph.Node(source_index).motion_space,
            artifact.graph.Node(target_index).motion_space, edge_config);
        EdgeBuildMetadata metadata;
        metadata.source_node = edge_spec.source_node;
        metadata.target_node = edge_spec.target_node;
        metadata.config = edge_config;
        metadata.report = result.report;
        artifact.metadata.edge_builds.push_back(metadata);

        if (!result.report.edge_created) {
            throw std::runtime_error(
                "BuildPmgArtifactFromSpec: edge '" + edge_spec.source_node +
                " -> " + edge_spec.target_node + "' rejected: " +
                result.report.reject_reason);
        }
        artifact.graph.AddEdge(std::move(result.edge));
    }
    return artifact;
}

}  // namespace pmg
