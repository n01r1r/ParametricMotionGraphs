#include "pmg/GraphSpec.h"

#include "pmg/MotionSpacePreparation.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
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

        if (keyword == "parameter_metric") {
            std::string node_name;
            std::string metric_name;
            if (!(line >> node_name >> metric_name)) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": expected parameter_metric <node> <turn_rate|none>");
            }
            auto node_it = std::find_if(
                spec.nodes.begin(), spec.nodes.end(),
                [&](const GraphSpecNode& node) { return node.name == node_name; });
            if (node_it == spec.nodes.end()) {
                throw std::runtime_error("LoadGraphSpec line " +
                                         std::to_string(line_number) +
                                         ": parameter_metric references unknown node '" +
                                         node_name + "'");
            }
            if (metric_name == "turn_rate") {
                node_it->parameter_metric = ParameterMetric::kTurnRate;
            } else if (metric_name == "none") {
                node_it->parameter_metric = ParameterMetric::kNone;
            } else {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": unknown parameter metric '" + metric_name + "'");
            }
            if (node_it->parameter_metric != ParameterMetric::kNone &&
                node_it->parameter_dimension != 1) {
                throw std::runtime_error(
                    "LoadGraphSpec line " + std::to_string(line_number) +
                    ": parameter_metric requires a one-dimensional node");
            }
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
    MotionSpacePreparationConfig preparation_config;
    preparation_config.calibration_frames_per_second =
        builder_config.generated_frames_per_second;
    preparation_config.skeleton_offset_tolerance =
        skeleton_offset_tolerance;
    const PreparedMotionSpaces prepared =
        PrepareMotionSpaces(spec, preparation_config);

    ParametricMotionGraph graph;
    for (const GraphSpecNode& node : spec.nodes) {
        graph.AddNode(node.name, prepared.Node(node.name).production);
    }

    for (const GraphSpecEdge& edge_spec : spec.edges) {
        const int source_index = FindNodeIndex(graph, edge_spec.source_node);
        const int target_index = FindNodeIndex(graph, edge_spec.target_node);
        EdgeBuildResult edge_result = PmgBuilder::BuildEdgeWithReport(
            prepared.skeleton,
            source_index,
            target_index,
            graph.Node(source_index).motion_space,
            graph.Node(target_index).motion_space,
            builder_config);
        // Paper: a graph is the set of edges that pass. A single
        // transition-incompatible pair must not abort the whole build (this
        // matches the viewer's tolerant build); rejected edges are skipped.
        if (!edge_result.edge.samples.empty()) {
            graph.AddEdge(std::move(edge_result.edge));
        }
    }

    if (!spec.edges.empty() && graph.NumEdges() == 0) {
        throw std::runtime_error(
            "BuildGraphFromSpec: every declared edge was rejected; no "
            "transitions built");
    }
    return graph;
}

BuiltPmgArtifact BuildPmgArtifactFromSpec(
    const GraphSpec& spec,
    const ArtifactBuildConfig& config) {
    BuiltPmgArtifact artifact;
    MotionSpacePreparationConfig preparation_config;
    preparation_config.default_cycle_joint = config.default_cycle_joint;
    preparation_config.default_contact_joints = config.default_contact_joints;
    preparation_config.default_min_contact_frames =
        config.default_min_contact_frames;
    preparation_config.default_dtw_refine = config.default_dtw_refine;
    preparation_config.calibration_frames_per_second =
        config.default_edge_config.generated_frames_per_second;
    preparation_config.skeleton_offset_tolerance =
        config.skeleton_offset_tolerance;
    const PreparedMotionSpaces prepared =
        PrepareMotionSpaces(spec, preparation_config);

    artifact.skeleton = prepared.skeleton;
    artifact.metadata.source_bvh_paths = prepared.source_bvh_paths;

    std::map<std::string, int> node_indices;
    for (const GraphSpecNode& node : spec.nodes) {
        const PreparedMotionSpace& prepared_node = prepared.Node(node.name);
        node_indices.emplace(
            node.name,
            artifact.graph.AddNode(node.name, prepared_node.production));
        artifact.metadata.node_registrations.push_back(
            prepared_node.registration);
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

        // Tolerant build: the reject is recorded in edge_builds above (visible
        // via the report and the CLI warning), and the edge is skipped instead
        // of aborting -- consistent with the viewer and with paper semantics.
        if (result.report.edge_created) {
            artifact.graph.AddEdge(std::move(result.edge));
        }
    }

    if (!spec.edges.empty() && artifact.graph.NumEdges() == 0) {
        throw std::runtime_error(
            "BuildPmgArtifactFromSpec: every declared edge was rejected; no "
            "transitions built");
    }
    return artifact;
}

}  // namespace pmg
