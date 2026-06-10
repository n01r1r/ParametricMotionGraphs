#include "pmg/GraphSpec.h"

#include "pmg/BvhLoader.h"
#include "pmg/SkeletonCompatibility.h"

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

}  // namespace pmg
