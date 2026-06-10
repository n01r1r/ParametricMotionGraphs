#pragma once

#include "pmg/ParametricMotionGraph.h"
#include "pmg/PmgBuilder.h"

#include <string>
#include <vector>

namespace pmg {

struct GraphSpecNode {
    std::string name;
    int parameter_dimension = 0;
};

struct GraphSpecExample {
    std::string node_name;
    ParameterVector parameter;
    std::string bvh_path;
};

struct GraphSpecEdge {
    std::string source_node;
    std::string target_node;
};

struct GraphSpec {
    std::vector<GraphSpecNode> nodes;
    std::vector<GraphSpecExample> examples;
    std::vector<GraphSpecEdge> edges;
};

// Minimal whitespace-delimited spec:
//   node <name> <parameter_dimension>
//   example <node_name> <p0> ... <pN-1> <path/to/file.bvh>
//   edge <source_node> <target_node>
// Lines starting with '#' are ignored. Relative BVH paths are resolved against
// the spec file's directory by LoadGraphSpec(). Names and paths may not contain
// whitespace.
GraphSpec LoadGraphSpec(const std::string& path);

ParametricMotionGraph BuildGraphFromSpec(
    const GraphSpec& spec,
    const PmgBuilderConfig& builder_config,
    float skeleton_offset_tolerance = 1.0e-4f);

}  // namespace pmg
