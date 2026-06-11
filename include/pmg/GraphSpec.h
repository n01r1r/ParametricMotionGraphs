#pragma once

#include "pmg/PmgArtifact.h"
#include "pmg/PmgBuilder.h"

#include <string>
#include <vector>

namespace pmg {

struct GraphSpecNode {
    std::string name;
    int parameter_dimension = 0;
    bool has_registration_config = false;
    std::string cycle_joint;
    std::vector<std::string> contact_joints;
    int min_contact_frames = 3;
    bool dtw_refine = true;
};

struct GraphSpecExample {
    std::string node_name;
    ParameterVector parameter;
    std::string bvh_path;
};

struct GraphSpecEdge {
    std::string source_node;
    std::string target_node;
    bool has_build_config = false;
    PmgBuilderConfig build_config;
};

struct GraphSpec {
    std::vector<GraphSpecNode> nodes;
    std::vector<GraphSpecExample> examples;
    std::vector<GraphSpecEdge> edges;
};

// Minimal whitespace-delimited spec:
//   node <name> <parameter_dimension>
//   registration <node> <cycle_joint|-> <contact_joint_csv|-> <min_frames> <dtw:0|1>
//   example <node_name> <p0> ... <pN-1> <path/to/file.bvh>
//   edge <source_node> <target_node>
//   edge_config <source> <target> <tgood> <tbad> <source_samples>
//               <target_samples> <seed>
// Lines starting with '#' are ignored. Relative BVH paths are resolved against
// the spec file's directory by LoadGraphSpec(). Names and paths may not contain
// whitespace.
GraphSpec LoadGraphSpec(const std::string& path);

ParametricMotionGraph BuildGraphFromSpec(
    const GraphSpec& spec,
    const PmgBuilderConfig& builder_config,
    float skeleton_offset_tolerance = 1.0e-4f);

struct ArtifactBuildConfig {
    PmgBuilderConfig default_edge_config;
    std::string default_cycle_joint;
    std::vector<std::string> default_contact_joints{"LeftAnkle", "RightAnkle"};
    int default_min_contact_frames = 3;
    bool default_dtw_refine = true;
    float skeleton_offset_tolerance = 1.0e-4f;
};

// Builds the paper-core offline artifact: loads and cycle-normalizes examples,
// installs contact/DTW registration, samples every requested edge, and returns
// the Skeleton plus reproducibility metadata needed by online playback.
BuiltPmgArtifact BuildPmgArtifactFromSpec(
    const GraphSpec& spec,
    const ArtifactBuildConfig& config = {});

}  // namespace pmg
