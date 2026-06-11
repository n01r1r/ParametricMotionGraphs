#pragma once

#include "pmg/ParametricMotionGraph.h"
#include "pmg/PmgBuilder.h"
#include "pmg/RuntimeController.h"
#include "pmg/Skeleton.h"

#include <string>
#include <vector>

namespace pmg {

struct NodeRegistrationMetadata {
    std::string node_name;
    std::string cycle_joint;
    std::vector<std::string> contact_joints;
    int min_contact_frames = 3;
    bool dtw_refine = true;
};

struct EdgeBuildMetadata {
    std::string source_node;
    std::string target_node;
    PmgBuilderConfig config;
    EdgeBuildReport report;
};

struct PmgArtifactMetadata {
    std::string units = "BVH native units";
    int generated_frame_count = 0;
    float frames_per_second = 0.0f;
    std::vector<std::string> source_bvh_paths;
    std::vector<NodeRegistrationMetadata> node_registrations;
    std::vector<EdgeBuildMetadata> edge_builds;
};

// Complete offline output needed by the online runtime. Unlike legacy graph
// files, this artifact owns the Skeleton required by point-cloud alignment and
// records the build settings needed to reproduce its transition samples.
struct BuiltPmgArtifact {
    Skeleton skeleton;
    ParametricMotionGraph graph;
    PmgArtifactMetadata metadata;
};

// Runtime transition window recorded by the offline edge builds. Runtime has
// one global transition window, so every edge build must use the same positive
// DistanceGridConfig::window_size. Legacy metadata with no edge records uses
// RuntimeControllerConfig's default.
RuntimeControllerConfig RuntimeControllerConfigFromArtifact(
    const BuiltPmgArtifact& artifact);

}  // namespace pmg
