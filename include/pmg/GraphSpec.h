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
    bool has_parameter_metrics_config = false;
    // Empty: Shepard weights. Otherwise one measured metric per parameter
    // axis, in axis order (KG04-style parameter accuracy).
    std::vector<ParameterMetric> parameter_metrics;
    bool has_calibration_sampling_config = false;
    int calibration_samples_per_axis = 9;
    // Declared structural expectations, checked against the parsed examples by
    // LoadGraphSpec so a spec's claim ("this is a full 2-D space") fails loudly
    // instead of building a silently degenerate space. Unset = no claim.
    bool expect_corner_coverage_full = false;
    bool has_expect_spanned_axes = false;
    int expect_spanned_axes = 0;
    bool parameter_support_simplex = false;
};

struct GraphSpecExample {
    std::string node_name;
    ParameterVector parameter;
    std::string bvh_path;
};

struct GraphSpecEdge {
    std::string source_node;
    std::string target_node;
    // True when any per-edge build line (edge_config / edge_phase_range) was
    // seen, so the build uses build_config instead of the caller default.
    bool has_build_config = false;
    bool has_threshold_config = false;  // edge_config duplicate guard
    bool has_phase_range = false;       // edge_phase_range duplicate guard
    bool has_metric_type = false;       // edge_metric duplicate guard
    bool has_metric_config = false;     // edge_metric_config duplicate guard
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
//   parameter_metric <node> <turn_rate|travel_speed|none>  # legacy 1-D form
//   parameter_metrics <node> <metric0> ... <metricN-1>
//   parameter_calibration <node> <samples_per_axis>
//   parameter_support <node> simplex  # exactly N+1 affinely independent
//               examples define the authored support for an N-D node; runtime
//               lookup still uses the implementation's current AABB behavior
//   example <node_name> <p0> ... <pN-1> <path/to/file.bvh>
//   edge <source_node> <target_node>
//   edge_config <source> <target> <tgood> <tbad> <source_samples>
//               <target_samples> <seed>
//   edge_phase_range <source> <target> <src_start> <src_end>
//               <tgt_start> <tgt_end>   # transition-search sub-range (§6.3),
//               each in [0,1] with start < end; pairs with edge_config to also
//               set thresholds (otherwise default thresholds apply)
//   edge_metric <source> <target> <kovar_directional_point_cloud|dynamics_window>
//   edge_metric_config <source> <target>
//               <position_weight> <velocity_weight> <acceleration_weight>
//               <root_motion_weight> <foot_contact_weight>
//               <position_scale> <velocity_scale> <acceleration_scale>
//               <root_speed_scale> <yaw_rate_scale>
//               <root_displacement_weight> <root_speed_weight>
//               <root_yaw_rate_weight> <foot_mismatch_penalty>
//   expect <node> corner_coverage full  # every axis-extreme corner of the
//               node's parameter box must be sampled by an example (catches a
//               declared N-D space that is really a lower-dimensional simplex)
//   expect <node> spanned_axes <count>  # exactly <count> parameter axes must
//               take two or more distinct values across the node's examples
// Lines starting with '#' are ignored. Relative BVH paths are resolved against
// the spec file's directory by LoadGraphSpec(). Names and paths may not contain
// whitespace.
GraphSpec LoadGraphSpec(const std::string& path);

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
