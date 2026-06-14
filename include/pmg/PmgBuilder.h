#pragma once

#include "pmg/MotionDistance.h"
#include "pmg/ParametricMotionGraph.h"

#include <string>
#include <vector>

namespace pmg {

// Configuration for sampling-based edge construction (paper §3.2).
struct PmgBuilderConfig {
    int source_sample_count = 50;
    int target_sample_count = 1000;
    // Retained for artifact-format compatibility only: sampled clips derive
    // their frame count from BlendedDurationSeconds at the sampled parameter,
    // so playback timing matches the blended examples.
    int generated_frame_count = 48;
    float generated_frames_per_second = 30.0f;
    // Raw weighted squared-sum units. These defaults preserve the previous
    // 0.5/0.7 mean-squared scale for the repository's 31-joint, 5-frame
    // baseline; other skeleton/window/weighting choices require calibration.
    float good_transition_threshold = 80.0f;
    float bad_transition_threshold = 110.0f;
    unsigned int seed = 1234u;
    float box_shrink_epsilon = 1.0e-4f;
    // Include authored examples in addition to random samples. This is an
    // engineering safeguard: it does not change PMG edge semantics.
    bool include_example_parameters = true;
    DistanceGridConfig distance_grid{
        /*window_size=*/5,
        /*source_frame_stride=*/2,
        /*target_frame_stride=*/2,
        /*source_phase_start=*/0.70f,
        /*source_phase_end=*/0.95f,
        /*target_phase_start=*/0.05f,
        /*target_phase_end=*/0.30f,
        /*weighting=*/{},
    };
    // Window convention for the transition METRIC (paper §3.1). PMG reuses
    // Kovar's distance metric, whose windows are directional: source [i, i+k-1],
    // target [j-k+1, j]. This asymmetry aligns the end->start concatenation seam
    // and the repo's calibrated phase sub-ranges/thresholds depend on it, so the
    // metric stays directional. (PMG's "centered" is a runtime BLEND-placement
    // choice, separate from the metric; see RuntimeControllerConfig::convention.)
    TransitionWindowConvention transition_convention =
        TransitionWindowConvention::kKovarDirectional;
};

struct SourceSampleBuildReport {
    ParameterVector source_parameter;
    int good_count = 0;
    int neutral_count = 0;
    int bad_count = 0;
    float min_distance = std::numeric_limits<float>::infinity();
    float p25_distance = std::numeric_limits<float>::infinity();
    float median_distance = std::numeric_limits<float>::infinity();
    float max_distance = std::numeric_limits<float>::infinity();
    ParameterAabb target_box_before_shrink;
    ParameterAabb target_box_after_shrink;
    bool accepted = false;
    std::string reject_reason;
};

struct EdgeBuildReport {
    int source_node = -1;
    int target_node = -1;
    bool edge_created = false;
    std::string reject_reason;
    std::vector<SourceSampleBuildReport> source_reports;
};

struct EdgeBuildResult {
    PmgEdge edge;
    EdgeBuildReport report;
};

class PmgBuilder {
public:
    static PmgEdge BuildEdge(
        const Skeleton& skeleton,
        int source_node_index,
        int target_node_index,
        const ParametricMotionSpace& source_space,
        const ParametricMotionSpace& target_space,
        const PmgBuilderConfig& config);

    static EdgeBuildResult BuildEdgeWithReport(
        const Skeleton& skeleton,
        int source_node_index,
        int target_node_index,
        const ParametricMotionSpace& source_space,
        const ParametricMotionSpace& target_space,
        const PmgBuilderConfig& config);
};

}  // namespace pmg
