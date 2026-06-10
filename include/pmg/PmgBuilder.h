#pragma once

#include "pmg/MotionDistance.h"
#include "pmg/ParametricMotionGraph.h"

namespace pmg {

// Configuration for sampling-based edge construction (paper §3.2).
//
// Units/contract:
//   source_sample_count / target_sample_count: number of random parameter
//     samples drawn from each space (paper uses ~50 source, ~1000 target).
//   good/bad_transition_threshold: aligned point-cloud distance gates. A target
//     sample is GOOD if D <= TGOOD and BAD if D >= TBAD (TBAD >= TGOOD).
//     Defaults are the paper's starting point (0.5 / 0.7). Now that BVH loads in
//     NATIVE units (no loader x10), the corpus distance scale matches the paper,
//     so these are directly usable. `pmg_cli --calibrate-thresholds BVH
//     locomotion_manifest.txt` suggests a tighter corpus fit (~0.05 / 0.58).
//   seed: RNG seed; identical seed + inputs yield identical edges.
//   box_shrink_epsilon: margin pushed past a BAD point when shrinking the box.
//   distance_grid: restricted transition-region search (late source phase ->
//     early target phase) used to score each candidate pair.
struct PmgBuilderConfig {
    int source_sample_count = 50;
    int target_sample_count = 1000;
    int generated_frame_count = 48;
    float generated_frames_per_second = 30.0f;
    float good_transition_threshold = 0.5f;
    float bad_transition_threshold = 0.7f;
    unsigned int seed = 1234u;
    float box_shrink_epsilon = 1.0e-4f;
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
};

}  // namespace pmg
