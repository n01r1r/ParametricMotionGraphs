#pragma once

#include "pmg/MathTypes.h"
#include "pmg/ContactDetection.h"
#include "pmg/MotionClip.h"
#include "pmg/RigidTransform2D.h"
#include "pmg/Skeleton.h"
#include "pmg/TransitionWindow.h"

#include <optional>
#include <vector>

namespace pmg {

// Point cloud sampled over a window of frames (Kovar et al. 2002 metric, paper
// §3.1). One point per joint per windowed frame; weights are per-point and
// default to uniform.
struct PointCloud {
    std::vector<Vec3> points;
    std::vector<float> weights;  // same length as points; uniform (1.0) by default

    std::size_t Size() const { return points.size(); }
};

// Optional weighting for BuildPointCloud (paper §3.1, A.3). Defaults reproduce
// the prior uniform behavior. per_joint_weights, when non-empty, must have one
// entry per skeleton joint (joint importance). add_velocity_weight scales each
// point by 1 + |finite-difference velocity| so faster-moving joints matter more.
struct PointCloudWeighting {
    std::vector<float> per_joint_weights;
    bool add_velocity_weight = false;
};

// Optimal 2D rigid alignment that maps cloud B onto cloud A (rotate B by yaw
// about +Y, then floor translate). Recovered in closed form (Kovar et al.
// 2002) as a RigidTransform2D.
struct AlignedDistanceResult {
    // Raw weighted squared world-space sum from Kovar et al. 2002 Eq. 1.
    // Magnitude scales with point count and configured weights.
    float distance = std::numeric_limits<float>::infinity();
    RigidTransform2D alignment;
};

// Configuration for a frame-pair distance grid (paper §3.1, Fig 3).
// Phase-range fields restrict the search to a sub-range of each clip and
// default to the full [0, 1] range, so the grid spans every frame pair unless
// narrowed. Strides subsample frames for speed (>= 1).
struct DistanceGridConfig {
    int window_size = 5;            // sampled frames; temporal span is k-1 intervals
    int source_frame_stride = 1;
    int target_frame_stride = 1;
    float source_phase_start = 0.0f;  // normalized [0,1], inclusive
    float source_phase_end = 1.0f;
    float target_phase_start = 0.0f;
    float target_phase_end = 1.0f;
    PointCloudWeighting weighting;
    // Ablation (default off = endpoint clamp). When true the windowed point
    // cloud wraps frame indices modulo the clip length instead of clamping at
    // the endpoints, so a cyclic self-edge metric sees the real previous/next
    // cycle frames rather than a repeated endpoint. Only valid for a single
    // cyclic clip (self-edge); leave false for cross-node clips.
    bool cyclic_wrap = false;
};

// Default-off transition-quality extension. Costs are RMS values divided by
// explicit native-unit scales before weighting; scales must be calibrated for
// each corpus/unit convention before thresholds are interpreted physically.
struct TransitionMetricConfig {
    float position_weight = 1.0f;
    float velocity_weight = 0.1f;
    float acceleration_weight = 0.01f;
    float root_motion_weight = 0.5f;
    float foot_contact_weight = 1.0f;

    float position_scale = 1.0f;
    float velocity_scale = 1.0f;
    float acceleration_scale = 1.0f;
    float root_speed_scale = 1.0f;
    float yaw_rate_scale = 1.0f;

    float root_displacement_weight = 1.0f;
    float root_speed_weight = 1.0f;
    float root_yaw_rate_weight = 1.0f;
    float foot_mismatch_penalty = 1.0f;

    std::vector<float> per_joint_weights;
    std::vector<int> contact_joint_indices;
    std::optional<ContactDetectionSettings> contact_settings;
};

struct TransitionMetricResult {
    float position_cost = 0.0f;
    float velocity_cost = 0.0f;
    float acceleration_cost = 0.0f;
    float root_cost = 0.0f;
    float foot_cost = 0.0f;
    float total_cost = 0.0f;

    float raw_position_sse = 0.0f;
    float raw_velocity_sse = 0.0f;
    float raw_acceleration_sse = 0.0f;

    int compared_frame_count = 0;
    int compared_joint_count = 0;
    int foot_mismatch_count = 0;
    int foot_comparison_count = 0;

    int source_first_frame = 0;
    int source_last_frame = 0;
    int target_first_frame = 0;
    int target_last_frame = 0;

    RigidTransform2D alignment;
};

// Aligned point-cloud distance for every sampled (source_frame, target_frame)
// pair, stored row-major. `alignments[i*TargetCount()+j]` maps the target frame
// cloud onto the source frame cloud (target aligned to source).
struct DistanceGrid {
    std::vector<int> source_frames;
    std::vector<int> target_frames;
    std::vector<float> distances;
    std::vector<RigidTransform2D> alignments;

    int SourceCount() const { return static_cast<int>(source_frames.size()); }
    int TargetCount() const { return static_cast<int>(target_frames.size()); }
    float At(int source_index, int target_index) const;
};

// Minimum-distance cell of a distance grid (the optimal transition point).
// `valid` is false when the grid is empty or no cell met the threshold gate.
struct OptimalTransition {
    bool valid = false;
    int source_frame = -1;
    int target_frame = -1;
    float distance = std::numeric_limits<float>::infinity();
    float source_phase = 0.0f;  // first source support frame, normalized [0,1]
    float target_phase = 0.0f;  // last target support frame, normalized [0,1]
    RigidTransform2D alignment;
};

class MotionDistance {
public:
    // Build a point cloud centered on `center_frame`, spanning `window_size`
    // frames (floor(window_size/2) on each side, clamped to the clip). Points
    // are joint world positions; weights follow `weighting`.
    static PointCloud BuildPointCloud(
        const Skeleton& skeleton,
        const MotionClip& clip,
        int center_frame,
        int window_size,
        const PointCloudWeighting& weighting = {});

    // Build a point cloud from an explicit first frame. Frame support is
    // [first_frame, first_frame + window_size - 1], endpoint-clamped.
    static PointCloud BuildPointCloudFromFirstFrame(
        const Skeleton& skeleton,
        const MotionClip& clip,
        int first_frame,
        int window_size,
        const PointCloudWeighting& weighting = {});

    // Closed-form optimal alignment + weighted distance. Aligns `cloud_b` onto
    // `cloud_a`. Both clouds must have equal point counts.
    static AlignedDistanceResult AlignedPointCloudDistance(
        const PointCloud& cloud_a,
        const PointCloud& cloud_b);

    // Aligned point-cloud distance over all sampled frame pairs of two clips.
    // For candidate (i, j), source samples [i, i+k-1] and target samples
    // [j-k+1, j], endpoint-clamped to preserve point correspondence.
    static DistanceGrid BuildDistanceGrid(
        const Skeleton& skeleton,
        const MotionClip& source_clip,
        const MotionClip& target_clip,
        const DistanceGridConfig& config = {});

    // Comparison path for transition-contract experiments. The default
    // BuildDistanceGrid remains Kovar directional for artifact compatibility.
    static DistanceGrid BuildDistanceGridForConvention(
        const Skeleton& skeleton,
        const MotionClip& source_clip,
        const MotionClip& target_clip,
        const DistanceGridConfig& config,
        TransitionWindowConvention convention);

    // Opt-in dynamics/contact extension over the same directional transition
    // windows. Alignment is estimated from positions; position uses the full
    // rigid transform, velocity/acceleration/root deltas use yaw rotation only.
    static TransitionMetricResult EvaluateDynamicsTransition(
        const Skeleton& skeleton,
        const MotionClip& source_clip,
        const MotionClip& target_clip,
        int source_frame,
        int target_frame,
        const DistanceGridConfig& grid_config,
        const TransitionMetricConfig& metric_config,
        TransitionWindowConvention convention =
            TransitionWindowConvention::kKovarDirectional);

    static DistanceGrid BuildDynamicsDistanceGridForConvention(
        const Skeleton& skeleton,
        const MotionClip& source_clip,
        const MotionClip& target_clip,
        const DistanceGridConfig& grid_config,
        const TransitionMetricConfig& metric_config,
        TransitionWindowConvention convention);

    // Minimum cell of the distance grid. The result is `valid` only when the
    // minimum distance is <= `max_distance` (default: accept any finite cell).
    static OptimalTransition FindOptimalTransition(
        const Skeleton& skeleton,
        const MotionClip& source_clip,
        const MotionClip& target_clip,
        const DistanceGridConfig& config = {},
        float max_distance = std::numeric_limits<float>::infinity());

    static OptimalTransition FindOptimalTransitionForConvention(
        const Skeleton& skeleton,
        const MotionClip& source_clip,
        const MotionClip& target_clip,
        const DistanceGridConfig& config,
        TransitionWindowConvention convention,
        float max_distance = std::numeric_limits<float>::infinity());

    static OptimalTransition FindOptimalDynamicsTransitionForConvention(
        const Skeleton& skeleton,
        const MotionClip& source_clip,
        const MotionClip& target_clip,
        const DistanceGridConfig& grid_config,
        const TransitionMetricConfig& metric_config,
        TransitionWindowConvention convention,
        float max_distance = std::numeric_limits<float>::infinity());
};

}  // namespace pmg
