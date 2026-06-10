#pragma once

#include "pmg/ParametricMotionGraph.h"
#include "pmg/PoseBlend.h"

namespace pmg {

struct Skeleton;

struct RuntimeControlRequest {
    int desired_node = -1;
    ParameterVector desired_parameter;
};

// 2D rigid transform from clip-local space to world: yaw about +Y (radians)
// then a floor-plane (x,z) translation. Accumulated across transitions so the
// streamed motion is continuous (no root pop, no facing jump).
struct WorldTransform2D {
    float yaw = 0.0f;
    float offset_x = 0.0f;
    float offset_z = 0.0f;
};

// Streams motion from a ParametricMotionGraph. On reaching a source transition
// phase it aligns and blends into the requested target clip, accumulating the
// world transform so the character keeps moving smoothly.
class RuntimeController {
public:
    explicit RuntimeController(const ParametricMotionGraph& graph);

    void Start(
        int node_index,
        const ParameterVector& initial_parameter,
        int generated_frame_count,
        float frames_per_second);

    void Update(float delta_seconds, const RuntimeControlRequest& request);

    // Paper-faithful runtime alignment (paper Sec 3.2 / Sec 5.2.1): when a
    // skeleton is supplied, each transition recomputes the exact point-cloud
    // alignment between the current and chosen target clips, rather than using
    // the (averaged) stored alignment. This is the paper's recommended path
    // ("we save storage space by computing the alignment transform ... at
    // runtime"). Takes precedence over SetUseStoredAlignment when set.
    void SetSkeleton(const Skeleton& skeleton) { skeleton_ = &skeleton; }

    // Fallback alignment when no skeleton is set. When true (default), use the
    // alignment stored on the edge sample (alignment_yaw/dx/dz). When false, use
    // a root-only alignment recomputed at runtime (legacy/debug path).
    void SetUseStoredAlignment(bool enabled) { use_stored_alignment_ = enabled; }

    Pose CurrentPose() const;   // world space
    int CurrentNode() const;
    float CurrentPhase() const; // normalized phase of the active clip
    bool IsTransitioning() const;
    int CompletedTransitions() const;
    const WorldTransform2D& WorldTransform() const;

private:
    float ClipPhase(const MotionClip& clip, float time_seconds) const;
    float ClampedClipPhase(const MotionClip& clip, float time_seconds) const;
    Pose SampleWorld(const MotionClip& clip, float time_seconds,
                     const WorldTransform2D& transform) const;
    Pose SampleWorldClamped(const MotionClip& clip, float time_seconds,
                            const WorldTransform2D& transform) const;
    void TryScheduleTransition(const RuntimeControlRequest& request);

    const ParametricMotionGraph& graph_;
    int generated_frame_count_ = 60;
    float frames_per_second_ = 30.0f;
    bool use_stored_alignment_ = true;
    const Skeleton* skeleton_ = nullptr;   // when set: runtime point-cloud align
    int alignment_window_ = 5;             // point-cloud window (frames)
    // Blend length as a fraction of the target clip's duration (phase-driven
    // window, replacing a fixed seconds value).
    float blend_window_phase_ = 0.20f;

    int current_node_ = -1;
    ParameterVector current_parameter_;
    MotionClip current_clip_;
    float current_time_seconds_ = 0.0f;
    WorldTransform2D world_transform_;

    bool transition_active_ = false;
    MotionClip next_clip_;
    int next_node_ = -1;
    ParameterVector next_parameter_;
    float next_time_seconds_ = 0.0f;
    WorldTransform2D next_world_transform_;
    float transition_elapsed_seconds_ = 0.0f;
    float transition_duration_seconds_ = 0.0f;
    int completed_transitions_ = 0;
};

}  // namespace pmg
