#pragma once

#include "pmg/AlignmentStrategy.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/PoseBlend.h"
#include "pmg/RigidTransform2D.h"

namespace pmg {

struct RuntimeControlRequest {
    int desired_node = -1;
    ParameterVector desired_parameter;
};

struct RuntimeControllerConfig {
    // Blend length as a fraction of the target clip duration. Paper-style PMG
    // transitions are windowed; exposing this makes phase/window calibration an
    // explicit runtime policy instead of a hidden constant.
    float blend_window_phase = 0.20f;
};

// The runtime's clip-local -> world placement is a RigidTransform2D, accumulated
// across transitions so the streamed motion is continuous (no root pop, no
// facing jump).

// Streams motion from a ParametricMotionGraph. On reaching a source transition
// phase it aligns and blends into the requested target clip, accumulating the
// world transform so the character keeps moving smoothly.
class RuntimeController {
public:
    // The alignment strategy is the seam for how transitions align the target
    // clip onto the current one (paper path: point-cloud; debug path: root-only). It must
    // outlive the controller.
    RuntimeController(const ParametricMotionGraph& graph,
                      const AlignmentStrategy& alignment,
                      RuntimeControllerConfig config = {});

    void Start(
        int node_index,
        const ParameterVector& initial_parameter,
        int generated_frame_count,
        float frames_per_second);

    void Update(float delta_seconds, const RuntimeControlRequest& request);

    Pose CurrentPose() const;   // world space
    int CurrentNode() const;
    float CurrentPhase() const; // normalized phase of the active clip
    bool IsTransitioning() const;
    int CompletedTransitions() const;
    const RigidTransform2D& WorldTransform() const;

private:
    float ClipPhase(const MotionClip& clip, float time_seconds) const;
    float ClampedClipPhase(const MotionClip& clip, float time_seconds) const;
    Pose SampleWorld(const MotionClip& clip, float time_seconds,
                     const RigidTransform2D& transform) const;
    Pose SampleWorldClamped(const MotionClip& clip, float time_seconds,
                            const RigidTransform2D& transform) const;
    void TryScheduleTransition(const RuntimeControlRequest& request);

    const ParametricMotionGraph& graph_;
    const AlignmentStrategy& alignment_;
    int generated_frame_count_ = 60;
    float frames_per_second_ = 30.0f;
    RuntimeControllerConfig config_;

    int current_node_ = -1;
    ParameterVector current_parameter_;
    MotionClip current_clip_;
    float current_time_seconds_ = 0.0f;
    RigidTransform2D world_transform_;

    bool transition_active_ = false;
    MotionClip next_clip_;
    int next_node_ = -1;
    ParameterVector next_parameter_;
    float next_time_seconds_ = 0.0f;
    RigidTransform2D next_world_transform_;
    float transition_elapsed_seconds_ = 0.0f;
    float transition_duration_seconds_ = 0.0f;
    int completed_transitions_ = 0;
};

}  // namespace pmg
