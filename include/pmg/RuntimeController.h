#pragma once

#include "pmg/AlignmentStrategy.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/PoseBlend.h"
#include "pmg/RigidTransform2D.h"
#include "pmg/TransitionWindow.h"

#include <optional>

namespace pmg {

struct RuntimeControlRequest {
    int desired_node = -1;
    ParameterVector desired_parameter;
};

// How the runtime starts the target clip when a centered pre-roll would land
// before its phase-0 frame. See RuntimeControllerConfig::preroll_policy.
enum class TransitionPreRollPolicy {
    kClampAtClipStart,  // clamp the start to phase 0 (default, cross-node safe)
    kWrapCyclicClip,    // wrap into the previous cycle tail (cyclic self-edge)
};

struct RuntimeControllerConfig {
    // Number of sampled blend frames at the runtime sampling rate. k samples
    // span k-1 frame intervals. This must equal the DistanceGridConfig
    // window_size the edges were built with (callers wire it from the
    // artifact's edge build settings; the default matches that config).
    int transition_blend_frames = 5;
    // Runtime BLEND placement around the stored optimal transition point.
    // kPmgCentered (paper §5.2.1, "blending window centered at the optimal
    // transition point") gates half a window early so the optimal point gets
    // maximum blend weight; kKovarDirectional places the blend forward from the
    // point. This is independent of the metric window convention
    // (PmgBuilderConfig::transition_convention) -- PMG locates the transition
    // with Kovar's directional metric, then centers the blend on it.
    TransitionWindowConvention convention =
        TransitionWindowConvention::kPmgCentered;
    // What to do when a centered (or wide directional) pre-roll would start the
    // target clip before its phase-0 frame, i.e. target_phase*duration < lead.
    // kClampAtClipStart (default, behavior-preserving) clamps the start to 0:
    // safe when the target's pre-start frames are meaningless (a true cross-node
    // clip). kWrapCyclicClip lets the start go negative and folds it into the
    // previous cycle's tail (FoldCompletedCycles backward branch): correct for a
    // cyclic locomotion self-edge, where the tail is the same clip's end and the
    // clamp otherwise drops the target's pre-roll velocity support.
    TransitionPreRollPolicy preroll_policy =
        TransitionPreRollPolicy::kClampAtClipStart;
};

// Read-only snapshot of the transition currently being blended.
//
// Purpose: expose the PMG runtime chain to diagnostics/UI without duplicating
// transition lookup, target clamping, or alignment logic outside the runtime.
// All parameters use their node's parameter-space coordinates. Phases and
// blend_progress are normalized to [0, 1]. alignment maps target clip-local
// floor coordinates into source clip-local floor coordinates.
struct RuntimeTransitionDiagnostics {
    int source_node = -1;
    int target_node = -1;
    ParameterVector source_parameter;
    ParameterVector requested_target_parameter;
    ParameterVector actual_target_parameter;
    ParameterAabb reachable_target_box;
    float source_transition_phase = 0.0f;
    float target_transition_phase = 0.0f;
    TransitionWindowConvention transition_window_convention =
        TransitionWindowConvention::kKovarDirectional;
    TransitionFrameWindows runtime_windows;
    float metric_window_span_seconds = 0.0f;
    RigidTransform2D alignment;
    float blend_elapsed_seconds = 0.0f;
    float blend_duration_seconds = 0.0f;
    float blend_progress = 0.0f;
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

    // Clip lengths derive from each space's BlendedDurationSeconds at the
    // active parameter (paper timing); only the sampling rate is configured.
    void Start(
        int node_index,
        const ParameterVector& initial_parameter,
        float frames_per_second);

    void Update(float delta_seconds, const RuntimeControlRequest& request);

    Pose CurrentPose() const;   // world space
    int CurrentNode() const;
    const ParameterVector& CurrentParameter() const;
    float CurrentPhase() const; // normalized phase of the active clip
    bool IsTransitioning() const;
    int CompletedTransitions() const;
    const RigidTransform2D& WorldTransform() const;
    std::optional<RuntimeTransitionDiagnostics> ActiveTransitionDiagnostics() const;

private:
    float ClipPhase(const MotionClip& clip, float time_seconds) const;
    Pose SampleWorld(const MotionClip& clip, float time_seconds,
                     const RigidTransform2D& transform) const;
    void TryScheduleTransition(const RuntimeControlRequest& request);
    // Fold completed cycles into the placement: while `time_seconds` has
    // passed the clip's end, subtract one duration and compose the clip's
    // cycle delta into `transform`. Keeps looped playback continuous (the
    // raw clip-local root returns to its start every cycle).
    void FoldCompletedCycles(const MotionClip& clip, float& time_seconds,
                             RigidTransform2D& transform) const;

    const ParametricMotionGraph& graph_;
    const AlignmentStrategy& alignment_;
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
    std::optional<RuntimeTransitionDiagnostics> transition_diagnostics_;
    int completed_transitions_ = 0;
};

}  // namespace pmg
