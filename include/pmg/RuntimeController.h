#pragma once

#include "pmg/AlignmentStrategy.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/PoseBlend.h"
#include "pmg/RigidTransform2D.h"
#include "pmg/TransitionWindow.h"

#include <optional>
#include <vector>

namespace pmg {

struct RuntimeControlRequest {
    int desired_node = -1;
    ParameterVector desired_parameter;
};

// How the runtime starts the target clip when a centered pre-roll would land
// before its phase-0 frame. See RuntimeControllerConfig::preroll_policy.
enum class TransitionPreRollPolicy {
    kClampAtClipStart,    // Clamp every target pre-roll at phase 0.
    kWrapCyclicClip,      // Wrap every target pre-roll; diagnostic/ablation mode.
    // Wrap only a self-edge (source==target) whose node is marked cyclic in
    // RuntimeControllerConfig::cyclic_nodes. A non-cyclic self-edge (e.g. a
    // punch or duck space with no cyclic boundary) clamps like a cross-node
    // transition, so cyclicity is metadata, not the self-edge topology.
    kWrapCyclicSelfEdges,
};

struct RuntimeControllerConfig {
    // Number of sampled blend frames at the runtime sampling rate. k samples
    // span k-1 frame intervals. This must equal the DistanceGridConfig
    // window_size the edges were built with.
    int transition_blend_frames = 5;

    // Runtime blend placement around the stored optimal transition point.
    // kPmgCentered gates half a window early so the optimal point receives
    // maximum blend weight. kKovarDirectional places the blend forward from
    // the stored point.
    TransitionWindowConvention convention =
        TransitionWindowConvention::kPmgCentered;

    // Default policy wraps only cyclic self-edges. Cross-node transitions remain
    // clamped because their pre-start target frames may not be meaningful.
    TransitionPreRollPolicy preroll_policy =
        TransitionPreRollPolicy::kWrapCyclicSelfEdges;

    // Per-node cyclicity, indexed by graph node index. A node is cyclic when its
    // motion space loops (locomotion), i.e. its phase-0 and phase-1 poses join;
    // this is derived from the node's registration cycle_joint in
    // RuntimeControllerConfigFromArtifact. Empty or out-of-range = not cyclic, so
    // kWrapCyclicSelfEdges only wraps nodes explicitly known to be cyclic.
    std::vector<bool> cyclic_nodes;

    // Continuous in-node parameter tracking for goto steering. When > 0, a
    // same-node parameter request is NOT routed through the phase-gated
    // self-edge transition (which re-aligns only ~once per cycle, so a steering
    // correction lands a full cycle late and the character orbits the target).
    // Instead current_parameter_ lerps toward the request by this fraction each
    // Update and the current clip is regenerated at the new parameter,
    // preserving phase -- a continuous re-blend with no splice and no
    // transition pop. 0 = use the gated self-edge (original behavior). Typical
    // 0.1-0.3; higher tracks faster but steps the pose harder per frame.
    // ponytail: regenerates the clip per Update while tracking; fine for the
    // single-node goto demo, revisit if a hot path tracks large blends.
    float same_node_tracking_rate = 0.0f;
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
    // kNN-interpolated build-time transition distance D for the scheduled edge
    // (proxy). Lets a streamed transition be correlated with the offline edge
    // quality it was chosen from. Zero for pre-V9 artifacts that stored no D.
    float interpolated_transition_distance = 0.0f;
    TransitionWindowConvention transition_window_convention =
        TransitionWindowConvention::kKovarDirectional;
    TransitionFrameWindows runtime_windows;
    float metric_window_span_seconds = 0.0f;

    RigidTransform2D alignment;

    float blend_elapsed_seconds = 0.0f;
    float blend_duration_seconds = 0.0f;
    float blend_progress = 0.0f;

    // Pre-roll diagnostics. These make the self-edge wrap policy observable in
    // tests and CLI diagnostics.
    bool target_preroll_wrapped = false;
    float raw_target_start_seconds = 0.0f;
    float scheduled_target_start_seconds = 0.0f;
};

// The runtime's clip-local -> world placement is a RigidTransform2D,
// accumulated across transitions so the streamed motion is continuous.
class RuntimeController {
public:
    // The alignment strategy is the seam for how transitions align the target
    // clip onto the current one. It must outlive the controller.
    RuntimeController(const ParametricMotionGraph& graph,
                      const AlignmentStrategy& alignment,
                      RuntimeControllerConfig config = {});

    // Clip lengths derive from each space's BlendedDurationSeconds at the
    // active parameter. Only the sampling rate is configured.
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
    std::optional<RuntimeTransitionDiagnostics>
    ActiveTransitionDiagnostics() const;

    // Live override of config_.same_node_tracking_rate. Lets goto steering turn
    // continuous in-node tracking on/off without reinstalling the controller
    // (which would reset playback). See
    // RuntimeControllerConfig::same_node_tracking_rate.
    void SetSameNodeTrackingRate(float rate) {
        config_.same_node_tracking_rate = rate;
    }

private:
    float ClipPhase(const MotionClip& clip, float time_seconds) const;
    Pose SampleWorld(const MotionClip& clip, float time_seconds,
                     const RigidTransform2D& transform) const;

    void TryScheduleTransition(const RuntimeControlRequest& request,
                               float previous_phase,
                               float phase_advance);

    // Continuous in-node parameter tracking. Lerps current_parameter_ toward
    // target_parameter by config_.same_node_tracking_rate and regenerates the
    // current clip at the new parameter, preserving phase. No transition, no
    // blend. See RuntimeControllerConfig::same_node_tracking_rate.
    void TrackSameNodeParameter(const ParameterVector& target_parameter);

    bool ShouldWrapTargetPreRoll(const PmgEdge& edge) const;

    // True when node_index is marked cyclic in config_.cyclic_nodes. Out-of-range
    // or unset entries are treated as non-cyclic.
    bool IsCyclicNode(int node_index) const;

    // Fold completed cycles into the placement. While time_seconds has passed
    // the clip end, subtract one duration and compose the clip's cycle delta
    // into transform. While time_seconds is negative, add one duration and
    // compose the inverse cycle delta. This supports cyclic pre-roll.
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