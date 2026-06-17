#include "pmg/RuntimeController.h"

#include "pmg/CyclicContinuity.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pmg {

namespace {

float NormalizeUnitPhase(float phase) {
    phase = std::fmod(phase, 1.0f);
    if (phase < 0.0f) {
        phase += 1.0f;
    }
    return phase;
}

float PhaseAdvance(const MotionClip& clip, float delta_seconds) {
    const float duration = clip.DurationSeconds();
    if (duration <= kSmallEpsilon) {
        return 0.0f;
    }
    return delta_seconds / duration;
}

// Returns true when forward playback from previous_phase by phase_advance
// crosses gate_phase. Unlike a point-in-window test, this remains robust when a
// frame drop or high playback speed jumps over the exact gate frame.
bool CrossedForwardPhase(float previous_phase,
                         float phase_advance,
                         float gate_phase) {
    previous_phase = NormalizeUnitPhase(previous_phase);
    gate_phase = NormalizeUnitPhase(gate_phase);

    if (phase_advance <= kSmallEpsilon) {
        return std::abs(gate_phase - previous_phase) <= kSmallEpsilon;
    }

    // If one update covers a full cycle or more, every phase was crossed.
    if (phase_advance >= 1.0f - kSmallEpsilon) {
        return true;
    }

    const float current_phase =
        NormalizeUnitPhase(previous_phase + phase_advance);

    if (previous_phase < current_phase) {
        return gate_phase + kSmallEpsilon >= previous_phase &&
               gate_phase <= current_phase + kSmallEpsilon;
    }

    // Wrapped across phase 1 -> 0.
    return gate_phase + kSmallEpsilon >= previous_phase ||
           gate_phase <= current_phase + kSmallEpsilon;
}

}  // namespace

RuntimeController::RuntimeController(const ParametricMotionGraph& graph,
                                     const AlignmentStrategy& alignment,
                                     RuntimeControllerConfig config)
    : graph_(graph), alignment_(alignment), config_(config) {
    if (config_.transition_blend_frames < 1) {
        throw std::runtime_error(
            "RuntimeController: transition_blend_frames must be at least 1");
    }
}

void RuntimeController::Start(
    int node_index,
    const ParameterVector& initial_parameter,
    float frames_per_second) {
    if (frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "RuntimeController::Start: frames_per_second must be positive");
    }

    const PmgNode& node = graph_.Node(node_index);
    if (static_cast<int>(initial_parameter.size()) !=
        node.motion_space.ParameterDimension()) {
        throw std::runtime_error(
            "RuntimeController::Start: initial parameter dimension mismatch");
    }

    current_node_ = node_index;
    current_parameter_ = initial_parameter;
    frames_per_second_ = frames_per_second;
    current_clip_ = node.motion_space.GenerateClip(
        initial_parameter,
        frames_per_second_);

    current_time_seconds_ = 0.0f;
    world_transform_ = RigidTransform2D{};

    transition_active_ = false;
    next_clip_ = MotionClip{};
    next_node_ = -1;
    next_parameter_.clear();
    next_time_seconds_ = 0.0f;
    next_world_transform_ = RigidTransform2D{};
    transition_elapsed_seconds_ = 0.0f;
    transition_duration_seconds_ = 0.0f;
    transition_diagnostics_.reset();

    completed_transitions_ = 0;
}

void RuntimeController::Update(float delta_seconds,
                               const RuntimeControlRequest& request) {
    if (current_node_ < 0) {
        throw std::runtime_error(
            "RuntimeController::Update: controller has not been started");
    }
    if (delta_seconds < 0.0f) {
        throw std::runtime_error(
            "RuntimeController::Update: delta_seconds must be non-negative");
    }

    const float previous_phase = CurrentPhase();
    const float phase_advance = PhaseAdvance(current_clip_, delta_seconds);

    current_time_seconds_ += delta_seconds;
    FoldCompletedCycles(
        current_clip_,
        current_time_seconds_,
        world_transform_);

    if (!transition_active_) {
        TryScheduleTransition(request, previous_phase, phase_advance);
        return;
    }

    next_time_seconds_ += delta_seconds;
    FoldCompletedCycles(
        next_clip_,
        next_time_seconds_,
        next_world_transform_);

    transition_elapsed_seconds_ += delta_seconds;

    if (transition_elapsed_seconds_ >= transition_duration_seconds_) {
        current_node_ = next_node_;
        current_parameter_ = next_parameter_;
        current_clip_ = next_clip_;
        current_time_seconds_ = next_time_seconds_;
        world_transform_ = next_world_transform_;

        transition_active_ = false;
        transition_diagnostics_.reset();
        ++completed_transitions_;

        FoldCompletedCycles(
            current_clip_,
            current_time_seconds_,
            world_transform_);
    }
}

void RuntimeController::FoldCompletedCycles(
    const MotionClip& clip,
    float& time_seconds,
    RigidTransform2D& transform) const {
    const float duration = clip.DurationSeconds();
    if (duration <= kSmallEpsilon) {
        return;
    }

    while (time_seconds >= duration) {
        time_seconds -= duration;
        transform =
            RigidTransform2D::Compose(transform, ComputeCycleDelta(clip));
    }

    while (time_seconds < 0.0f) {
        time_seconds += duration;
        transform = RigidTransform2D::Compose(
            transform, ComputeCycleDelta(clip).Inverse());
    }
}

float RuntimeController::ClipPhase(
    const MotionClip& clip,
    float time_seconds) const {
    const float duration = clip.DurationSeconds();
    if (duration <= kSmallEpsilon) {
        return 0.0f;
    }

    float phase = std::fmod(time_seconds, duration) / duration;
    if (phase < 0.0f) {
        phase += 1.0f;
    }
    return phase;
}

Pose RuntimeController::SampleWorld(
    const MotionClip& clip,
    float time_seconds,
    const RigidTransform2D& transform) const {
    const Pose local_pose =
        clip.SampleNormalizedPhase(ClipPhase(clip, time_seconds));
    return transform.Apply(local_pose);
}

Pose RuntimeController::CurrentPose() const {
    if (current_node_ < 0) {
        throw std::runtime_error(
            "RuntimeController::CurrentPose: controller has not been started");
    }

    if (!transition_active_) {
        return SampleWorld(
            current_clip_,
            current_time_seconds_,
            world_transform_);
    }

    const float linear_alpha =
        transition_duration_seconds_ <= kSmallEpsilon
            ? 1.0f
            : std::clamp(
                  transition_elapsed_seconds_ / transition_duration_seconds_,
                  0.0f,
                  1.0f);

    // C1-continuous cubic smoothstep blend weight. Zero slope at both endpoints
    // reduces the velocity discontinuity left by linear blend weights.
    const float alpha =
        linear_alpha * linear_alpha * (3.0f - 2.0f * linear_alpha);

    const Pose source_world =
        SampleWorld(
            current_clip_,
            current_time_seconds_,
            world_transform_);

    const Pose target_world =
        SampleWorld(
            next_clip_,
            next_time_seconds_,
            next_world_transform_);

    return BlendPose(source_world, target_world, alpha);
}

int RuntimeController::CurrentNode() const {
    return current_node_;
}

const ParameterVector& RuntimeController::CurrentParameter() const {
    if (current_node_ < 0) {
        throw std::runtime_error(
            "RuntimeController::CurrentParameter: controller has not been "
            "started");
    }
    return current_parameter_;
}

float RuntimeController::CurrentPhase() const {
    return ClipPhase(current_clip_, current_time_seconds_);
}

bool RuntimeController::IsTransitioning() const {
    return transition_active_;
}

int RuntimeController::CompletedTransitions() const {
    return completed_transitions_;
}

const RigidTransform2D& RuntimeController::WorldTransform() const {
    return world_transform_;
}

std::optional<RuntimeTransitionDiagnostics>
RuntimeController::ActiveTransitionDiagnostics() const {
    if (!transition_active_ || !transition_diagnostics_.has_value()) {
        return std::nullopt;
    }

    RuntimeTransitionDiagnostics diagnostics = *transition_diagnostics_;
    diagnostics.blend_elapsed_seconds = transition_elapsed_seconds_;
    diagnostics.blend_duration_seconds = transition_duration_seconds_;
    diagnostics.blend_progress =
        transition_duration_seconds_ <= kSmallEpsilon
            ? 1.0f
            : std::clamp(
                  transition_elapsed_seconds_ / transition_duration_seconds_,
                  0.0f,
                  1.0f);

    return diagnostics;
}

bool RuntimeController::IsCyclicNode(int node_index) const {
    if (node_index < 0 ||
        node_index >= static_cast<int>(config_.cyclic_nodes.size())) {
        return false;
    }
    return config_.cyclic_nodes[static_cast<std::size_t>(node_index)];
}

bool RuntimeController::ShouldWrapTargetPreRoll(const PmgEdge& edge) const {
    switch (config_.preroll_policy) {
        case TransitionPreRollPolicy::kClampAtClipStart:
            return false;

        case TransitionPreRollPolicy::kWrapCyclicClip:
            return true;

        case TransitionPreRollPolicy::kWrapCyclicSelfEdges:
            // A self-edge only pre-rolls into the previous cycle tail when the
            // node is genuinely cyclic; otherwise its pre-start frames are
            // undefined and we clamp like a cross-node transition.
            return edge.source_node == edge.target_node &&
                   IsCyclicNode(edge.target_node);
    }

    return false;
}

void RuntimeController::TryScheduleTransition(
    const RuntimeControlRequest& request,
    float previous_phase,
    float phase_advance) {
    if (request.desired_node < 0) {
        return;
    }

    for (const int edge_index : graph_.OutgoingEdgeIndices(current_node_)) {
        const PmgEdge& edge = graph_.Edge(edge_index);
        if (edge.target_node != request.desired_node) {
            continue;
        }

        const PmgNode& target_node = graph_.Node(edge.target_node);
        const int target_dimension =
            target_node.motion_space.ParameterDimension();

        if (static_cast<int>(request.desired_parameter.size()) !=
            target_dimension) {
            continue;
        }

        const std::optional<InterpolatedTransition> transition =
            edge.LookupInterpolated(
                current_parameter_,
                request.desired_parameter);

        if (!transition.has_value()) {
            continue;
        }

        const float blend_seconds = TransitionWindowSpanSeconds(
            config_.transition_blend_frames,
            frames_per_second_);

        const int half_window = config_.transition_blend_frames / 2;

        // First source blend frame relative to the stored reference:
        // - directional: stored reference is the first blend frame
        // - centered: stored reference is the window center
        const float source_first_offset_phase =
            config_.convention == TransitionWindowConvention::kPmgCentered
                ? static_cast<float>(half_window) /
                      static_cast<float>(
                          std::max(1, current_clip_.NumFrames() - 1))
                : 0.0f;

        const float gate_phase =
            transition->source_transition_phase - source_first_offset_phase;

        if (!CrossedForwardPhase(previous_phase, phase_advance, gate_phase)) {
            continue;
        }

        ParameterVector p = target_node.motion_space.ProjectToSupport(request.desired_parameter);
        p = transition->target_parameter_box.Clamp(p);
        p = target_node.motion_space.ProjectToSupport(p);

        // TODO: Exact projection to support ∩ reachable_box is not currently implemented.
        // The second projection guarantees the target is inside the simplex support (preventing
        // invalid extrapolations like the missing (1,1) corner), but it may push the parameter
        // slightly outside the AABB reachable box if the box boundary and simplex face are not aligned.
        const ParameterVector target_parameter = p;

        next_clip_ = target_node.motion_space.GenerateClip(
            target_parameter,
            frames_per_second_);
        next_node_ = edge.target_node;
        next_parameter_ = target_parameter;

        const float target_duration = next_clip_.DurationSeconds();
        const float target_lead_seconds =
            config_.convention == TransitionWindowConvention::kPmgCentered
                ? static_cast<float>(half_window) / frames_per_second_
                : blend_seconds;

        const float raw_target_start =
            transition->target_transition_phase * target_duration -
            target_lead_seconds;

        const bool wrap_target_preroll = ShouldWrapTargetPreRoll(edge);

        next_time_seconds_ =
            wrap_target_preroll
                ? raw_target_start
                : std::max(0.0f, raw_target_start);

        const float scheduled_target_start_seconds = next_time_seconds_;

        const int source_reference_frame = static_cast<int>(std::lround(
            transition->source_transition_phase *
            static_cast<float>(
                std::max(1, current_clip_.NumFrames() - 1))));

        const int target_reference_frame = static_cast<int>(std::lround(
            transition->target_transition_phase *
            static_cast<float>(
                std::max(1, next_clip_.NumFrames() - 1))));

        const TransitionFrameWindows runtime_windows =
            ResolveTransitionFrameWindows(
                current_clip_.NumFrames(),
                next_clip_.NumFrames(),
                source_reference_frame,
                target_reference_frame,
                config_.transition_blend_frames,
                config_.convention);

        const AlignmentContext alignment_context{
            current_clip_,
            next_clip_,
            CurrentPhase(),
            *transition,
            config_.transition_blend_frames,
            config_.convention};

        const RigidTransform2D alignment =
            alignment_.Resolve(alignment_context);

        next_world_transform_ =
            RigidTransform2D::Compose(world_transform_, alignment);

        // If next_time_seconds_ is negative and wrapping is enabled, this folds
        // the target into the previous cycle tail. If clamped, this is a no-op.
        FoldCompletedCycles(
            next_clip_,
            next_time_seconds_,
            next_world_transform_);

        transition_active_ = true;
        transition_elapsed_seconds_ = 0.0f;
        transition_duration_seconds_ = blend_seconds;

        RuntimeTransitionDiagnostics diagnostics;
        diagnostics.source_node = current_node_;
        diagnostics.target_node = next_node_;
        diagnostics.source_parameter = current_parameter_;
        diagnostics.requested_target_parameter = request.desired_parameter;
        diagnostics.actual_target_parameter = next_parameter_;
        diagnostics.reachable_target_box = transition->target_parameter_box;
        diagnostics.source_transition_phase =
            transition->source_transition_phase;
        diagnostics.target_transition_phase =
            transition->target_transition_phase;
        diagnostics.interpolated_transition_distance =
            transition->transition_distance;
        diagnostics.transition_window_convention = config_.convention;
        diagnostics.runtime_windows = runtime_windows;
        diagnostics.metric_window_span_seconds = TransitionWindowSpanSeconds(
            config_.transition_blend_frames,
            frames_per_second_);
        diagnostics.alignment = alignment;
        diagnostics.blend_elapsed_seconds = 0.0f;
        diagnostics.blend_duration_seconds = transition_duration_seconds_;
        diagnostics.blend_progress = 0.0f;
        diagnostics.target_preroll_wrapped =
            wrap_target_preroll && raw_target_start < 0.0f;
        diagnostics.raw_target_start_seconds = raw_target_start;
        diagnostics.scheduled_target_start_seconds =
            scheduled_target_start_seconds;

        transition_diagnostics_ = diagnostics;
        return;
    }
}

}  // namespace pmg
