#include "pmg/RuntimeController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pmg {

RuntimeController::RuntimeController(const ParametricMotionGraph& graph,
                                    const AlignmentStrategy& alignment,
                                    RuntimeControllerConfig config)
    : graph_(graph), alignment_(alignment), config_(config) {
    if (config_.blend_window_phase <= 0.0f || config_.blend_window_phase > 1.0f) {
        throw std::runtime_error(
            "RuntimeController: blend_window_phase must be in the interval (0, 1]");
    }
}

void RuntimeController::Start(
    int node_index,
    const ParameterVector& initial_parameter,
    float frames_per_second) {
    if (frames_per_second <= 0.0f) {
        throw std::runtime_error("RuntimeController::Start: frames_per_second must be positive");
    }

    const PmgNode& node = graph_.Node(node_index);
    if (static_cast<int>(initial_parameter.size()) != node.motion_space.ParameterDimension()) {
        throw std::runtime_error("RuntimeController::Start: initial parameter dimension mismatch");
    }

    current_node_ = node_index;
    current_parameter_ = initial_parameter;
    frames_per_second_ = frames_per_second;
    current_clip_ = node.motion_space.GenerateClip(
        initial_parameter, frames_per_second_);
    current_time_seconds_ = 0.0f;
    world_transform_ = RigidTransform2D{};
    transition_active_ = false;
    completed_transitions_ = 0;
}

void RuntimeController::Update(float delta_seconds, const RuntimeControlRequest& request) {
    if (current_node_ < 0) {
        throw std::runtime_error("RuntimeController::Update: controller has not been started");
    }
    if (delta_seconds < 0.0f) {
        throw std::runtime_error("RuntimeController::Update: delta_seconds must be non-negative");
    }

    current_time_seconds_ += delta_seconds;

    if (!transition_active_) {
        TryScheduleTransition(request);
        return;
    }

    next_time_seconds_ += delta_seconds;
    transition_elapsed_seconds_ += delta_seconds;
    if (transition_elapsed_seconds_ >= transition_duration_seconds_) {
        current_node_ = next_node_;
        current_parameter_ = next_parameter_;
        current_clip_ = next_clip_;
        current_time_seconds_ = next_time_seconds_;
        world_transform_ = next_world_transform_;
        transition_active_ = false;
        ++completed_transitions_;
    }
}

float RuntimeController::ClipPhase(const MotionClip& clip, float time_seconds) const {
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

float RuntimeController::ClampedClipPhase(const MotionClip& clip, float time_seconds) const {
    const float duration = clip.DurationSeconds();
    if (duration <= kSmallEpsilon) {
        return 0.0f;
    }
    return std::clamp(time_seconds / duration, 0.0f, 1.0f);
}

Pose RuntimeController::SampleWorld(const MotionClip& clip, float time_seconds,
                                    const RigidTransform2D& transform) const {
    const Pose local_pose = clip.SampleNormalizedPhase(ClipPhase(clip, time_seconds));
    return transform.Apply(local_pose);
}

Pose RuntimeController::SampleWorldClamped(const MotionClip& clip, float time_seconds,
                                           const RigidTransform2D& transform) const {
    const Pose local_pose = clip.SampleNormalizedPhase(ClampedClipPhase(clip, time_seconds));
    return transform.Apply(local_pose);
}

Pose RuntimeController::CurrentPose() const {
    if (current_node_ < 0) {
        throw std::runtime_error("RuntimeController::CurrentPose: controller has not been started");
    }

    if (!transition_active_) {
        return SampleWorld(current_clip_, current_time_seconds_, world_transform_);
    }

    const float linear_alpha =
        transition_duration_seconds_ <= kSmallEpsilon
            ? 1.0f
            : std::clamp(transition_elapsed_seconds_ / transition_duration_seconds_, 0.0f, 1.0f);
    // C1-continuous cubic (smoothstep) blend weight: zero slope at alpha 0 and 1
    // removes the velocity discontinuity a linear weight leaves at both ends
    // (MG-style transition curve, paper §runtime). Also softens the final step
    // into a completed transition so facing/root do not snap.
    const float alpha = linear_alpha * linear_alpha * (3.0f - 2.0f * linear_alpha);
    // During active transitions, clips are finite segments. Do not use the
    // looped phase sampler here: wrapping the source clip from phase 1 back to
    // phase 0 while alpha is between 0 and 1 creates a root pop.
    const Pose source_world =
        SampleWorldClamped(current_clip_, current_time_seconds_, world_transform_);
    const Pose target_world =
        SampleWorldClamped(next_clip_, next_time_seconds_, next_world_transform_);
    return BlendPose(source_world, target_world, alpha);
}

int RuntimeController::CurrentNode() const {
    return current_node_;
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

void RuntimeController::TryScheduleTransition(const RuntimeControlRequest& request) {
    if (request.desired_node < 0) {
        return;
    }

    const float phase = CurrentPhase();
    for (const int edge_index : graph_.OutgoingEdgeIndices(current_node_)) {
        const PmgEdge& edge = graph_.Edge(edge_index);
        if (edge.target_node != request.desired_node) {
            continue;
        }

        const std::optional<InterpolatedTransition> transition =
            edge.LookupInterpolated(current_parameter_);
        if (!transition.has_value()) {
            continue;
        }

        // Center the blend window on the optimal transition point (paper Sec 3.1
        // / Sec 5.2.1: "a short window centered at these frames"). The window
        // spans config_.blend_window_phase; gate half a window early so the window's
        // midpoint (alpha = 0.5, max weight) lands on the optimal point where the
        // two motions are most similar.
        const float half_window_phase = 0.5f * config_.blend_window_phase;
        if (phase < transition->source_transition_phase - half_window_phase) {
            continue;
        }
        if (phase > transition->source_transition_phase + half_window_phase) {
            continue;
        }

        const PmgNode& target_node = graph_.Node(edge.target_node);
        const int target_dimension = target_node.motion_space.ParameterDimension();
        if (static_cast<int>(request.desired_parameter.size()) != target_dimension) {
            continue;
        }

        const ParameterVector target_parameter =
            transition->target_parameter_box.Clamp(request.desired_parameter);

        next_clip_ = target_node.motion_space.GenerateClip(
            target_parameter, frames_per_second_);
        next_node_ = edge.target_node;
        next_parameter_ = target_parameter;
        // Start the target half a window before its transition point so the
        // window is centered on the optimal point (matches the source gate above).
        const float target_duration = next_clip_.DurationSeconds();
        const float half_window_target_seconds = half_window_phase * target_duration;
        next_time_seconds_ = std::max(
            0.0f, transition->target_transition_phase * target_duration - half_window_target_seconds);

        // Alignment maps the target clip onto the source clip (target->source:
        // yaw about +Y then floor translation), then composes with the source's
        // accumulated world transform. How it is resolved (paper path: point-cloud; debug path: root-only)
        // lives behind the AlignmentStrategy seam.
        const AlignmentContext alignment_context{
            current_clip_, next_clip_, CurrentPhase(), *transition};
        const RigidTransform2D alignment = alignment_.Resolve(alignment_context);
        next_world_transform_ = RigidTransform2D::Compose(world_transform_, alignment);

        transition_active_ = true;
        transition_elapsed_seconds_ = 0.0f;

        // Blend length. The source is sampled with a *clamped* phase during the
        // transition (it holds its final pose, never wraps), so the source's
        // remaining time must NOT shorten the blend. Bound only by the target's
        // remaining time from its (centered) start so it does not overrun its clip.
        const float target_remaining_seconds =
            std::max(kSmallEpsilon, target_duration - next_time_seconds_);
        const float requested_blend_seconds = config_.blend_window_phase * target_duration;
        transition_duration_seconds_ =
            std::max(kSmallEpsilon, std::min(requested_blend_seconds, target_remaining_seconds));
        return;
    }
}

}  // namespace pmg
