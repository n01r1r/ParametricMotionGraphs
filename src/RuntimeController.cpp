#include "pmg/RuntimeController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pmg {

namespace {

float PoseHeading(const Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const Vec3 forward = Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

float WrapPi(float angle_radians) {
    return angle_radians - 2.0f * kPi * std::round(angle_radians / (2.0f * kPi));
}

// Rigid floor transform that maps the clip's first-frame placement onto its
// last-frame placement: applying it to a pose at phase p yields the pose one
// cycle later in clip-local coordinates.
RigidTransform2D CycleDelta(const MotionClip& clip) {
    const Pose& first = clip.frames.front();
    const Pose& last = clip.frames.back();

    RigidTransform2D delta;
    delta.yaw = WrapPi(PoseHeading(last) - PoseHeading(first));
    float rotated_x = 0.0f;
    float rotated_z = 0.0f;
    delta.RotateFloor(first.root_position.x, first.root_position.z,
                      rotated_x, rotated_z);
    delta.dx = last.root_position.x - rotated_x;
    delta.dz = last.root_position.z - rotated_z;
    return delta;
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
    transition_diagnostics_.reset();
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
    // Looping is explicit: completed cycles fold into the world placement so
    // the clip-local wrap never teleports the root. During a transition this
    // keeps the source playing real frames past its end (the paper plays both
    // motions through the whole window; the source never freezes).
    FoldCompletedCycles(current_clip_, current_time_seconds_, world_transform_);

    if (!transition_active_) {
        TryScheduleTransition(request);
        return;
    }

    next_time_seconds_ += delta_seconds;
    FoldCompletedCycles(next_clip_, next_time_seconds_, next_world_transform_);
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
        FoldCompletedCycles(current_clip_, current_time_seconds_, world_transform_);
    }
}

void RuntimeController::FoldCompletedCycles(const MotionClip& clip,
                                            float& time_seconds,
                                            RigidTransform2D& transform) const {
    const float duration = clip.DurationSeconds();
    if (duration <= kSmallEpsilon) {
        return;
    }
    while (time_seconds >= duration) {
        time_seconds -= duration;
        transform = RigidTransform2D::Compose(transform, CycleDelta(clip));
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

Pose RuntimeController::SampleWorld(const MotionClip& clip, float time_seconds,
                                    const RigidTransform2D& transform) const {
    const Pose local_pose = clip.SampleNormalizedPhase(ClipPhase(clip, time_seconds));
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
    // Both clips keep playing through the whole blend. Update() folds completed
    // cycles into each world transform, so both times stay inside their clips
    // without freezing at an endpoint.
    const Pose source_world =
        SampleWorld(current_clip_, current_time_seconds_, world_transform_);
    const Pose target_world =
        SampleWorld(next_clip_, next_time_seconds_, next_world_transform_);
    return BlendPose(source_world, target_world, alpha);
}

int RuntimeController::CurrentNode() const {
    return current_node_;
}

const ParameterVector& RuntimeController::CurrentParameter() const {
    if (current_node_ < 0) {
        throw std::runtime_error(
            "RuntimeController::CurrentParameter: controller has not been started");
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
                  0.0f, 1.0f);
    return diagnostics;
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

        const PmgNode& target_node = graph_.Node(edge.target_node);
        const int target_dimension =
            target_node.motion_space.ParameterDimension();
        if (static_cast<int>(request.desired_parameter.size()) !=
            target_dimension) {
            continue;
        }

        const std::optional<InterpolatedTransition> transition =
            edge.LookupInterpolated(
                current_parameter_, request.desired_parameter);
        if (!transition.has_value()) {
            continue;
        }

        // The blend spans the same temporal window the distance metric scored
        // (transition_blend_frames = the edges' DistanceGridConfig
        // window_size), centered on the optimal transition point (paper Sec
        // 3.1 / Sec 5.2.1: "a short window centered at these frames"). Gate
        // half a window early so the window's midpoint (alpha = 0.5, max
        // weight) lands on the optimal point where the two motions are most
        // similar.
        const float blend_seconds =
            static_cast<float>(config_.transition_blend_frames) / frames_per_second_;
        const float source_duration =
            std::max(kSmallEpsilon, current_clip_.DurationSeconds());
        const float half_window_phase = 0.5f * blend_seconds / source_duration;
        if (phase < transition->source_transition_phase - half_window_phase) {
            continue;
        }
        if (phase > transition->source_transition_phase + half_window_phase) {
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
        next_time_seconds_ = std::max(
            0.0f, transition->target_transition_phase * target_duration -
                      0.5f * blend_seconds);

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

        // Blend length exactly equals the metric window. Both clips loop
        // continuously through the window when it crosses a cycle boundary.
        transition_duration_seconds_ = blend_seconds;
        transition_diagnostics_ = RuntimeTransitionDiagnostics{
            current_node_,
            next_node_,
            current_parameter_,
            request.desired_parameter,
            next_parameter_,
            transition->target_parameter_box,
            transition->source_transition_phase,
            transition->target_transition_phase,
            alignment,
            0.0f,
            transition_duration_seconds_,
            0.0f,
        };
        return;
    }
}

}  // namespace pmg
