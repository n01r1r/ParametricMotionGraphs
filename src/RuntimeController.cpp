#include "pmg/RuntimeController.h"

#include "pmg/MotionDistance.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pmg {

namespace {

// Rotate a floor-plane vector by `yaw` about +Y (same convention as the
// Phase-A alignment: x' = cos*x + sin*z, z' = -sin*x + cos*z).
void RotateFloor(float yaw, float x, float z, float& out_x, float& out_z) {
    const float cos_yaw = std::cos(yaw);
    const float sin_yaw = std::sin(yaw);
    out_x = cos_yaw * x + sin_yaw * z;
    out_z = -sin_yaw * x + cos_yaw * z;
}

// Apply a clip-local pose into world space under `transform`.
Pose ApplyWorldTransform(const Pose& local_pose, const WorldTransform2D& transform) {
    Pose world_pose = local_pose;

    float rotated_x = 0.0f;
    float rotated_z = 0.0f;
    RotateFloor(transform.yaw, local_pose.root_position.x, local_pose.root_position.z,
                rotated_x, rotated_z);
    world_pose.root_position.x = rotated_x + transform.offset_x;
    world_pose.root_position.z = rotated_z + transform.offset_z;

    if (!world_pose.local_rotations.empty()) {
        const Quaternion world_yaw =
            Quaternion::FromAxisAngle({0.0f, 1.0f, 0.0f}, transform.yaw);
        world_pose.local_rotations[0] = world_yaw * world_pose.local_rotations[0];
    }
    return world_pose;
}

float RootFacingYaw(const Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }

    const Vec3 forward = Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

WorldTransform2D RootAlignmentFromTargetToSource(
    const Pose& source_pose,
    const Pose& target_pose) {
    WorldTransform2D alignment;
    alignment.yaw = RootFacingYaw(source_pose) - RootFacingYaw(target_pose);

    float rotated_target_x = 0.0f;
    float rotated_target_z = 0.0f;
    RotateFloor(alignment.yaw,
                target_pose.root_position.x,
                target_pose.root_position.z,
                rotated_target_x,
                rotated_target_z);
    alignment.offset_x = source_pose.root_position.x - rotated_target_x;
    alignment.offset_z = source_pose.root_position.z - rotated_target_z;
    return alignment;
}

// Compose transforms so that applying `inner` then `outer` equals the result:
// (outer o inner)(p) = R(outer.yaw) * (R(inner.yaw)*p + inner.t) + outer.t.
WorldTransform2D Compose(const WorldTransform2D& outer, const WorldTransform2D& inner) {
    WorldTransform2D composed;
    composed.yaw = outer.yaw + inner.yaw;
    float rotated_x = 0.0f;
    float rotated_z = 0.0f;
    RotateFloor(outer.yaw, inner.offset_x, inner.offset_z, rotated_x, rotated_z);
    composed.offset_x = rotated_x + outer.offset_x;
    composed.offset_z = rotated_z + outer.offset_z;
    return composed;
}

}  // namespace

RuntimeController::RuntimeController(const ParametricMotionGraph& graph)
    : graph_(graph) {}

void RuntimeController::Start(
    int node_index,
    const ParameterVector& initial_parameter,
    int generated_frame_count,
    float frames_per_second) {
    if (generated_frame_count <= 0) {
        throw std::runtime_error("RuntimeController::Start: generated_frame_count must be positive");
    }
    if (frames_per_second <= 0.0f) {
        throw std::runtime_error("RuntimeController::Start: frames_per_second must be positive");
    }

    const PmgNode& node = graph_.Node(node_index);
    if (static_cast<int>(initial_parameter.size()) != node.motion_space.ParameterDimension()) {
        throw std::runtime_error("RuntimeController::Start: initial parameter dimension mismatch");
    }

    current_node_ = node_index;
    current_parameter_ = initial_parameter;
    generated_frame_count_ = generated_frame_count;
    frames_per_second_ = frames_per_second;
    current_clip_ = node.motion_space.GenerateClip(
        initial_parameter, generated_frame_count_, frames_per_second_);
    current_time_seconds_ = 0.0f;
    world_transform_ = WorldTransform2D{};
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
                                    const WorldTransform2D& transform) const {
    const Pose local_pose = clip.SampleNormalizedPhase(ClipPhase(clip, time_seconds));
    return ApplyWorldTransform(local_pose, transform);
}

Pose RuntimeController::SampleWorldClamped(const MotionClip& clip, float time_seconds,
                                           const WorldTransform2D& transform) const {
    const Pose local_pose = clip.SampleNormalizedPhase(ClampedClipPhase(clip, time_seconds));
    return ApplyWorldTransform(local_pose, transform);
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

const WorldTransform2D& RuntimeController::WorldTransform() const {
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
        // spans blend_window_phase_; gate half a window early so the window's
        // midpoint (alpha = 0.5, max weight) lands on the optimal point where the
        // two motions are most similar.
        const float half_window_phase = 0.5f * blend_window_phase_;
        if (phase < transition->source_transition_phase - half_window_phase) {
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
            target_parameter, generated_frame_count_, frames_per_second_);
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
        // accumulated world transform.
        //   1. Paper-faithful (skeleton set): recompute the exact point-cloud
        //      alignment between the current and chosen target clips at the
        //      transition point (paper Sec 3.2 / Sec 5.2.1).
        //   2. Stored: the (averaged) alignment baked on the edge sample.
        //   3. Root-only: legacy recompute from the root pose alone.
        const float current_phase = CurrentPhase();

        WorldTransform2D alignment;
        if (skeleton_ != nullptr) {
            const int source_center = static_cast<int>(std::lround(
                transition->source_transition_phase *
                static_cast<float>(std::max(1, current_clip_.NumFrames() - 1))));
            const int target_center = static_cast<int>(std::lround(
                transition->target_transition_phase *
                static_cast<float>(std::max(1, next_clip_.NumFrames() - 1))));
            const PointCloud source_cloud = MotionDistance::BuildPointCloud(
                *skeleton_, current_clip_, source_center, alignment_window_);
            const PointCloud target_cloud = MotionDistance::BuildPointCloud(
                *skeleton_, next_clip_, target_center, alignment_window_);
            const RigidAlignment2D recovered =
                MotionDistance::AlignedPointCloudDistance(source_cloud, target_cloud).alignment;
            alignment.yaw = recovered.theta;
            alignment.offset_x = recovered.dx;
            alignment.offset_z = recovered.dz;
        } else if (use_stored_alignment_) {
            alignment.yaw = transition->alignment_yaw;
            alignment.offset_x = transition->alignment_dx;
            alignment.offset_z = transition->alignment_dz;
        } else {
            const Pose source_transition_pose =
                current_clip_.SampleNormalizedPhase(current_phase);
            const Pose target_transition_pose =
                next_clip_.SampleNormalizedPhase(transition->target_transition_phase);
            alignment =
                RootAlignmentFromTargetToSource(source_transition_pose, target_transition_pose);
        }
        next_world_transform_ = Compose(world_transform_, alignment);

        transition_active_ = true;
        transition_elapsed_seconds_ = 0.0f;

        // Blend length. The source is sampled with a *clamped* phase during the
        // transition (it holds its final pose, never wraps), so the source's
        // remaining time must NOT shorten the blend. Bound only by the target's
        // remaining time from its (centered) start so it does not overrun its clip.
        const float target_remaining_seconds =
            std::max(kSmallEpsilon, target_duration - next_time_seconds_);
        const float requested_blend_seconds = blend_window_phase_ * target_duration;
        transition_duration_seconds_ =
            std::max(kSmallEpsilon, std::min(requested_blend_seconds, target_remaining_seconds));
        return;
    }
}

}  // namespace pmg
