#include "pmg/FootLocking.h"

#include "pmg/ForwardKinematics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pmg {

namespace {

constexpr float kEpsilon = 1.0e-6f;

// Quaternion rotating unit vector `from` onto unit vector `to`.
Quaternion RotationBetween(const Vec3& from, const Vec3& to) {
    const float dot = std::clamp(Dot(from, to), -1.0f, 1.0f);
    Vec3 axis = Cross(from, to);
    if (axis.Norm() < kEpsilon) {
        if (dot > 0.0f) {
            return Quaternion::Identity();
        }
        // Opposite vectors: rotate half a turn about any perpendicular axis.
        axis = Cross(from, {1.0f, 0.0f, 0.0f});
        if (axis.Norm() < kEpsilon) {
            axis = Cross(from, {0.0f, 1.0f, 0.0f});
        }
    }
    return Quaternion::FromAxisAngle(axis, std::acos(dot));
}

Vec3 NormalizedOrZero(const Vec3& value) {
    const float norm = value.Norm();
    return norm < kEpsilon ? Vec3{} : value * (1.0f / norm);
}

// Interior angle at the middle joint of a two-bone chain with thigh/shin
// lengths `upper` / `lower` whose endpoints sit `reach` apart.
float InteriorAngle(float upper, float lower, float reach) {
    const float cosine = std::clamp(
        (upper * upper + lower * lower - reach * reach) / (2.0f * upper * lower),
        -1.0f, 1.0f);
    return std::acos(cosine);
}

}  // namespace

void SolveTwoBoneIk(
    const Skeleton& skeleton,
    Pose& pose,
    int end_joint,
    const Vec3& target) {
    skeleton.RequireValidJointIndex(end_joint, "SolveTwoBoneIk");
    const int mid_joint = skeleton.joints[end_joint].parent_index;
    if (mid_joint < 0) {
        throw std::runtime_error("SolveTwoBoneIk: end joint has no parent to bend");
    }
    const int base_joint = skeleton.joints[mid_joint].parent_index;
    if (base_joint < 0) {
        throw std::runtime_error("SolveTwoBoneIk: chain needs a grandparent joint");
    }

    std::vector<JointWorldState> world = ComputeForwardKinematics(skeleton, pose);
    const Vec3 base = world[base_joint].position;
    const Vec3 mid = world[mid_joint].position;
    const Vec3 end = world[end_joint].position;
    const Quaternion end_world_rotation = world[end_joint].rotation;

    const float upper_length = (mid - base).Norm();
    const float lower_length = (end - mid).Norm();
    if (upper_length < kEpsilon || lower_length < kEpsilon) {
        return;  // degenerate chain; nothing to solve
    }

    // Clamp the target into the chain's annulus of reach.
    Vec3 to_target = target - base;
    float reach = to_target.Norm();
    const float max_reach = upper_length + lower_length - 1.0e-4f;
    const float min_reach = std::abs(upper_length - lower_length) + 1.0e-4f;
    if (reach < kEpsilon) {
        return;  // target on top of the base; direction undefined
    }
    reach = std::clamp(reach, min_reach, max_reach);
    to_target = NormalizedOrZero(to_target) * reach;

    // Bend axis: the chain's current bend plane normal; for a straight chain
    // fall back to any axis perpendicular to the target direction.
    Vec3 bend_axis = Cross(mid - base, end - mid);
    if (bend_axis.Norm() < kEpsilon) {
        bend_axis = Cross(to_target, {0.0f, 1.0f, 0.0f});
        if (bend_axis.Norm() < kEpsilon) {
            bend_axis = Cross(to_target, {1.0f, 0.0f, 0.0f});
        }
    }
    bend_axis = NormalizedOrZero(bend_axis);

    // 1) Set the mid joint's interior angle for the clamped reach.
    const float current_interior =
        InteriorAngle(upper_length, lower_length, (end - base).Norm());
    const float desired_interior = InteriorAngle(upper_length, lower_length, reach);
    // Opening the interior angle (longer reach) rotates the lower bone away
    // from the upper bone about the bend axis.
    const Quaternion mid_delta =
        Quaternion::FromAxisAngle(bend_axis, current_interior - desired_interior);
    pose.local_rotations[mid_joint] =
        (world[base_joint].rotation.Conjugate() * (mid_delta * world[mid_joint].rotation))
            .Normalized();

    // 2) Swing the base joint so the end lands on the target direction.
    world = ComputeForwardKinematics(skeleton, pose);
    const Vec3 current_direction = NormalizedOrZero(world[end_joint].position - base);
    const Vec3 target_direction = NormalizedOrZero(to_target);
    if (current_direction.Norm() > 0.0f && target_direction.Norm() > 0.0f) {
        const Quaternion base_delta = RotationBetween(current_direction, target_direction);
        const Quaternion base_parent_world =
            skeleton.joints[base_joint].parent_index >= 0
                ? world[skeleton.joints[base_joint].parent_index].rotation
                : Quaternion::Identity();
        pose.local_rotations[base_joint] =
            (base_parent_world.Conjugate() * (base_delta * world[base_joint].rotation))
                .Normalized();
    }

    // 3) Keep the end joint's world orientation (planted foot must not twist).
    world = ComputeForwardKinematics(skeleton, pose);
    pose.local_rotations[end_joint] =
        (world[mid_joint].rotation.Conjugate() * end_world_rotation).Normalized();
}

void LockFootContacts(
    const Skeleton& skeleton,
    MotionClip& clip,
    const std::vector<int>& contact_joints,
    const FootLockSettings& settings) {
    if (clip.NumFrames() == 0) {
        return;
    }
    const std::vector<ContactInterval> intervals =
        DetectContacts(skeleton, clip, contact_joints, settings.contacts);

    for (const ContactInterval& interval : intervals) {
        const Vec3 anchor =
            ComputeJointWorldPositions(skeleton, clip.frames[interval.first_frame])
                [interval.joint_index];

        const int blend = std::max(settings.blend_frames, 0);
        const int first = std::max(interval.first_frame - blend, 0);
        const int last = std::min(interval.last_frame + blend, clip.NumFrames() - 1);
        for (int frame = first; frame <= last; ++frame) {
            float weight = 1.0f;
            if (frame < interval.first_frame) {
                weight = static_cast<float>(blend - (interval.first_frame - frame) + 1) /
                         static_cast<float>(blend + 1);
            } else if (frame > interval.last_frame) {
                weight = static_cast<float>(blend - (frame - interval.last_frame) + 1) /
                         static_cast<float>(blend + 1);
            }

            Pose& pose = clip.frames[frame];
            const Vec3 current =
                ComputeJointWorldPositions(skeleton, pose)[interval.joint_index];
            const Vec3 target = current + (anchor - current) * weight;
            SolveTwoBoneIk(skeleton, pose, interval.joint_index, target);
        }
    }
}

}  // namespace pmg
