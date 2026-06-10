#include "pmg/AlignmentStrategy.h"

#include "pmg/MotionDistance.h"

#include <algorithm>
#include <cmath>

namespace pmg {

namespace {

float RootFacingYaw(const Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const Vec3 forward = Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

}  // namespace

RigidTransform2D PointCloudAlignment::Resolve(const AlignmentContext& context) const {
    const int source_center = static_cast<int>(std::lround(
        context.transition.source_transition_phase *
        static_cast<float>(std::max(1, context.source_clip.NumFrames() - 1))));
    const int target_center = static_cast<int>(std::lround(
        context.transition.target_transition_phase *
        static_cast<float>(std::max(1, context.target_clip.NumFrames() - 1))));

    const PointCloud source_cloud = MotionDistance::BuildPointCloud(
        *skeleton_, context.source_clip, source_center, window_);
    const PointCloud target_cloud = MotionDistance::BuildPointCloud(
        *skeleton_, context.target_clip, target_center, window_);

    return MotionDistance::AlignedPointCloudDistance(source_cloud, target_cloud).alignment;
}

RigidTransform2D RootOnlyAlignment::Resolve(const AlignmentContext& context) const {
    const Pose source_pose =
        context.source_clip.SampleNormalizedPhase(context.live_source_phase);
    const Pose target_pose =
        context.target_clip.SampleNormalizedPhase(context.transition.target_transition_phase);

    RigidTransform2D alignment;
    alignment.yaw = RootFacingYaw(source_pose) - RootFacingYaw(target_pose);

    float rotated_x = 0.0f;
    float rotated_z = 0.0f;
    alignment.RotateFloor(target_pose.root_position.x, target_pose.root_position.z,
                          rotated_x, rotated_z);
    alignment.dx = source_pose.root_position.x - rotated_x;
    alignment.dz = source_pose.root_position.z - rotated_z;
    return alignment;
}

}  // namespace pmg
