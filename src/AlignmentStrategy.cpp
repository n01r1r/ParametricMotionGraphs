#include "pmg/AlignmentStrategy.h"

#include "pmg/MotionDistance.h"
#include "pmg/TransitionWindow.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

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
    if (context.window_size != window_) {
        throw std::runtime_error(
            "PointCloudAlignment::Resolve: context window_size does not match "
            "the alignment strategy window");
    }

    const int source_reference = static_cast<int>(std::lround(
        context.transition.source_transition_phase *
        static_cast<float>(std::max(1, context.source_clip.NumFrames() - 1))));
    const int target_reference = static_cast<int>(std::lround(
        context.transition.target_transition_phase *
        static_cast<float>(std::max(1, context.target_clip.NumFrames() - 1))));

    const TransitionFrameWindows windows =
        ResolveTransitionFrameWindows(
            context.source_clip.NumFrames(),
            context.target_clip.NumFrames(),
            source_reference, target_reference, context.window_size,
            context.convention);
    const PointCloud source_cloud =
        MotionDistance::BuildPointCloudFromFirstFrame(
            *skeleton_, context.source_clip,
            windows.source.first_unclamped_frame, context.window_size);
    const PointCloud target_cloud =
        MotionDistance::BuildPointCloudFromFirstFrame(
            *skeleton_, context.target_clip,
            windows.target.first_unclamped_frame, context.window_size);

    return MotionDistance::AlignedPointCloudDistance(source_cloud, target_cloud).alignment;
}

RigidTransform2D RootOnlyAlignment::Resolve(const AlignmentContext& context) const {
    const int source_reference = static_cast<int>(std::lround(
        context.transition.source_transition_phase *
        static_cast<float>(std::max(1, context.source_clip.NumFrames() - 1))));
    const int target_reference = static_cast<int>(std::lround(
        context.transition.target_transition_phase *
        static_cast<float>(std::max(1, context.target_clip.NumFrames() - 1))));
    const TransitionFrameWindows windows =
        ResolveTransitionFrameWindows(
            context.source_clip.NumFrames(),
            context.target_clip.NumFrames(),
            source_reference, target_reference,
            context.window_size, context.convention);
    const Pose source_pose =
        context.source_clip.SampleNormalizedPhase(context.live_source_phase);
    const Pose target_pose = context.target_clip.frames[
        windows.target.sampled_frames.front()];

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
