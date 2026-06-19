#include "pmg/RootCanonicalization.h"

#include "pmg/RigidTransform2D.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pmg {

namespace {

float WrapPi(float radians) {
    return radians - 2.0f * kPi * std::round(radians / (2.0f * kPi));
}

}  // namespace

float RootHeadingYaw(const Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const Vec3 forward = Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

MotionClip CanonicalizeRootOrigin(const MotionClip& clip) {
    clip.RequireNotEmpty("CanonicalizeRootOrigin");

    MotionClip canonical = clip;
    const Pose& first = clip.frames.front();
    const float start_x = first.root_position.x;
    const float start_z = first.root_position.z;
    const float start_yaw = RootHeadingYaw(first);

    RigidTransform2D inverse_start;
    inverse_start.yaw = -start_yaw;
    const Quaternion inverse_yaw =
        Quaternion::FromAxisAngle({0.0f, 1.0f, 0.0f}, -start_yaw);

    for (Pose& pose : canonical.frames) {
        const float shifted_x = pose.root_position.x - start_x;
        const float shifted_z = pose.root_position.z - start_z;
        float normalized_x = 0.0f;
        float normalized_z = 0.0f;
        inverse_start.RotateFloor(shifted_x, shifted_z, normalized_x, normalized_z);
        pose.root_position.x = normalized_x;
        pose.root_position.z = normalized_z;
        if (!pose.local_rotations.empty()) {
            pose.local_rotations.front() =
                (inverse_yaw * pose.local_rotations.front()).Normalized();
        }
    }
    return canonical;
}

RootStartSummary SummarizeRootStart(const MotionClip& clip) {
    clip.RequireNotEmpty("SummarizeRootStart");

    RootStartSummary summary;
    summary.first_root_position = clip.frames.front().root_position;
    summary.first_heading_yaw = RootHeadingYaw(clip.frames.front());
    summary.final_relative_displacement =
        clip.frames.back().root_position - clip.frames.front().root_position;
    summary.final_relative_heading = WrapPi(
        RootHeadingYaw(clip.frames.back()) - summary.first_heading_yaw);

    summary.normalized_bounds_min = {
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
    };
    summary.normalized_bounds_max = {
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
    };

    for (const Pose& pose : clip.frames) {
        summary.normalized_bounds_min.x =
            std::min(summary.normalized_bounds_min.x, pose.root_position.x);
        summary.normalized_bounds_min.y =
            std::min(summary.normalized_bounds_min.y, pose.root_position.y);
        summary.normalized_bounds_min.z =
            std::min(summary.normalized_bounds_min.z, pose.root_position.z);
        summary.normalized_bounds_max.x =
            std::max(summary.normalized_bounds_max.x, pose.root_position.x);
        summary.normalized_bounds_max.y =
            std::max(summary.normalized_bounds_max.y, pose.root_position.y);
        summary.normalized_bounds_max.z =
            std::max(summary.normalized_bounds_max.z, pose.root_position.z);
    }
    return summary;
}

}  // namespace pmg
