#include "pmg/MotionClip.h"
#include "pmg/MotionDistance.h"
#include "pmg/Skeleton.h"

#include <cassert>
#include <cmath>

namespace {

// Root + two off-axis children so world positions are sensitive to yaw.
pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;

    pmg::Joint root;
    root.name = "root";
    root.parent_index = -1;
    root.offset = {0.0f, 0.0f, 0.0f};
    skeleton.joints.push_back(root);

    pmg::Joint left;
    left.name = "left";
    left.parent_index = 0;
    left.offset = {1.0f, 0.0f, 0.0f};
    skeleton.joints.push_back(left);

    pmg::Joint front;
    front.name = "front";
    front.parent_index = 0;
    front.offset = {0.0f, 1.0f, 1.0f};
    skeleton.joints.push_back(front);

    return skeleton;
}

pmg::MotionClip MakeClip() {
    pmg::MotionClip clip;
    clip.name = "base";
    clip.frames_per_second = 30.0f;

    constexpr int kFrameCount = 8;
    for (int frame_index = 0; frame_index < kFrameCount; ++frame_index) {
        const float phase = static_cast<float>(frame_index) / static_cast<float>(kFrameCount - 1);
        pmg::Pose pose;
        pose.root_position = {0.5f * phase, 0.1f, -0.3f * phase};
        pose.local_rotations.push_back(pmg::EulerAxisRotation('Y', 15.0f * phase));
        pose.local_rotations.push_back(pmg::EulerAxisRotation('X', 10.0f * phase));
        pose.local_rotations.push_back(pmg::EulerAxisRotation('Z', -8.0f * phase));
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::MotionClip TranslateClip(pmg::MotionClip clip, float dx, float dz) {
    for (pmg::Pose& pose : clip.frames) {
        pose.root_position = pose.root_position + pmg::Vec3{dx, 0.0f, dz};
    }
    return clip;
}

// Rotate every world position about +Y through the origin by `theta` radians.
pmg::MotionClip RotateClipY(pmg::MotionClip clip, float theta) {
    const pmg::Quaternion yaw = pmg::Quaternion::FromAxisAngle({0.0f, 1.0f, 0.0f}, theta);
    for (pmg::Pose& pose : clip.frames) {
        pose.root_position = pmg::Rotate(yaw, pose.root_position);
        pose.local_rotations[0] = yaw * pose.local_rotations[0];
    }
    return clip;
}

pmg::PointCloud Cloud(const pmg::Skeleton& skeleton, const pmg::MotionClip& clip) {
    return pmg::MotionDistance::BuildPointCloud(skeleton, clip, clip.NumFrames() / 2, clip.NumFrames());
}

}  // namespace

int main() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip base = MakeClip();
    const pmg::PointCloud base_cloud = Cloud(skeleton, base);

    // Window spans the whole clip: one point per joint per frame.
    assert(static_cast<int>(base_cloud.Size()) == base.NumFrames() * skeleton.NumJoints());

    // Identical clip -> distance ~ 0, alignment ~ identity.
    {
        const pmg::AlignedDistanceResult result =
            pmg::MotionDistance::AlignedPointCloudDistance(base_cloud, base_cloud);
        assert(result.distance < 1.0e-4f);
        assert(std::abs(result.alignment.yaw) < 1.0e-3f);
        assert(std::abs(result.alignment.dx) < 1.0e-3f);
        assert(std::abs(result.alignment.dz) < 1.0e-3f);
    }

    // Translated clip -> distance ~ 0, recovered translation ~ (-dx, -dz).
    {
        const float dx = 2.0f;
        const float dz = -1.5f;
        const pmg::PointCloud moved = Cloud(skeleton, TranslateClip(base, dx, dz));
        const pmg::AlignedDistanceResult result =
            pmg::MotionDistance::AlignedPointCloudDistance(base_cloud, moved);
        assert(result.distance < 1.0e-3f);
        assert(std::abs(result.alignment.dx - (-dx)) < 1.0e-3f);
        assert(std::abs(result.alignment.dz - (-dz)) < 1.0e-3f);
        assert(std::abs(result.alignment.yaw) < 1.0e-3f);
    }

    // Rotated clip -> distance ~ 0, recovered yaw ~ -theta.
    {
        const float theta = 0.5f;
        const pmg::PointCloud rotated = Cloud(skeleton, RotateClipY(base, theta));
        const pmg::AlignedDistanceResult result =
            pmg::MotionDistance::AlignedPointCloudDistance(base_cloud, rotated);
        assert(result.distance < 1.0e-3f);
        assert(std::abs(result.alignment.yaw - (-theta)) < 1.0e-3f);
    }

    // Distinct poses -> distance > 0 and symmetric.
    {
        pmg::MotionClip other = MakeClip();
        for (pmg::Pose& pose : other.frames) {
            pose.root_position = pose.root_position + pmg::Vec3{0.0f, 0.7f, 0.0f};
            pose.local_rotations[1] = pmg::EulerAxisRotation('X', 55.0f);
        }
        const pmg::PointCloud other_cloud = Cloud(skeleton, other);
        const float ab = pmg::MotionDistance::AlignedPointCloudDistance(base_cloud, other_cloud).distance;
        const float ba = pmg::MotionDistance::AlignedPointCloudDistance(other_cloud, base_cloud).distance;
        assert(ab > 1.0e-3f);
        assert(std::abs(ab - ba) < 1.0e-3f);
    }

    // Equation 1 returns the raw weighted squared sum, not a weighted mean.
    {
        pmg::PointCloud cloud_a;
        cloud_a.points = {
            {0.0f, 0.0f, 0.0f},
            {1.0f, 0.0f, 0.0f},
        };
        cloud_a.weights = {2.0f, 3.0f};

        pmg::PointCloud cloud_b;
        cloud_b.points = {
            {0.0f, 1.0f, 0.0f},
            {1.0f, 2.0f, 0.0f},
        };

        const pmg::AlignedDistanceResult result =
            pmg::MotionDistance::AlignedPointCloudDistance(cloud_a, cloud_b);
        constexpr float kExpectedWeightedSquaredSum =
            2.0f * 1.0f * 1.0f + 3.0f * 2.0f * 2.0f;
        assert(std::abs(result.distance - kExpectedWeightedSquaredSum) < 1.0e-5f);
    }

    return 0;
}
