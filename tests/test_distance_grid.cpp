#include "pmg/MotionClip.h"
#include "pmg/MotionDistance.h"
#include "pmg/Skeleton.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace {

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;

    pmg::Joint root;
    root.name = "root";
    root.parent_index = -1;
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

pmg::MotionClip MakeClip(float yaw_scale, float lift_scale) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;
    constexpr int kFrameCount = 12;
    for (int frame_index = 0; frame_index < kFrameCount; ++frame_index) {
        const float phase = static_cast<float>(frame_index) / static_cast<float>(kFrameCount - 1);
        pmg::Pose pose;
        pose.root_position = {0.4f * phase, 0.1f, 0.2f * phase};
        pose.local_rotations.push_back(pmg::EulerAxisRotation('Y', yaw_scale * phase));
        pose.local_rotations.push_back(pmg::EulerAxisRotation('X', lift_scale * phase));
        pose.local_rotations.push_back(pmg::EulerAxisRotation('Z', -6.0f * phase));
        clip.frames.push_back(pose);
    }
    return clip;
}

}  // namespace

int main() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip_a = MakeClip(30.0f, 12.0f);
    const pmg::MotionClip clip_b = MakeClip(-25.0f, 40.0f);

    pmg::DistanceGridConfig config;
    config.window_size = 3;

    // Self-grid: the diagonal (same source/target frame) must be ~0.
    {
        const pmg::DistanceGrid grid =
            pmg::MotionDistance::BuildDistanceGrid(skeleton, clip_a, clip_a, config);
        assert(grid.SourceCount() == clip_a.NumFrames());
        assert(grid.TargetCount() == clip_a.NumFrames());
        for (int i = 0; i < grid.SourceCount(); ++i) {
            assert(grid.At(i, i) < 1.0e-4f);
        }
    }

    // Min cell of two distinct clips matches an independent brute-force scan.
    {
        const pmg::OptimalTransition optimal =
            pmg::MotionDistance::FindOptimalTransition(skeleton, clip_a, clip_b, config);

        float brute_best = std::numeric_limits<float>::infinity();
        int brute_source = -1;
        int brute_target = -1;
        for (int sf = 0; sf < clip_a.NumFrames(); ++sf) {
            const pmg::PointCloud source_cloud =
                pmg::MotionDistance::BuildPointCloud(skeleton, clip_a, sf, config.window_size);
            for (int tf = 0; tf < clip_b.NumFrames(); ++tf) {
                const pmg::PointCloud target_cloud =
                    pmg::MotionDistance::BuildPointCloud(skeleton, clip_b, tf, config.window_size);
                const float distance =
                    pmg::MotionDistance::AlignedPointCloudDistance(source_cloud, target_cloud).distance;
                if (distance < brute_best) {
                    brute_best = distance;
                    brute_source = sf;
                    brute_target = tf;
                }
            }
        }

        assert(optimal.valid);
        assert(optimal.source_frame == brute_source);
        assert(optimal.target_frame == brute_target);
        assert(std::abs(optimal.distance - brute_best) < 1.0e-5f);

        // Normalized transition phases stay in [0, 1].
        assert(optimal.source_phase >= 0.0f && optimal.source_phase <= 1.0f);
        assert(optimal.target_phase >= 0.0f && optimal.target_phase <= 1.0f);
    }

    // Threshold gate: an impossibly tight bound rejects the transition.
    {
        const pmg::OptimalTransition gated = pmg::MotionDistance::FindOptimalTransition(
            skeleton, clip_a, clip_b, config, /*max_distance=*/-1.0f);
        assert(!gated.valid);
    }

    return 0;
}
