#include "pmg/MotionClip.h"
#include "pmg/MotionDistance.h"
#include "pmg/Skeleton.h"

#include <algorithm>
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

pmg::PointCloud BuildDirectionalCloud(
    const pmg::Skeleton& skeleton,
    const pmg::MotionClip& clip,
    int candidate_frame,
    int window_size,
    bool source_window) {
    pmg::PointCloud cloud;
    for (int offset = 0; offset < window_size; ++offset) {
        const int unclamped_frame =
            source_window
                ? candidate_frame + offset
                : candidate_frame - window_size + 1 + offset;
        const int frame = std::clamp(
            unclamped_frame, 0, clip.NumFrames() - 1);
        const pmg::PointCloud frame_cloud =
            pmg::MotionDistance::BuildPointCloud(
                skeleton, clip, frame, /*window_size=*/1);
        cloud.points.insert(
            cloud.points.end(),
            frame_cloud.points.begin(),
            frame_cloud.points.end());
        cloud.weights.insert(
            cloud.weights.end(),
            frame_cloud.weights.begin(),
            frame_cloud.weights.end());
    }
    return cloud;
}

}  // namespace

int main() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip_a = MakeClip(30.0f, 12.0f);
    const pmg::MotionClip clip_b = MakeClip(-25.0f, 40.0f);

    pmg::DistanceGridConfig config;
    config.window_size = 3;

    // Kovar's source window starts at i and target window ends at j. Therefore
    // (i, i + k - 1) compares the same ordered frames and must be ~0.
    {
        const pmg::DistanceGrid grid =
            pmg::MotionDistance::BuildDistanceGrid(skeleton, clip_a, clip_a, config);
        assert(grid.SourceCount() == clip_a.NumFrames());
        assert(grid.TargetCount() == clip_a.NumFrames());
        for (int source_frame = 0;
             source_frame + config.window_size - 1 < clip_a.NumFrames();
             ++source_frame) {
            const int target_frame =
                source_frame + config.window_size - 1;
            assert(grid.At(source_frame, target_frame) < 1.0e-4f);
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
                BuildDirectionalCloud(
                    skeleton, clip_a, sf, config.window_size,
                    /*source_window=*/true);
            for (int tf = 0; tf < clip_b.NumFrames(); ++tf) {
                const pmg::PointCloud target_cloud =
                    BuildDirectionalCloud(
                        skeleton, clip_b, tf, config.window_size,
                        /*source_window=*/false);
                const float distance =
                    pmg::MotionDistance::AlignedPointCloudDistance(
                        source_cloud, target_cloud).distance;
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
