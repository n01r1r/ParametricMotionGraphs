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

pmg::MotionClip RaiseClip(pmg::MotionClip clip, float dy) {
    for (pmg::Pose& pose : clip.frames) {
        pose.root_position.y += dy;
    }
    return clip;
}

pmg::TransitionMetricConfig PositionVelocityMetricConfig() {
    pmg::TransitionMetricConfig config;
    config.position_weight = 1.0f;
    config.velocity_weight = 1.0f;
    config.acceleration_weight = 1.0f;
    config.root_motion_weight = 0.0f;
    config.foot_contact_weight = 0.0f;
    return config;
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

    // DistanceGridConfig::cyclic_wrap actually engages at a cycle boundary: a
    // window past the clip end repeats the last frame when clamped but pulls in
    // the wrapped start frames when cyclic, so the metric distance differs. This
    // backs the self-edge cyclic-metric ablation (a null effect on the corpus
    // would otherwise be indistinguishable from the flag never running).
    {
        pmg::DistanceGridConfig clamp_config;
        clamp_config.window_size = 4;
        clamp_config.source_phase_start = 1.0f;  // last source frame only
        clamp_config.source_phase_end = 1.0f;
        clamp_config.target_phase_start = 0.0f;  // first target frame only
        clamp_config.target_phase_end = 0.0f;
        pmg::DistanceGridConfig wrap_config = clamp_config;
        wrap_config.cyclic_wrap = true;

        const pmg::DistanceGrid clamped =
            pmg::MotionDistance::BuildDistanceGridForConvention(
                skeleton, base, base, clamp_config,
                pmg::TransitionWindowConvention::kKovarDirectional);
        const pmg::DistanceGrid wrapped =
            pmg::MotionDistance::BuildDistanceGridForConvention(
                skeleton, base, base, wrap_config,
                pmg::TransitionWindowConvention::kKovarDirectional);
        assert(clamped.SourceCount() == 1 && clamped.TargetCount() == 1);
        assert(std::abs(clamped.At(0, 0) - wrapped.At(0, 0)) > 1.0e-3f);
    }

    // Dynamics metric uses the same point/vector alignment contract as the
    // paper metric: translation affects positions only, not velocities or
    // accelerations.
    {
        pmg::DistanceGridConfig grid_config;
        grid_config.window_size = base.NumFrames();
        const pmg::TransitionMetricConfig metric_config =
            PositionVelocityMetricConfig();

        const pmg::TransitionMetricResult translated =
            pmg::MotionDistance::EvaluateDynamicsTransition(
                skeleton, base, TranslateClip(base, 12.0f, -5.0f),
                /*source_frame=*/0, /*target_frame=*/base.NumFrames() - 1,
                grid_config, metric_config);
        assert(translated.position_cost < 1.0e-3f);
        assert(translated.velocity_cost < 1.0e-3f);
        assert(translated.acceleration_cost < 1.0e-3f);

        const pmg::TransitionMetricResult yawed =
            pmg::MotionDistance::EvaluateDynamicsTransition(
                skeleton, base, RotateClipY(base, 0.35f),
                /*source_frame=*/0, /*target_frame=*/base.NumFrames() - 1,
                grid_config, metric_config);
        assert(yawed.position_cost < 1.0e-3f);
        assert(yawed.velocity_cost < 1.0e-3f);
    }

    // Dynamics metric reports the directional Kovar support explicitly:
    // source starts at i, target ends at j.
    {
        pmg::DistanceGridConfig grid_config;
        grid_config.window_size = 3;
        const pmg::TransitionMetricResult result =
            pmg::MotionDistance::EvaluateDynamicsTransition(
                skeleton, base, base, /*source_frame=*/2,
                /*target_frame=*/6, grid_config,
                PositionVelocityMetricConfig());
        assert(result.source_first_frame == 2);
        assert(result.source_last_frame == 4);
        assert(result.target_first_frame == 4);
        assert(result.target_last_frame == 6);
        assert(result.compared_frame_count == 3);
        assert(result.compared_joint_count == skeleton.NumJoints());
    }

    // Foot contacts are inactive unless explicit contact config and joints are
    // supplied; with explicit config, mismatched low/high foot states add cost.
    {
        pmg::DistanceGridConfig grid_config;
        grid_config.window_size = 3;
        pmg::TransitionMetricConfig no_contact_config =
            PositionVelocityMetricConfig();
        no_contact_config.foot_contact_weight = 1.0f;
        const pmg::TransitionMetricResult inactive =
            pmg::MotionDistance::EvaluateDynamicsTransition(
                skeleton, base, RaiseClip(base, 5.0f), /*source_frame=*/1,
                /*target_frame=*/3, grid_config, no_contact_config);
        assert(inactive.foot_comparison_count == 0);
        assert(inactive.foot_cost == 0.0f);

        pmg::TransitionMetricConfig contact_config = no_contact_config;
        contact_config.contact_joint_indices = {1};
        pmg::ContactDetectionSettings settings;
        settings.height_threshold = 1.0f;
        settings.speed_threshold = 1.0e6f;
        settings.min_contact_frames = 1;
        contact_config.contact_settings = settings;
        const pmg::TransitionMetricResult active =
            pmg::MotionDistance::EvaluateDynamicsTransition(
                skeleton, base, RaiseClip(base, 5.0f), /*source_frame=*/1,
                /*target_frame=*/3, grid_config, contact_config);
        assert(active.foot_comparison_count == grid_config.window_size);
        assert(active.foot_mismatch_count == grid_config.window_size);
        assert(active.foot_cost > 0.0f);
    }

    // Optimal dynamics transition carries the same scalar contract as the grid:
    // OptimalTransition::distance is the selected cell's total_cost.
    {
        pmg::DistanceGridConfig grid_config;
        grid_config.window_size = 3;
        grid_config.source_frame_stride = 2;
        grid_config.target_frame_stride = 2;
        const pmg::TransitionMetricConfig metric_config =
            PositionVelocityMetricConfig();
        const pmg::OptimalTransition best =
            pmg::MotionDistance::FindOptimalDynamicsTransitionForConvention(
                skeleton, base, TranslateClip(base, 0.2f, 0.1f),
                grid_config, metric_config,
                pmg::TransitionWindowConvention::kKovarDirectional);
        assert(best.valid);
        const pmg::TransitionMetricResult selected =
            pmg::MotionDistance::EvaluateDynamicsTransition(
                skeleton, base, TranslateClip(base, 0.2f, 0.1f),
                best.source_frame, best.target_frame, grid_config,
                metric_config,
                pmg::TransitionWindowConvention::kKovarDirectional);
        assert(std::abs(best.distance - selected.total_cost) < 1.0e-6f);
    }

    return 0;
}
