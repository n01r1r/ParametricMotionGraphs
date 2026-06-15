#include "pmg/TransitionQuality.h"

#include <cassert>
#include <cmath>
#include <vector>

namespace {

constexpr float kFramesPerSecond = 30.0f;
constexpr int kTransitionPose = 3;

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.name = "root";
    root.parent_index = -1;
    pmg::Joint marker;
    marker.name = "marker";
    marker.parent_index = 0;
    marker.offset = {1.0f, 0.0f, 0.0f};
    pmg::Joint tip;
    tip.name = "tip";
    tip.parent_index = 1;
    tip.offset = {1.0f, 0.0f, 0.0f};
    skeleton.joints = {root, marker, tip};
    return skeleton;
}

pmg::Pose MakePose(float root_x, float yaw_radians, float marker_yaw_radians) {
    pmg::Pose pose;
    pose.root_position = {root_x, 0.0f, 0.0f};
    pose.local_rotations = {
        pmg::Quaternion::FromAxisAngle(
            {0.0f, 1.0f, 0.0f}, yaw_radians),
        pmg::Quaternion::FromAxisAngle(
            {0.0f, 1.0f, 0.0f}, marker_yaw_radians),
        pmg::Quaternion::Identity(),
    };
    return pose;
}

pmg::TransitionQualityContext Context(float baseline_step) {
    pmg::TransitionQualityContext context;
    context.frames_per_second = kFramesPerSecond;
    context.transition_distance = 12.5f;
    context.baseline_median_step = baseline_step;
    return context;
}

void TestPosePopClassification() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    std::vector<pmg::Pose> poses;
    for (int frame = 0; frame < 7; ++frame) {
        const float marker_yaw = frame == kTransitionPose ? 3.0f : 0.0f;
        poses.push_back(MakePose(
            0.1f * static_cast<float>(frame), 0.0f, marker_yaw));
    }

    pmg::TransitionQualityConfig config;
    config.contact_drift_threshold = 100.0f;
    const pmg::TransitionQualityRecord quality =
        pmg::MeasureTransitionQuality(
            skeleton, poses, kTransitionPose, Context(0.1f), config);

    assert(quality.transition_distance == 12.5f);
    assert(quality.local_pop_ratio > config.pose_pop_ratio_threshold);
    assert(std::abs(quality.root_speed_ratio - 1.0f) < 1.0e-5f);
    assert(
        quality.classification ==
        pmg::TransitionQualityClassification::kPosePop);
}

void TestRootSpeedJumpClassification() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const std::vector<float> root_x{
        0.0f, 0.1f, 0.2f, 0.3f, 0.5f, 0.7f, 0.9f};
    std::vector<pmg::Pose> poses;
    for (const float x : root_x) {
        poses.push_back(MakePose(x, 0.0f, 0.0f));
    }

    pmg::TransitionQualityConfig config;
    config.pose_pop_ratio_threshold = 3.0f;
    config.contact_drift_threshold = 100.0f;
    const pmg::TransitionQualityRecord quality =
        pmg::MeasureTransitionQuality(
            skeleton, poses, kTransitionPose, Context(0.1f), config);

    assert(quality.root_speed_ratio > 1.9f);
    assert(quality.yaw_rate_ratio == 1.0f);
    assert(
        quality.classification ==
        pmg::TransitionQualityClassification::kRootSpeedDiscontinuity);
}

void TestYawRateJumpClassification() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const std::vector<float> yaw{
        0.00f, 0.01f, 0.02f, 0.03f, 0.06f, 0.09f, 0.12f};
    std::vector<pmg::Pose> poses;
    for (int frame = 0; frame < 7; ++frame) {
        poses.push_back(MakePose(
            0.1f * static_cast<float>(frame), yaw[frame], 0.0f));
    }

    pmg::TransitionQualityConfig config;
    config.pose_pop_ratio_threshold = 10.0f;
    config.contact_drift_threshold = 100.0f;
    const pmg::TransitionQualityRecord quality =
        pmg::MeasureTransitionQuality(
            skeleton, poses, kTransitionPose, Context(0.1f), config);

    assert(quality.yaw_rate_ratio > 2.9f);
    assert(std::abs(quality.root_speed_ratio - 1.0f) < 1.0e-5f);
    assert(
        quality.classification ==
        pmg::TransitionQualityClassification::kYawRateDiscontinuity);
}

void TestYawRateReversalClassification() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const std::vector<float> yaw{
        0.00f, 0.01f, 0.02f, 0.03f, 0.02f, 0.01f, 0.00f};
    std::vector<pmg::Pose> poses;
    for (int frame = 0; frame < 7; ++frame) {
        poses.push_back(MakePose(
            0.1f * static_cast<float>(frame), yaw[frame], 0.0f));
    }

    pmg::TransitionQualityConfig config;
    config.pose_pop_ratio_threshold = 10.0f;
    config.contact_drift_threshold = 100.0f;
    const pmg::TransitionQualityRecord quality =
        pmg::MeasureTransitionQuality(
            skeleton, poses, kTransitionPose, Context(0.1f), config);

    assert(quality.pre_yaw_rate > 0.0f);
    assert(quality.post_yaw_rate < 0.0f);
    assert(quality.yaw_rate_ratio > 2.9f);
    assert(
        quality.classification ==
        pmg::TransitionQualityClassification::kYawRateDiscontinuity);
}

}  // namespace

int main() {
    TestPosePopClassification();
    TestRootSpeedJumpClassification();
    TestYawRateJumpClassification();
    TestYawRateReversalClassification();
    return 0;
}
