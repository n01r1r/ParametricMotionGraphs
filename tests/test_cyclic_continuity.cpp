#include "pmg/CyclicContinuity.h"

#include "pmg/ForwardKinematics.h"

#include <cassert>
#include <cmath>
#include <string>

namespace {

constexpr float kFramesPerSecond = 30.0f;

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
    tip.offset = {0.0f, 0.0f, 1.0f};
    skeleton.joints = {root, marker, tip};
    return skeleton;
}

pmg::Pose MakePose(
    float root_x,
    float root_yaw,
    float marker_yaw) {
    pmg::Pose pose;
    pose.root_position = {root_x, 0.0f, 0.0f};
    pose.local_rotations = {
        pmg::Quaternion::FromAxisAngle(
            {0.0f, 1.0f, 0.0f}, root_yaw),
        pmg::Quaternion::FromAxisAngle(
            {0.0f, 1.0f, 0.0f}, marker_yaw),
        pmg::Quaternion::Identity(),
    };
    return pose;
}

pmg::MotionClip MakeClip(std::initializer_list<pmg::Pose> poses) {
    pmg::MotionClip clip;
    clip.name = "synthetic_cycle";
    clip.frames_per_second = kFramesPerSecond;
    clip.frames.assign(poses.begin(), poses.end());
    return clip;
}

void TestPerfectSyntheticCycleIsStrong() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeClip({
        MakePose(0.0f, 0.0f, 0.0f),
        MakePose(0.0f, 0.0f, 0.2f),
        MakePose(0.0f, 0.0f, 0.4f),
        MakePose(0.0f, 0.0f, 0.2f),
        MakePose(0.0f, 0.0f, 0.0f),
    });

    const pmg::CyclicContinuityRecord record =
        pmg::MeasureCyclicContinuity(skeleton, clip);

    assert(record.classification ==
           pmg::CyclicContinuityClassification::kStrong);
    assert(record.seam_step < 1.0e-5f);
    assert(record.seam_step_ratio < 0.01f);
    assert(std::string(pmg::CyclicContinuityClassificationName(
               record.classification)) == "strong");
}

void TestPoseSeamDiscontinuityIsWeakPoseSeam() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeClip({
        MakePose(0.0f, 0.0f, 0.0f),
        MakePose(0.0f, 0.0f, 0.1f),
        MakePose(0.0f, 0.0f, 0.2f),
        MakePose(0.0f, 0.0f, 0.3f),
        MakePose(0.0f, 0.0f, 2.5f),
    });

    pmg::CyclicContinuityConfig config;
    config.root_speed_ratio_threshold = 100.0f;
    config.yaw_rate_ratio_threshold = 100.0f;
    const pmg::CyclicContinuityRecord record =
        pmg::MeasureCyclicContinuity(skeleton, clip, {}, config);

    assert(record.seam_step_ratio > config.pose_seam_ratio_threshold);
    assert(record.classification ==
           pmg::CyclicContinuityClassification::kWeakPoseSeam);
}

void TestRootSpeedDiscontinuityIsWeakRootSpeed() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeClip({
        MakePose(0.0f, 0.0f, 0.0f),
        MakePose(0.05f, 0.0f, 0.0f),
        MakePose(0.10f, 0.0f, 0.0f),
        MakePose(0.30f, 0.0f, 0.0f),
        MakePose(0.50f, 0.0f, 0.0f),
    });

    pmg::CyclicContinuityConfig config;
    config.pose_seam_ratio_threshold = 100.0f;
    config.yaw_rate_ratio_threshold = 100.0f;
    const pmg::CyclicContinuityRecord record =
        pmg::MeasureCyclicContinuity(skeleton, clip, {}, config);

    assert(record.seam_root_speed == 0.0f);
    assert(record.pre_root_speed > record.post_root_speed);
    assert(record.root_speed_ratio > config.root_speed_ratio_threshold);
    assert(record.classification ==
           pmg::CyclicContinuityClassification::kWeakRootSpeed);
}

void TestYawRateDiscontinuityIsWeakYawRate() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeClip({
        MakePose(0.0f, 0.00f, 0.0f),
        MakePose(0.0f, 0.01f, 0.0f),
        MakePose(0.0f, 0.02f, 0.0f),
        MakePose(0.0f, 0.12f, 0.0f),
        MakePose(0.0f, 0.22f, 0.0f),
    });

    pmg::CyclicContinuityConfig config;
    config.pose_seam_ratio_threshold = 100.0f;
    config.root_speed_ratio_threshold = 100.0f;
    const pmg::CyclicContinuityRecord record =
        pmg::MeasureCyclicContinuity(skeleton, clip, {}, config);

    assert(record.seam_yaw_rate == 0.0f);
    assert(record.pre_yaw_rate > record.post_yaw_rate);
    assert(record.yaw_rate_ratio > config.yaw_rate_ratio_threshold);
    assert(record.classification ==
           pmg::CyclicContinuityClassification::kWeakYawRate);
}

void TestZeroSeamRatesDoNotExplodeRatios() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeClip({
        MakePose(0.0f, 0.00f, 0.0f),
        MakePose(0.1f, 0.05f, 0.0f),
        MakePose(0.2f, 0.10f, 0.0f),
        MakePose(0.3f, 0.15f, 0.0f),
        MakePose(0.4f, 0.20f, 0.0f),
    });

    pmg::CyclicContinuityConfig config;
    config.pose_seam_ratio_threshold = 100.0f;
    const pmg::CyclicContinuityRecord record =
        pmg::MeasureCyclicContinuity(skeleton, clip, {}, config);

    assert(record.seam_root_speed == 0.0f);
    assert(record.seam_yaw_rate == 0.0f);
    assert(std::isfinite(record.root_speed_ratio));
    assert(std::isfinite(record.yaw_rate_ratio));
    assert(record.root_speed_ratio < 1.01f);
    assert(record.yaw_rate_ratio < 1.01f);
}

void TestPoseSeamPrecedesContactWeakness() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeClip({
        MakePose(0.0f, 0.0f, 0.0f),
        MakePose(0.0f, 0.0f, 0.1f),
        MakePose(0.0f, 0.0f, 0.2f),
        MakePose(0.0f, 0.0f, 0.3f),
        MakePose(0.0f, 0.0f, 2.5f),
    });

    pmg::ContactDetectionSettings contact_settings;
    contact_settings.height_threshold = 10.0f;
    contact_settings.speed_threshold = 100.0f;

    pmg::CyclicContinuityContext context;
    context.left_foot_joint = 2;
    context.contact_settings = contact_settings;

    pmg::CyclicContinuityConfig config;
    config.contact_drift_threshold = 0.001f;
    config.root_speed_ratio_threshold = 100.0f;
    config.yaw_rate_ratio_threshold = 100.0f;

    const pmg::CyclicContinuityRecord record =
        pmg::MeasureCyclicContinuity(skeleton, clip, context, config);

    assert(record.seam_step_ratio > config.pose_seam_ratio_threshold);
    assert(record.max_contact_drift > config.contact_drift_threshold);
    assert(record.classification ==
           pmg::CyclicContinuityClassification::kWeakPoseSeam);
}

void TestInsufficientDataDoesNotCrash() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeClip({
        MakePose(0.0f, 0.0f, 0.0f),
        MakePose(0.0f, 0.0f, 0.0f),
    });

    const pmg::CyclicContinuityRecord record =
        pmg::MeasureCyclicContinuity(skeleton, clip);

    assert(record.classification ==
           pmg::CyclicContinuityClassification::kInsufficientData);
}

void TestArtifactSummaryWarnsForWeakCyclicNode() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    pmg::ParametricMotionSpace space("walk", 1);
    space.AddExample({0.0f}, MakeClip({
                               MakePose(0.0f, 0.0f, 0.0f),
                               MakePose(0.0f, 0.0f, 0.1f),
                               MakePose(0.0f, 0.0f, 0.2f),
                               MakePose(0.0f, 0.0f, 0.3f),
                               MakePose(0.0f, 0.0f, 2.5f),
                           }));

    pmg::BuiltPmgArtifact artifact;
    artifact.skeleton = skeleton;
    artifact.graph.AddNode("walk", space);
    artifact.metadata.node_registrations.push_back(
        {"walk", "root", {}, 3, false});

    pmg::CyclicContinuityConfig config;
    config.root_speed_ratio_threshold = 100.0f;
    config.yaw_rate_ratio_threshold = 100.0f;
    const pmg::CyclicContinuityGraphSummary summary =
        pmg::SummarizeArtifactCyclicContinuity(artifact, config);
    const std::string warning =
        pmg::FormatCyclicContinuityWarning(summary);

    assert(summary.cyclic_sample_count == 1);
    assert(summary.weak_pose_seam_count == 1);
    assert(summary.strong_count == 0);
    assert(warning.find("Cyclic continuity warning") != std::string::npos);
    assert(warning.find("weak_pose_seam") != std::string::npos);
}

}  // namespace

int main() {
    TestPerfectSyntheticCycleIsStrong();
    TestPoseSeamDiscontinuityIsWeakPoseSeam();
    TestRootSpeedDiscontinuityIsWeakRootSpeed();
    TestYawRateDiscontinuityIsWeakYawRate();
    TestZeroSeamRatesDoNotExplodeRatios();
    TestPoseSeamPrecedesContactWeakness();
    TestInsufficientDataDoesNotCrash();
    TestArtifactSummaryWarnsForWeakCyclicNode();
    return 0;
}
