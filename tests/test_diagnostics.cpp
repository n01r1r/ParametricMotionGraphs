#include "pmg/Diagnostics.h"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace {

pmg::Pose MakePose(float root_x, float root_y, float root_z, float knee_yaw_degrees) {
    pmg::Pose pose;
    pose.root_position = {root_x, root_y, root_z};
    pose.local_rotations = {
        pmg::Quaternion::FromAxisAngle({0.0f, 1.0f, 0.0f}, 0.0f),
        pmg::Quaternion::FromAxisAngle({0.0f, 1.0f, 0.0f}, knee_yaw_degrees * 3.1415926535f / 180.0f),
        pmg::Quaternion::Identity(),
    };
    return pose;
}

pmg::BvhData MakeSyntheticBvh() {
    pmg::BvhData data;
    data.skeleton.joints = {
        {"Hips", -1, {0.0f, 0.0f, 0.0f},
         {pmg::BvhChannelType::XPosition, pmg::BvhChannelType::YPosition,
          pmg::BvhChannelType::ZPosition, pmg::BvhChannelType::XRotation,
          pmg::BvhChannelType::YRotation, pmg::BvhChannelType::ZRotation}},
        {"LeftAnkle", 0, {0.0f, -1.0f, 0.0f}, {pmg::BvhChannelType::XRotation}},
        {"LeftToe_End", 1, {0.0f, -0.5f, 0.0f}, {}},
    };
    data.clip.frames_per_second = 30.0f;
    data.clip.frames.push_back(MakePose(0.0f, 10.0f, 0.0f, 0.0f));
    data.clip.frames.push_back(MakePose(1.0f, 10.0f, 0.2f, 5.0f));
    return data;
}

void TestSkeletonInspectionWritesFiles() {
    const pmg::BvhData data = MakeSyntheticBvh();
    const pmg::SkeletonInspectionReport report = pmg::InspectSkeleton(data, "synthetic.bvh");

    assert(report.joint_count == 3);
    assert(report.end_site_count == 1);
    assert(report.likely_foot_joints.size() == 1);
    assert(report.joints[2].is_end_site);
    assert(!report.joints[1].is_end_site);

    const std::filesystem::path out_dir =
        std::filesystem::temp_directory_path() / "pmg_diagnostics_skeleton";
    std::filesystem::remove_all(out_dir);
    pmg::WriteSkeletonInspectionArtifacts(report, out_dir);
    assert(std::filesystem::exists(out_dir / "skeleton_report.md"));
    assert(std::filesystem::exists(out_dir / "skeleton_hierarchy.json"));
    assert(std::filesystem::exists(out_dir / "joint_positions_rest.csv"));
    assert(std::filesystem::exists(out_dir / "joint_positions_frame0.csv"));
}

void TestLoopAuditProducesFiniteMetrics() {
    const pmg::BvhData data = MakeSyntheticBvh();
    const pmg::LoopAuditReport report = pmg::AuditLoop(data, 0, 1);

    assert(std::isfinite(report.root_normalized_start_end_distance));
    assert(std::isfinite(report.root_velocity_discontinuity));
    assert(std::isfinite(report.root_yaw_discontinuity));
    assert(std::isfinite(report.cycle_score));
}

void TestTransitionPopAuditHandlesEmptyArtifact() {
    const pmg::BuiltPmgArtifact artifact;
    bool threw = false;
    try {
        (void)pmg::AuditTransitionPop(artifact, 3);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

}  // namespace

int main() {
    TestSkeletonInspectionWritesFiles();
    TestLoopAuditProducesFiniteMetrics();
    TestTransitionPopAuditHandlesEmptyArtifact();
    return 0;
}
