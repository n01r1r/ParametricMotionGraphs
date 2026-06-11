#include "pmg/FootLocking.h"

#include "pmg/ForwardKinematics.h"

#include <cassert>
#include <cmath>

namespace {

// Hip (root) -> knee -> ankle, each bone one unit long, hanging straight down.
pmg::Skeleton MakeLegSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint hip;
    hip.name = "hip";
    hip.parent_index = -1;
    pmg::Joint knee;
    knee.name = "knee";
    knee.parent_index = 0;
    knee.offset = {0.0f, -1.0f, 0.0f};
    pmg::Joint ankle;
    ankle.name = "ankle";
    ankle.parent_index = 1;
    ankle.offset = {0.0f, -1.0f, 0.0f};
    skeleton.joints = {hip, knee, ankle};
    return skeleton;
}

pmg::Pose MakePose(const pmg::Vec3& root_position) {
    pmg::Pose pose;
    pose.root_position = root_position;
    pose.local_rotations = {pmg::Quaternion::Identity(), pmg::Quaternion::Identity(),
                            pmg::Quaternion::Identity()};
    return pose;
}

bool NearVec(const pmg::Vec3& left, const pmg::Vec3& right, float tolerance) {
    return (left - right).Norm() < tolerance;
}

void TestIkReachesTarget() {
    const pmg::Skeleton skeleton = MakeLegSkeleton();
    pmg::Pose pose = MakePose({0.0f, 2.0f, 0.0f});

    const pmg::Vec3 target = {0.5f, 0.6f, 0.2f};  // reachable: |target - hip| < 2
    pmg::SolveTwoBoneIk(skeleton, pose, 2, target);

    const std::vector<pmg::Vec3> positions = pmg::ComputeJointWorldPositions(skeleton, pose);
    assert(NearVec(positions[2], target, 1.0e-3f));
    // Bone lengths must survive the solve (rotations only, no stretching).
    assert(std::abs((positions[1] - positions[0]).Norm() - 1.0f) < 1.0e-4f);
    assert(std::abs((positions[2] - positions[1]).Norm() - 1.0f) < 1.0e-4f);
    // The root translation is untouched.
    assert(NearVec(pose.root_position, {0.0f, 2.0f, 0.0f}, 1.0e-6f));
}

void TestIkClampsUnreachableTarget() {
    const pmg::Skeleton skeleton = MakeLegSkeleton();
    pmg::Pose pose = MakePose({0.0f, 2.0f, 0.0f});

    pmg::SolveTwoBoneIk(skeleton, pose, 2, {5.0f, 2.0f, 0.0f});

    const std::vector<pmg::Vec3> positions = pmg::ComputeJointWorldPositions(skeleton, pose);
    // Clamped to full extension toward the target.
    assert(std::abs((positions[2] - positions[0]).Norm() - 2.0f) < 1.0e-3f);
    assert(NearVec(positions[2], {2.0f, 2.0f, 0.0f}, 1.0e-2f));
}

void TestIkPreservesFootOrientationAndBendSide() {
    const pmg::Skeleton skeleton = MakeLegSkeleton();
    pmg::Pose pose = MakePose({0.0f, 2.0f, 0.0f});
    // Pre-bend the knee about +Z so the chain has an established bend plane.
    pose.local_rotations[1] = pmg::Quaternion::FromAxisAngle({0.0f, 0.0f, 1.0f}, 0.4f);

    const pmg::Quaternion ankle_world_before =
        pmg::ComputeForwardKinematics(skeleton, pose)[2].rotation;
    const std::vector<pmg::Vec3> before = pmg::ComputeJointWorldPositions(skeleton, pose);
    const pmg::Vec3 bend_normal_before =
        pmg::Cross(before[1] - before[0], before[2] - before[1]);

    pmg::SolveTwoBoneIk(skeleton, pose, 2, {0.3f, 0.4f, 0.0f});

    const std::vector<pmg::JointWorldState> world =
        pmg::ComputeForwardKinematics(skeleton, pose);
    assert(NearVec(world[2].position, {0.3f, 0.4f, 0.0f}, 1.0e-3f));
    // Foot world orientation unchanged (planted foot must not twist).
    const pmg::Quaternion difference = world[2].rotation * ankle_world_before.Conjugate();
    assert(std::abs(std::abs(difference.w) - 1.0f) < 1.0e-3f);
    // Knee keeps bending to the same side instead of snapping through.
    const pmg::Vec3 bend_normal_after =
        pmg::Cross(world[1].position - world[0].position,
                   world[2].position - world[1].position);
    assert(pmg::Dot(bend_normal_before, bend_normal_after) > 0.0f);
}

void TestLockFootContactsRemovesSlide() {
    const pmg::Skeleton skeleton = MakeLegSkeleton();

    // Root drifts 0.3 units in x while the ankle stays on the floor: the
    // synthetic version of blended-output stance slide. The knee carries a
    // constant bend so the chain has slack to absorb the drift (a fully
    // extended leg could only clamp toward an out-of-reach anchor).
    pmg::MotionClip clip;
    clip.name = "slide";
    clip.frames_per_second = 30.0f;
    for (int frame = 0; frame < 12; ++frame) {
        const float drift = 0.3f * static_cast<float>(frame) / 11.0f;
        pmg::Pose pose = MakePose({drift, 1.825f, 0.0f});
        pose.local_rotations[1] = pmg::Quaternion::FromAxisAngle({0.0f, 0.0f, 1.0f}, 0.6f);
        clip.frames.push_back(pose);
    }

    pmg::FootLockSettings settings;
    settings.contacts.height_threshold = 0.1f;
    settings.contacts.speed_threshold = 2.0f;
    settings.contacts.min_contact_frames = 3;

    pmg::LockFootContacts(skeleton, clip, {2}, settings);

    const pmg::Vec3 anchor = pmg::ComputeJointWorldPositions(skeleton, clip.frames[0])[2];
    for (int frame = 0; frame < clip.NumFrames(); ++frame) {
        const pmg::Vec3 ankle =
            pmg::ComputeJointWorldPositions(skeleton, clip.frames[frame])[2];
        assert(NearVec(ankle, anchor, 2.0e-3f));
        // Root translation (the clip's locomotion) is untouched by the lock.
        assert(std::abs(clip.frames[frame].root_position.x -
                        0.3f * static_cast<float>(frame) / 11.0f) < 1.0e-6f);
    }
}

}  // namespace

int main() {
    TestIkReachesTarget();
    TestIkClampsUnreachableTarget();
    TestIkPreservesFootOrientationAndBendSide();
    TestLockFootContactsRemovesSlide();
    return 0;
}
