#include "pmg/PoseBlend.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace {

float AbsQuaternionDot(const pmg::Quaternion& a, const pmg::Quaternion& b) {
    return std::abs(pmg::QuaternionDot(a, b));
}

pmg::Pose MakeSingleJointPose(float root_x, const pmg::Quaternion& rotation) {
    pmg::Pose pose;
    pose.root_position = {root_x, 0.0f, 0.0f};
    pose.local_rotations.push_back(rotation);
    return pose;
}

}  // namespace

int main() {
    pmg::Pose first;
    first.root_position = {0.0f, 0.0f, 0.0f};
    first.local_rotations.push_back(pmg::Quaternion::Identity());

    pmg::Pose second;
    second.root_position = {10.0f, 0.0f, 0.0f};
    second.local_rotations.push_back(pmg::EulerAxisRotation('Y', 90.0f));

    const pmg::Pose blended = pmg::BlendPose(first, second, 0.25f);

    assert(std::abs(blended.root_position.x - 2.5f) < 1.0e-5f);
    assert(blended.NumJoints() == 1);
    assert(std::abs(blended.local_rotations[0].SquaredNorm() - 1.0f) < 1.0e-4f);

    // N-way blending must not depend on the order in which examples are stored.
    // Sequential SLERP violates this for 3+ rotations; hemisphere-aligned
    // weighted quaternion averaging should pass.
    {
        const pmg::Pose a = MakeSingleJointPose(
            0.0f, pmg::EulerAxisRotation('X', 25.0f));
        const pmg::Pose b = MakeSingleJointPose(
            10.0f, pmg::EulerAxisRotation('Y', 60.0f));
        const pmg::Pose c = MakeSingleJointPose(
            20.0f, pmg::EulerAxisRotation('Z', -35.0f));

        const pmg::Pose abc =
            pmg::BlendPoseN({a, b, c}, {0.2f, 0.3f, 0.5f});
        const pmg::Pose cab =
            pmg::BlendPoseN({c, a, b}, {0.5f, 0.2f, 0.3f});

        assert(std::abs(abc.root_position.x - cab.root_position.x) < 1.0e-5f);
        assert(AbsQuaternionDot(
                   abc.local_rotations[0],
                   cab.local_rotations[0]) > 0.99999f);
        assert(std::abs(abc.local_rotations[0].SquaredNorm() - 1.0f) < 1.0e-4f);
        assert(std::abs(cab.local_rotations[0].SquaredNorm() - 1.0f) < 1.0e-4f);
    }

    // Antipodal representation should not cancel the weighted mean.
    {
        pmg::Quaternion q = pmg::EulerAxisRotation('Y', 45.0f);
        pmg::Quaternion neg_q(-q.w, -q.x, -q.y, -q.z);

        const pmg::Pose p = MakeSingleJointPose(0.0f, q);
        const pmg::Pose neg_p = MakeSingleJointPose(0.0f, neg_q);

        const pmg::Pose mean =
            pmg::BlendPoseN({p, neg_p}, {0.5f, 0.5f});

        assert(AbsQuaternionDot(mean.local_rotations[0], q) > 0.99999f);
    }

    return 0;
}