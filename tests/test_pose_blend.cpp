#include "pmg/PoseBlend.h"

#include <cassert>
#include <cmath>

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
    return 0;
}
