#pragma once

#include "pmg/Pose.h"
#include "pmg/Skeleton.h"

#include <vector>

namespace pmg {

struct JointWorldState {
    Vec3 position;
    Quaternion rotation;
};

std::vector<JointWorldState> ComputeForwardKinematics(
    const Skeleton& skeleton,
    const Pose& pose);

std::vector<Vec3> ComputeJointWorldPositions(
    const Skeleton& skeleton,
    const Pose& pose);

}  // namespace pmg
