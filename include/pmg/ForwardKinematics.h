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

// Mean Euclidean distance between corresponding joint world positions of two
// poses (the pose-pop metric). The vector overload compares precomputed
// positions and returns 0 for empty or size-mismatched inputs; the skeleton
// overload computes world positions first via ComputeJointWorldPositions.
float MeanJointDistance(
    const std::vector<Vec3>& first_positions,
    const std::vector<Vec3>& second_positions);

float MeanJointDistance(
    const Skeleton& skeleton,
    const Pose& first,
    const Pose& second);

}  // namespace pmg
