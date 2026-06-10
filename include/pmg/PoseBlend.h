#pragma once

#include "pmg/Pose.h"

#include <vector>

namespace pmg {

Pose BlendPose(const Pose& first_pose, const Pose& second_pose, float second_pose_weight);

Pose BlendPoseN(
    const std::vector<Pose>& poses,
    const std::vector<float>& weights);

}  // namespace pmg
