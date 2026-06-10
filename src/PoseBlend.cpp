#include "pmg/PoseBlend.h"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace pmg {

Pose BlendPose(const Pose& first_pose, const Pose& second_pose, float second_pose_weight) {
    if (first_pose.NumJoints() != second_pose.NumJoints()) {
        throw std::runtime_error("BlendPose: pose joint count mismatch");
    }

    second_pose_weight = std::clamp(second_pose_weight, 0.0f, 1.0f);
    const float first_pose_weight = 1.0f - second_pose_weight;

    Pose output;
    output.root_position =
        first_pose_weight * first_pose.root_position +
        second_pose_weight * second_pose.root_position;
    output.local_rotations.resize(first_pose.local_rotations.size());

    for (std::size_t joint_index = 0; joint_index < output.local_rotations.size(); ++joint_index) {
        output.local_rotations[joint_index] = Slerp(
            first_pose.local_rotations[joint_index],
            second_pose.local_rotations[joint_index],
            second_pose_weight);
    }

    return output;
}

Pose BlendPoseN(const std::vector<Pose>& poses, const std::vector<float>& weights) {
    if (poses.empty()) {
        throw std::runtime_error("BlendPoseN: poses must not be empty");
    }
    if (poses.size() != weights.size()) {
        throw std::runtime_error("BlendPoseN: pose/weight count mismatch");
    }

    const int joint_count = poses.front().NumJoints();
    for (const Pose& pose : poses) {
        pose.RequireJointCount(joint_count, "BlendPoseN");
    }

    const float weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0f);
    if (weight_sum <= kSmallEpsilon) {
        throw std::runtime_error("BlendPoseN: weight sum must be positive");
    }

    std::vector<float> normalized_weights(weights.size());
    for (std::size_t index = 0; index < weights.size(); ++index) {
        if (weights[index] < 0.0f) {
            throw std::runtime_error("BlendPoseN: weights must be non-negative");
        }
        normalized_weights[index] = weights[index] / weight_sum;
    }

    Pose output;
    output.root_position = {};
    output.local_rotations.resize(joint_count, Quaternion::Identity());

    for (std::size_t pose_index = 0; pose_index < poses.size(); ++pose_index) {
        output.root_position = output.root_position +
            normalized_weights[pose_index] * poses[pose_index].root_position;
    }

    for (int joint_index = 0; joint_index < joint_count; ++joint_index) {
        Quaternion blended = poses.front().local_rotations[joint_index];
        float accumulated_weight = normalized_weights.front();

        for (std::size_t pose_index = 1; pose_index < poses.size(); ++pose_index) {
            const float next_weight = normalized_weights[pose_index];
            const float combined_weight = accumulated_weight + next_weight;
            if (combined_weight > kSmallEpsilon) {
                const float alpha = next_weight / combined_weight;
                blended = Slerp(blended, poses[pose_index].local_rotations[joint_index], alpha);
            }
            accumulated_weight = combined_weight;
        }

        blended.Normalize();
        output.local_rotations[joint_index] = blended;
    }

    return output;
}

}  // namespace pmg
