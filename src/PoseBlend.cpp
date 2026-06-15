#include "pmg/PoseBlend.h"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace pmg {

namespace {

Quaternion Negated(Quaternion value) {
    value.w = -value.w;
    value.x = -value.x;
    value.y = -value.y;
    value.z = -value.z;
    return value;
}

Quaternion WeightedQuaternionMean(
    const std::vector<Pose>& poses,
    const std::vector<float>& normalized_weights,
    int joint_index) {
    std::size_t reference_index = 0;
    for (std::size_t index = 0; index < normalized_weights.size(); ++index) {
        if (normalized_weights[index] > kSmallEpsilon) {
            reference_index = index;
            break;
        }
    }

    const Quaternion reference =
        poses[reference_index].local_rotations[joint_index];

    Quaternion sum(0.0f, 0.0f, 0.0f, 0.0f);

    for (std::size_t pose_index = 0; pose_index < poses.size();
         ++pose_index) {
        Quaternion q = poses[pose_index].local_rotations[joint_index];

        // Quaternions q and -q represent the same rotation. Align every sample
        // to the same hemisphere before averaging, otherwise antipodal samples
        // can cancel each other.
        if (QuaternionDot(reference, q) < 0.0f) {
            q = Negated(q);
        }

        const float weight = normalized_weights[pose_index];
        sum.w += weight * q.w;
        sum.x += weight * q.x;
        sum.y += weight * q.y;
        sum.z += weight * q.z;
    }

    if (sum.SquaredNorm() <= kSmallEpsilon) {
        return reference.Normalized();
    }

    sum.Normalize();
    return sum;
}

}  // namespace

Pose BlendPose(
    const Pose& first_pose,
    const Pose& second_pose,
    float second_pose_weight) {
    if (first_pose.NumJoints() != second_pose.NumJoints()) {
        throw std::runtime_error("BlendPose: pose joint count mismatch");
    }

    second_pose_weight =
        std::clamp(second_pose_weight, 0.0f, 1.0f);
    const float first_pose_weight = 1.0f - second_pose_weight;

    Pose output;
    output.root_position =
        first_pose_weight * first_pose.root_position +
        second_pose_weight * second_pose.root_position;
    output.local_rotations.resize(first_pose.local_rotations.size());

    for (std::size_t joint_index = 0;
         joint_index < output.local_rotations.size();
         ++joint_index) {
        output.local_rotations[joint_index] = Slerp(
            first_pose.local_rotations[joint_index],
            second_pose.local_rotations[joint_index],
            second_pose_weight);
    }

    return output;
}

Pose BlendPoseN(
    const std::vector<Pose>& poses,
    const std::vector<float>& weights) {
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

    const float weight_sum =
        std::accumulate(weights.begin(), weights.end(), 0.0f);
    if (weight_sum <= kSmallEpsilon) {
        throw std::runtime_error(
            "BlendPoseN: weight sum must be positive");
    }

    std::vector<float> normalized_weights(weights.size());
    for (std::size_t index = 0; index < weights.size(); ++index) {
        if (weights[index] < 0.0f) {
            throw std::runtime_error(
                "BlendPoseN: weights must be non-negative");
        }
        normalized_weights[index] = weights[index] / weight_sum;
    }

    Pose output;
    output.root_position = {};
    output.local_rotations.resize(joint_count, Quaternion::Identity());

    for (std::size_t pose_index = 0; pose_index < poses.size();
         ++pose_index) {
        output.root_position =
            output.root_position +
            normalized_weights[pose_index] *
                poses[pose_index].root_position;
    }

    for (int joint_index = 0; joint_index < joint_count; ++joint_index) {
        output.local_rotations[joint_index] =
            WeightedQuaternionMean(
                poses,
                normalized_weights,
                joint_index);
    }

    return output;
}

}  // namespace pmg