#include "pmg/ForwardKinematics.h"

#include <stdexcept>

namespace pmg {

std::vector<JointWorldState> ComputeForwardKinematics(
    const Skeleton& skeleton,
    const Pose& pose) {
    pose.RequireJointCount(skeleton.NumJoints(), "ComputeForwardKinematics");

    std::vector<JointWorldState> world_states(skeleton.NumJoints());

    for (int joint_index = 0; joint_index < skeleton.NumJoints(); ++joint_index) {
        const Joint& joint = skeleton.joints[joint_index];
        const Quaternion local_rotation = pose.local_rotations[joint_index];

        if (joint.parent_index < 0) {
            world_states[joint_index].position = pose.root_position + joint.offset;
            world_states[joint_index].rotation = local_rotation;
            world_states[joint_index].rotation.Normalize();
            continue;
        }

        if (joint.parent_index < 0 || joint.parent_index >= skeleton.NumJoints()) {
            throw std::runtime_error("ComputeForwardKinematics: invalid parent joint index");
        }
        const JointWorldState& parent = world_states[joint.parent_index];
        world_states[joint_index].position = parent.position + Rotate(parent.rotation, joint.offset);
        world_states[joint_index].rotation = parent.rotation * local_rotation;
        world_states[joint_index].rotation.Normalize();
    }

    return world_states;
}

std::vector<Vec3> ComputeJointWorldPositions(
    const Skeleton& skeleton,
    const Pose& pose) {
    const std::vector<JointWorldState> world_states = ComputeForwardKinematics(skeleton, pose);
    std::vector<Vec3> positions;
    positions.reserve(world_states.size());

    for (const JointWorldState& state : world_states) {
        positions.push_back(state.position);
    }

    return positions;
}

float MeanJointDistance(
    const std::vector<Vec3>& first_positions,
    const std::vector<Vec3>& second_positions) {
    if (first_positions.empty() ||
        first_positions.size() != second_positions.size()) {
        return 0.0f;
    }
    float distance_sum = 0.0f;
    for (std::size_t joint = 0; joint < first_positions.size(); ++joint) {
        distance_sum +=
            (second_positions[joint] - first_positions[joint]).Norm();
    }
    return distance_sum / static_cast<float>(first_positions.size());
}

float MeanJointDistance(
    const Skeleton& skeleton,
    const Pose& first,
    const Pose& second) {
    return MeanJointDistance(
        ComputeJointWorldPositions(skeleton, first),
        ComputeJointWorldPositions(skeleton, second));
}

}  // namespace pmg
