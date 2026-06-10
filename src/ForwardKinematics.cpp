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

        skeleton.RequireValidJointIndex(joint.parent_index, "ComputeForwardKinematics");
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

}  // namespace pmg
