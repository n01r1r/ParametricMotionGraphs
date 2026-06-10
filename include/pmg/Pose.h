#pragma once

#include "pmg/MathTypes.h"

#include <stdexcept>
#include <vector>

namespace pmg {

struct Pose {
    Vec3 root_position;
    std::vector<Quaternion> local_rotations;

    int NumJoints() const {
        return static_cast<int>(local_rotations.size());
    }

    void NormalizeRotations() {
        for (Quaternion& rotation : local_rotations) {
            rotation.Normalize();
        }
    }

    void RequireJointCount(int expected_joint_count, const char* context) const {
        if (NumJoints() != expected_joint_count) {
            throw std::runtime_error(std::string(context) + ": pose joint count mismatch");
        }
    }
};

}  // namespace pmg
