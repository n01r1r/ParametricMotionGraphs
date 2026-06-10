#pragma once

#include "pmg/MathTypes.h"

#include <stdexcept>
#include <string>
#include <vector>

namespace pmg {

enum class BvhChannelType {
    XPosition,
    YPosition,
    ZPosition,
    XRotation,
    YRotation,
    ZRotation,
};

struct Joint {
    std::string name;
    int parent_index = -1;
    Vec3 offset;
    std::vector<BvhChannelType> channels;
};

struct Skeleton {
    std::vector<Joint> joints;

    int NumJoints() const {
        return static_cast<int>(joints.size());
    }

    void RequireValidJointIndex(int joint_index, const char* context) const {
        if (joint_index < 0 || joint_index >= NumJoints()) {
            throw std::runtime_error(std::string(context) + ": invalid joint index");
        }
    }
};

}  // namespace pmg
