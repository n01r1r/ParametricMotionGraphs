#include "pmg/SkeletonCompatibility.h"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace pmg {

namespace {

bool OffsetsClose(const Vec3& a, const Vec3& b, float tolerance) {
    return std::abs(a.x - b.x) <= tolerance &&
           std::abs(a.y - b.y) <= tolerance &&
           std::abs(a.z - b.z) <= tolerance;
}

std::string JointPrefix(int joint_index) {
    std::ostringstream out;
    out << "joint[" << joint_index << "]";
    return out.str();
}

}  // namespace

SkeletonCompatibilityResult CheckSkeletonCompatibility(
    const Skeleton& reference,
    const Skeleton& candidate,
    float offset_tolerance) {
    if (offset_tolerance < 0.0f) {
        return {false, "offset tolerance must be non-negative"};
    }

    if (reference.NumJoints() != candidate.NumJoints()) {
        std::ostringstream out;
        out << "joint count mismatch: reference=" << reference.NumJoints()
            << " candidate=" << candidate.NumJoints();
        return {false, out.str()};
    }

    for (int joint_index = 0; joint_index < reference.NumJoints(); ++joint_index) {
        const Joint& ref_joint = reference.joints[joint_index];
        const Joint& cand_joint = candidate.joints[joint_index];
        const std::string prefix = JointPrefix(joint_index);

        if (ref_joint.name != cand_joint.name) {
            return {false, prefix + " name mismatch: reference='" + ref_joint.name +
                           "' candidate='" + cand_joint.name + "'"};
        }
        if (ref_joint.parent_index != cand_joint.parent_index) {
            std::ostringstream out;
            out << prefix << " parent mismatch: reference=" << ref_joint.parent_index
                << " candidate=" << cand_joint.parent_index;
            return {false, out.str()};
        }
        if (!OffsetsClose(ref_joint.offset, cand_joint.offset, offset_tolerance)) {
            std::ostringstream out;
            out << prefix << " offset mismatch: reference=(" << ref_joint.offset.x << ", "
                << ref_joint.offset.y << ", " << ref_joint.offset.z << ") candidate=("
                << cand_joint.offset.x << ", " << cand_joint.offset.y << ", "
                << cand_joint.offset.z << ")";
            return {false, out.str()};
        }
        if (ref_joint.channels != cand_joint.channels) {
            std::ostringstream out;
            out << prefix << " channel convention mismatch";
            return {false, out.str()};
        }
    }

    return {true, {}};
}

void RequireSkeletonCompatible(
    const Skeleton& reference,
    const Skeleton& candidate,
    const char* context,
    float offset_tolerance) {
    const SkeletonCompatibilityResult result =
        CheckSkeletonCompatibility(reference, candidate, offset_tolerance);
    if (!result.compatible) {
        throw std::runtime_error(std::string(context) + ": " + result.reason);
    }
}

}  // namespace pmg
