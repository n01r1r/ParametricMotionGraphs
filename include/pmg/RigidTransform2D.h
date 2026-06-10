#pragma once

#include "pmg/MathTypes.h"
#include "pmg/Pose.h"

namespace pmg {

// A 2D rigid transform on the floor plane: yaw about +Y (radians) then an
// (x, z) translation. This is the single representation of "place or align
// motion on the floor", shared by the point-cloud metric (as a recovered
// alignment), the graph edge store (baked per sample), and the runtime
// (the accumulated world placement, composed across transitions).
//
// Convention (matches the Kovar et al. 2002 closed-form alignment): rotating a
// floor vector by `yaw` gives x' = cos(yaw)*x + sin(yaw)*z,
// z' = -sin(yaw)*x + cos(yaw)*z. y is never touched by a floor transform.
struct RigidTransform2D {
    float yaw = 0.0f;  // radians, about +Y
    float dx = 0.0f;
    float dz = 0.0f;

    static RigidTransform2D Identity() { return {}; }

    // Rotate a floor-plane vector by `yaw` (no translation).
    void RotateFloor(float x, float z, float& out_x, float& out_z) const {
        const float cos_yaw = std::cos(yaw);
        const float sin_yaw = std::sin(yaw);
        out_x = cos_yaw * x + sin_yaw * z;
        out_z = -sin_yaw * x + cos_yaw * z;
    }

    // Map a floor-plane point through the full transform (rotate then translate).
    void ApplyFloor(float x, float z, float& out_x, float& out_z) const {
        RotateFloor(x, z, out_x, out_z);
        out_x += dx;
        out_z += dz;
    }

    // Apply to a clip-local pose, producing a world-space pose: the root's
    // (x, z) is mapped through the transform and the root joint's facing is
    // yawed by `yaw`. y and all non-root local rotations are unchanged.
    Pose Apply(const Pose& local_pose) const {
        Pose world_pose = local_pose;

        float rotated_x = 0.0f;
        float rotated_z = 0.0f;
        ApplyFloor(local_pose.root_position.x, local_pose.root_position.z,
                   rotated_x, rotated_z);
        world_pose.root_position.x = rotated_x;
        world_pose.root_position.z = rotated_z;

        if (!world_pose.local_rotations.empty()) {
            const Quaternion world_yaw =
                Quaternion::FromAxisAngle({0.0f, 1.0f, 0.0f}, yaw);
            world_pose.local_rotations[0] = world_yaw * world_pose.local_rotations[0];
        }
        return world_pose;
    }

    // Compose so that applying `inner` then `outer` equals applying the result:
    // (outer o inner)(p) = R(outer.yaw) * (R(inner.yaw)*p + inner.t) + outer.t.
    static RigidTransform2D Compose(const RigidTransform2D& outer,
                                    const RigidTransform2D& inner) {
        RigidTransform2D composed;
        composed.yaw = outer.yaw + inner.yaw;
        float rotated_x = 0.0f;
        float rotated_z = 0.0f;
        outer.RotateFloor(inner.dx, inner.dz, rotated_x, rotated_z);
        composed.dx = rotated_x + outer.dx;
        composed.dz = rotated_z + outer.dz;
        return composed;
    }
};

}  // namespace pmg
