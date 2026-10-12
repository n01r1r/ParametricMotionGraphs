#pragma once

#include "pmg/ForwardKinematics.h"

namespace pmg {

struct FootPlantDiagnostics {
    int planted_feet = 0;
    float max_raw_anchor_error = 0.0f; // same active anchors, before cleanup
    float max_anchor_error = 0.0f; // native BVH units, after IK
    float root_correction = 0.0f;
};

// Stateful, fixed-bone-length contact cleanup. Contact labels must come from
// the uncorrected animation (or an authored track), never from corrected feet.
// This enforces kinematic constraints; it is not a rigid-body simulator.
class StreamingFootLock {
public:
    StreamingFootLock(Skeleton skeleton, std::vector<int> ankle_joints,
                      float release_seconds = 0.1f);
    void Reset();
    Pose Update(const Pose& raw_pose, const std::vector<bool>& contacts, float dt);
    const Skeleton& GetSkeleton() const { return skeleton_; }
    const std::vector<int>& Joints() const { return joints_; }
    const FootPlantDiagnostics& Diagnostics() const { return diagnostics_; }

private:
    struct Plant {
        bool active = false;
        float release_weight = 0.0f;
        Vec3 anchor;
        Quaternion orientation;
        Vec3 release_offset;
    };
    Skeleton skeleton_;
    std::vector<int> joints_;
    std::vector<Plant> plants_;
    float release_seconds_;
    Vec3 root_offset_;
    FootPlantDiagnostics diagnostics_;
};

} // namespace pmg
