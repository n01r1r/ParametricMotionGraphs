#pragma once

#include "pmg/ContactDetection.h"
#include "pmg/MotionClip.h"
#include "pmg/Skeleton.h"

#include <vector>

namespace pmg {

struct FootLockSettings {
    ContactDetectionSettings contacts;
    // Frames before the strike / after the lift over which the lock eases in
    // and out, so the pinned foot does not pop at interval boundaries.
    int blend_frames = 3;
};

// Two-bone analytic IK: move `end_joint` (e.g. an ankle) of `pose` to
// `target`, bending at its parent and rotating at its grandparent. The end
// joint's world orientation is preserved so a planted foot does not twist.
// Unreachable targets clamp to the chain's reach.
void SolveTwoBoneIk(
    const Skeleton& skeleton,
    Pose& pose,
    int end_joint,
    const Vec3& target);

// Remove residual foot slide from a blended clip: detect contact intervals of
// `contact_joints`, then pin each joint at its strike-frame world position for
// the interval's duration via two-bone IK, easing in/out over blend_frames.
// This is a post-process for generated (blended) clips whose stance feet
// drift; original mocap should not need it.
void LockFootContacts(
    const Skeleton& skeleton,
    MotionClip& clip,
    const std::vector<int>& contact_joints,
    const FootLockSettings& settings);

}  // namespace pmg
