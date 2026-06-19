#pragma once

#include "pmg/MotionClip.h"

namespace pmg {

struct RootStartSummary {
    Vec3 first_root_position;
    float first_heading_yaw = 0.0f;
    Vec3 final_relative_displacement;
    float final_relative_heading = 0.0f;
    Vec3 normalized_bounds_min;
    Vec3 normalized_bounds_max;
};

// Clip-local canonical frame for PMG examples: first root x/z at origin and
// first root heading at yaw 0. Root height and relative motion are preserved.
MotionClip CanonicalizeRootOrigin(const MotionClip& clip);

RootStartSummary SummarizeRootStart(const MotionClip& clip);

float RootHeadingYaw(const Pose& pose);

}  // namespace pmg
