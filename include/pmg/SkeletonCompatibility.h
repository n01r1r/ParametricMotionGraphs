#pragma once

#include "pmg/Skeleton.h"

#include <string>

namespace pmg {

struct SkeletonCompatibilityResult {
    bool compatible = true;
    std::string reason;
};

// Strict BVH skeleton compatibility for blending and point-cloud correspondence.
// Joint count alone is insufficient: all examples in a parametric motion space
// must agree on names, hierarchy, offsets, and channel conventions.
SkeletonCompatibilityResult CheckSkeletonCompatibility(
    const Skeleton& reference,
    const Skeleton& candidate,
    float offset_tolerance = 1.0e-4f);

void RequireSkeletonCompatible(
    const Skeleton& reference,
    const Skeleton& candidate,
    const char* context,
    float offset_tolerance = 1.0e-4f);

}  // namespace pmg
