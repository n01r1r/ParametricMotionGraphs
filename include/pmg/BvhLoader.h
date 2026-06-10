#pragma once

#include "pmg/MotionClip.h"
#include "pmg/Skeleton.h"

#include <string>

namespace pmg {

struct BvhData {
    Skeleton skeleton;
    MotionClip clip;
};

class BvhLoader {
public:
    // Loads BVH hierarchy/motion data into the internal Y-up coordinate system.
    // The loader preserves BVH axis order (x, y, z) and the file's NATIVE units
    // (no unit scaling: any display scale is a render-time concern), and composes
    // Euler rotations in the channel order declared by the BVH file.
    static BvhData Load(const std::string& path);
};

}  // namespace pmg
