#pragma once

#include "pmg/MathTypes.h"

namespace pmgviewer {

// A single, time-independent world/display transform. Scaling around a moving
// skeleton centroid adds artificial velocity to otherwise planted feet.
struct SceneCoordinates {
    float scale = 1.0f;
    float origin_x = 0.0f;
    float origin_z = 0.0f;

    pmg::Vec3 ToDisplay(const pmg::Vec3& p) const {
        return {(p.x - origin_x) * scale, p.y * scale, (p.z - origin_z) * scale};
    }
    pmg::Vec3 ToNative(const pmg::Vec3& p) const {
        return {p.x / scale + origin_x, p.y / scale, p.z / scale + origin_z};
    }
};

} // namespace pmgviewer
