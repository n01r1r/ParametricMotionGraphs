#pragma once

#include <glm/glm.hpp>

namespace pmgviewer {

// Orbit camera parameterized by spherical angles around a target point.
//
// Purpose: let the user inspect a skeleton from any direction with mouse drag
// (orbit), scroll (zoom), while the target auto-follows the motion centroid.
// Inputs:  orbit deltas in radians, zoom steps (unitless), aspect ratio.
// Outputs: view/projection matrices and world-space eye position.
// Assumptions: world is Y-up; pitch is clamped to avoid flipping over the pole.
class OrbitCamera {
public:
    void Orbit(float delta_yaw_radians, float delta_pitch_radians);
    void Zoom(float scroll_steps);
    // Screen-aligned translation (WASD): move the camera along its own right and
    // up axes by translating the orbit target (the eye is derived from it, so
    // both move together and the view direction is unchanged). Amounts are
    // fractions of the orbit distance, so the feel is constant at any zoom.
    void Pan(float right_amount, float up_amount);
    void SetTarget(const glm::vec3& target_world);
    void SetAspectRatio(float aspect_ratio);

    // Restore orbit angles, zoom distance, and target to their defaults.
    // Aspect ratio is left untouched (driven by the framebuffer each frame).
    void Reset();

    glm::mat4 ViewMatrix() const;
    glm::mat4 ProjectionMatrix() const;
    glm::vec3 Position() const;
    const glm::vec3& Target() const { return target_world_; }

private:
    glm::vec3 target_world_{0.0f, 35.0f, 0.0f};
    float yaw_radians_ = 0.7f;
    float pitch_radians_ = 0.35f;
    float distance_ = 120.0f;
    float aspect_ratio_ = 16.0f / 9.0f;
};

}  // namespace pmgviewer
