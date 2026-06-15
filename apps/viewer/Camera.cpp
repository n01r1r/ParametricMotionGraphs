#include "Camera.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

namespace pmgviewer {

namespace {
constexpr float kFieldOfViewYRadians = 0.7853981634f;  // 45 degrees
constexpr float kNearPlane = 0.5f;
constexpr float kFarPlane = 20000.0f;
constexpr float kMinPitchRadians = -1.5f;  // just shy of straight down
constexpr float kMaxPitchRadians = 1.5f;   // just shy of straight up
constexpr float kMinDistance = 3.0f;
constexpr float kMaxDistance = 2000.0f;
constexpr float kZoomPerStep = 0.9f;       // distance multiplier per scroll step

// Defaults mirror the in-class member initializers; Reset() restores them.
constexpr float kDefaultYawRadians = 0.7f;
constexpr float kDefaultPitchRadians = 0.35f;
constexpr float kDefaultDistance = 120.0f;
const glm::vec3 kDefaultTarget(0.0f, 35.0f, 0.0f);
}  // namespace

void OrbitCamera::Orbit(float delta_yaw_radians, float delta_pitch_radians) {
    yaw_radians_ += delta_yaw_radians;
    pitch_radians_ = std::clamp(pitch_radians_ + delta_pitch_radians,
                                kMinPitchRadians, kMaxPitchRadians);
}

void OrbitCamera::Zoom(float scroll_steps) {
    distance_ = std::clamp(distance_ * std::pow(kZoomPerStep, scroll_steps),
                           kMinDistance, kMaxDistance);
}

void OrbitCamera::Pan(float right_amount, float up_amount) {
    const glm::vec3 forward = glm::normalize(target_world_ - Position());
    const glm::vec3 world_up(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::cross(forward, world_up);
    const float right_length = glm::length(right);
    right = right_length > 1.0e-6f ? right / right_length
                                   : glm::vec3(1.0f, 0.0f, 0.0f);
    const glm::vec3 up = glm::normalize(glm::cross(right, forward));
    target_world_ += (right * right_amount + up * up_amount) * distance_;
}

void OrbitCamera::SetTarget(const glm::vec3& target_world) {
    target_world_ = target_world;
}

void OrbitCamera::SetAspectRatio(float aspect_ratio) {
    aspect_ratio_ = aspect_ratio > 0.0f ? aspect_ratio : aspect_ratio_;
}

void OrbitCamera::Reset() {
    yaw_radians_ = kDefaultYawRadians;
    pitch_radians_ = kDefaultPitchRadians;
    distance_ = kDefaultDistance;
    target_world_ = kDefaultTarget;
}

glm::vec3 OrbitCamera::Position() const {
    const float cos_pitch = std::cos(pitch_radians_);
    const glm::vec3 offset_direction(
        cos_pitch * std::cos(yaw_radians_),
        std::sin(pitch_radians_),
        cos_pitch * std::sin(yaw_radians_));
    return target_world_ + offset_direction * distance_;
}

glm::mat4 OrbitCamera::ViewMatrix() const {
    return glm::lookAt(Position(), target_world_, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 OrbitCamera::ProjectionMatrix() const {
    return glm::perspective(kFieldOfViewYRadians, aspect_ratio_, kNearPlane, kFarPlane);
}

}  // namespace pmgviewer
