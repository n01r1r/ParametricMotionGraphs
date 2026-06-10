#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace pmg {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegreesToRadians = kPi / 180.0f;
constexpr float kSmallEpsilon = 1.0e-6f;

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_value, float y_value, float z_value)
        : x(x_value), y(y_value), z(z_value) {}

    float SquaredNorm() const { return x * x + y * y + z * z; }
    float Norm() const { return std::sqrt(SquaredNorm()); }
};

inline Vec3 operator+(const Vec3& left, const Vec3& right) {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

inline Vec3 operator-(const Vec3& left, const Vec3& right) {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

inline Vec3 operator*(float scale, const Vec3& value) {
    return {scale * value.x, scale * value.y, scale * value.z};
}

inline Vec3 operator*(const Vec3& value, float scale) {
    return scale * value;
}

inline Vec3 operator/(const Vec3& value, float scale) {
    if (std::abs(scale) <= kSmallEpsilon) {
        throw std::runtime_error("cannot divide Vec3 by near-zero scale");
    }
    return {value.x / scale, value.y / scale, value.z / scale};
}

inline float Dot(const Vec3& left, const Vec3& right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

inline Vec3 Cross(const Vec3& left, const Vec3& right) {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x,
    };
}

struct Quaternion {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(float w_value, float x_value, float y_value, float z_value)
        : w(w_value), x(x_value), y(y_value), z(z_value) {}

    static Quaternion Identity() { return {}; }

    static Quaternion FromAxisAngle(const Vec3& axis, float angle_radians) {
        const float axis_norm = axis.Norm();
        if (axis_norm <= kSmallEpsilon) {
            return Identity();
        }

        const Vec3 normalized_axis = axis / axis_norm;
        const float half_angle = 0.5f * angle_radians;
        const float sin_half = std::sin(half_angle);
        return {
            std::cos(half_angle),
            normalized_axis.x * sin_half,
            normalized_axis.y * sin_half,
            normalized_axis.z * sin_half,
        };
    }

    float SquaredNorm() const { return w * w + x * x + y * y + z * z; }

    void Normalize() {
        const float norm = std::sqrt(SquaredNorm());
        if (norm <= kSmallEpsilon) {
            *this = Identity();
            return;
        }
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    Quaternion Normalized() const {
        Quaternion copy = *this;
        copy.Normalize();
        return copy;
    }

    Quaternion Conjugate() const { return {w, -x, -y, -z}; }
};

inline Quaternion operator*(const Quaternion& left, const Quaternion& right) {
    return {
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
    };
}

inline Vec3 Rotate(const Quaternion& rotation, const Vec3& point) {
    const Quaternion point_quaternion(0.0f, point.x, point.y, point.z);
    const Quaternion result = rotation * point_quaternion * rotation.Conjugate();
    return {result.x, result.y, result.z};
}

inline float QuaternionDot(const Quaternion& left, const Quaternion& right) {
    return left.w * right.w + left.x * right.x + left.y * right.y + left.z * right.z;
}

inline Quaternion Slerp(const Quaternion& start, const Quaternion& end, float alpha) {
    alpha = std::clamp(alpha, 0.0f, 1.0f);

    Quaternion target = end;
    float dot = QuaternionDot(start, target);

    if (dot < 0.0f) {
        target.w = -target.w;
        target.x = -target.x;
        target.y = -target.y;
        target.z = -target.z;
        dot = -dot;
    }

    constexpr float kLinearThreshold = 0.9995f;
    if (dot > kLinearThreshold) {
        Quaternion blended(
            start.w + alpha * (target.w - start.w),
            start.x + alpha * (target.x - start.x),
            start.y + alpha * (target.y - start.y),
            start.z + alpha * (target.z - start.z));
        blended.Normalize();
        return blended;
    }

    dot = std::clamp(dot, -1.0f, 1.0f);
    const float theta_0 = std::acos(dot);
    const float theta = theta_0 * alpha;
    const float sin_theta = std::sin(theta);
    const float sin_theta_0 = std::sin(theta_0);

    const float scale_start = std::cos(theta) - dot * sin_theta / sin_theta_0;
    const float scale_end = sin_theta / sin_theta_0;

    Quaternion blended(
        scale_start * start.w + scale_end * target.w,
        scale_start * start.x + scale_end * target.x,
        scale_start * start.y + scale_end * target.y,
        scale_start * start.z + scale_end * target.z);
    blended.Normalize();
    return blended;
}

inline Quaternion EulerAxisRotation(const char axis, float degrees) {
    const float radians = degrees * kDegreesToRadians;
    switch (axis) {
        case 'X': return Quaternion::FromAxisAngle({1.0f, 0.0f, 0.0f}, radians);
        case 'Y': return Quaternion::FromAxisAngle({0.0f, 1.0f, 0.0f}, radians);
        case 'Z': return Quaternion::FromAxisAngle({0.0f, 0.0f, 1.0f}, radians);
        default: throw std::runtime_error("unsupported Euler axis");
    }
}

}  // namespace pmg
