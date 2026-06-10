#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

namespace pmg {

using ParameterVector = std::vector<float>;

inline void RequireSameParameterDimension(
    const ParameterVector& left,
    const ParameterVector& right,
    const char* context) {
    if (left.size() != right.size()) {
        throw std::runtime_error(std::string(context) + ": parameter dimension mismatch");
    }
}

inline float SquaredDistance(
    const ParameterVector& left,
    const ParameterVector& right) {
    RequireSameParameterDimension(left, right, "SquaredDistance");

    float sum = 0.0f;
    for (std::size_t index = 0; index < left.size(); ++index) {
        const float delta = left[index] - right[index];
        sum += delta * delta;
    }
    return sum;
}

inline float Distance(
    const ParameterVector& left,
    const ParameterVector& right) {
    return std::sqrt(SquaredDistance(left, right));
}

}  // namespace pmg
