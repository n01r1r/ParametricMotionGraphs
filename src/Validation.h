#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace pmg::detail {

inline void RequireFinite(float value, const char* context) {
    if (!std::isfinite(value)) {
        throw std::runtime_error(std::string(context) + ": value must be finite");
    }
}

inline void RequirePositive(float value, const char* context) {
    RequireFinite(value, context);
    if (value <= 0.0f) {
        throw std::runtime_error(std::string(context) + ": value must be positive");
    }
}

// Accumulate in double: valid finite float weights can overflow a float sum.
inline std::vector<float> NormalizeWeights(
    const std::vector<float>& weights, std::size_t count, const char* context) {
    if (count == 0 || weights.size() != count) {
        throw std::runtime_error(std::string(context) + ": weight count mismatch or empty input");
    }
    double sum = 0.0;
    for (float weight : weights) {
        RequireFinite(weight, context);
        if (weight < 0.0f) {
            throw std::runtime_error(std::string(context) + ": weights must be non-negative");
        }
        sum += weight;
    }
    if (sum <= 0.0) {
        throw std::runtime_error(std::string(context) + ": weight sum must be positive");
    }
    std::vector<float> normalized;
    normalized.reserve(count);
    for (float weight : weights) {
        normalized.push_back(static_cast<float>(weight / sum));
    }
    return normalized;
}

inline int SampleFrameCount(float duration, float fps) {
    RequireFinite(duration, "SampleFrameCount duration");
    RequirePositive(fps, "SampleFrameCount fps");
    const double intervals = static_cast<double>(duration) * fps;
    if (duration < 0.0f || intervals > std::numeric_limits<int>::max() - 2.0) {
        throw std::runtime_error("SampleFrameCount: duration/rate exceeds supported frame count");
    }
    return std::max(2, static_cast<int>(std::llround(intervals)) + 1);
}

}  // namespace pmg::detail
