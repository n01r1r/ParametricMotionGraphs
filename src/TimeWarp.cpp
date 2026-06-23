#include "pmg/TimeWarp.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace pmg {

namespace {

void RequireStrictlyIncreasingInterior(
    const std::vector<float>& phases, const char* label) {
    float previous = 0.0f;
    for (const float phase : phases) {
        if (phase <= previous || phase >= 1.0f) {
            throw std::runtime_error(
                std::string("TimeWarp::FromAnchors: ") + label +
                " anchors must be strictly increasing within (0, 1)");
        }
        previous = phase;
    }
}

// Fritsch-Carlson monotone cubic Hermite tangents. Given strictly increasing
// knots (x, y), returns dy/dx at each knot such that the cubic Hermite spline
// through the knots is C1 and never overshoots, so a monotone mapping stays
// monotone. See KG04 section 4.1 (strictly increasing timewarp spline).
std::vector<float> ComputeMonotoneTangents(
    const std::vector<float>& x, const std::vector<float>& y) {
    const std::size_t n = x.size();
    std::vector<float> secant(n > 0 ? n - 1 : 0, 0.0f);
    for (std::size_t i = 0; i + 1 < n; ++i) {
        secant[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
    }

    std::vector<float> tangent(n, 0.0f);
    tangent.front() = secant.front();
    tangent.back() = secant.back();
    for (std::size_t i = 1; i + 1 < n; ++i) {
        tangent[i] = 0.5f * (secant[i - 1] + secant[i]);
    }

    // Clamp tangents into the monotone region (Fritsch-Carlson): on any segment
    // where alpha^2 + beta^2 > 9, scale both endpoint tangents back.
    for (std::size_t i = 0; i + 1 < n; ++i) {
        if (secant[i] == 0.0f) {
            tangent[i] = 0.0f;
            tangent[i + 1] = 0.0f;
            continue;
        }
        const float alpha = tangent[i] / secant[i];
        const float beta = tangent[i + 1] / secant[i];
        const float magnitude = alpha * alpha + beta * beta;
        if (magnitude > 9.0f) {
            const float scale = 3.0f / std::sqrt(magnitude);
            tangent[i] = scale * alpha * secant[i];
            tangent[i + 1] = scale * beta * secant[i];
        }
    }
    return tangent;
}

}  // namespace

TimeWarp TimeWarp::FromAnchors(
    const std::vector<float>& from_phases,
    const std::vector<float>& to_phases) {
    if (from_phases.size() != to_phases.size()) {
        throw std::runtime_error(
            "TimeWarp::FromAnchors: anchor lists must have matching length");
    }
    RequireStrictlyIncreasingInterior(from_phases, "from");
    RequireStrictlyIncreasingInterior(to_phases, "to");

    TimeWarp warp;
    if (from_phases.empty()) {
        return warp;
    }

    warp.from_knots_.reserve(from_phases.size() + 2);
    warp.to_knots_.reserve(to_phases.size() + 2);

    warp.from_knots_.push_back(0.0f);
    warp.to_knots_.push_back(0.0f);
    warp.from_knots_.insert(warp.from_knots_.end(), from_phases.begin(), from_phases.end());
    warp.to_knots_.insert(warp.to_knots_.end(), to_phases.begin(), to_phases.end());
    warp.from_knots_.push_back(1.0f);
    warp.to_knots_.push_back(1.0f);
    warp.tangents_ = ComputeMonotoneTangents(warp.from_knots_, warp.to_knots_);
    return warp;
}

float TimeWarp::Evaluate(float from_phase) const {
    from_phase = std::clamp(from_phase, 0.0f, 1.0f);
    if (from_knots_.empty()) {
        return from_phase;
    }

    const auto upper = std::upper_bound(from_knots_.begin(), from_knots_.end(), from_phase);
    if (upper == from_knots_.begin()) {
        return to_knots_.front();
    }
    if (upper == from_knots_.end()) {
        return to_knots_.back();
    }

    const std::size_t segment_end = static_cast<std::size_t>(upper - from_knots_.begin());
    const std::size_t segment_start = segment_end - 1;

    const float from_start = from_knots_[segment_start];
    const float from_end = from_knots_[segment_end];
    const float span = from_end - from_start;
    if (span <= 0.0f) {
        return to_knots_[segment_start];
    }

    // Monotone cubic Hermite on the segment (C1 across knots).
    const float t = (from_phase - from_start) / span;
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    const float h10 = t3 - 2.0f * t2 + t;
    const float h01 = -2.0f * t3 + 3.0f * t2;
    const float h11 = t3 - t2;
    return h00 * to_knots_[segment_start] +
           h10 * span * tangents_[segment_start] +
           h01 * to_knots_[segment_end] +
           h11 * span * tangents_[segment_end];
}

bool TimeWarp::IsIdentity() const {
    return from_knots_.empty();
}

int TimeWarp::NumAnchors() const {
    if (from_knots_.empty()) {
        return 0;
    }
    return static_cast<int>(from_knots_.size()) - 2;
}

std::vector<float> TimeWarp::InteriorFromPhases() const {
    if (from_knots_.empty()) {
        return {};
    }
    return {from_knots_.begin() + 1, from_knots_.end() - 1};
}

std::vector<float> TimeWarp::InteriorToPhases() const {
    if (to_knots_.empty()) {
        return {};
    }
    return {to_knots_.begin() + 1, to_knots_.end() - 1};
}

}  // namespace pmg
