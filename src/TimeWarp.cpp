#include "pmg/TimeWarp.h"

#include <algorithm>
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
    const float segment_span = from_end - from_start;

    const float alpha = segment_span > 0.0f ? (from_phase - from_start) / segment_span : 0.0f;
    return to_knots_[segment_start] + alpha * (to_knots_[segment_end] - to_knots_[segment_start]);
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

}  // namespace pmg
