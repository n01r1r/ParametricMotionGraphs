#pragma once

#include <vector>

namespace pmg {

// Monotonic piecewise-linear mapping between two normalized phase domains,
// pinned at (0, 0) and (1, 1). Built from matched interior anchor lists —
// e.g. foot-strike phases in a canonical domain mapped onto the phases where
// the same strikes occur in one example clip. This is the registration
// primitive that lets a motion space sample every example at the structurally
// corresponding moment instead of at the same raw phase.
class TimeWarp {
public:
    // Identity warp: Evaluate(phase) == phase.
    TimeWarp() = default;

    // `from_phases` / `to_phases` are matched interior anchors, both strictly
    // increasing within (0, 1) and the same length. Endpoints 0 and 1 are
    // added implicitly. Empty lists yield the identity warp.
    static TimeWarp FromAnchors(
        const std::vector<float>& from_phases,
        const std::vector<float>& to_phases);

    float Evaluate(float from_phase) const;

    bool IsIdentity() const;
    int NumAnchors() const;

private:
    // Full knot lists including the pinned endpoints; empty for identity.
    std::vector<float> from_knots_;
    std::vector<float> to_knots_;
};

}  // namespace pmg
