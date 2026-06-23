#pragma once

#include <vector>

namespace pmg {

// Monotonic C1 mapping between two normalized phase domains, pinned at (0, 0)
// and (1, 1). Built from matched interior anchor lists, e.g. foot-strike phases
// in a canonical domain mapped onto the phases where the same strikes occur in
// one example clip. This is the registration primitive that lets a motion space
// sample every example at the structurally corresponding moment instead of at
// the same raw phase. Interpolation is monotone cubic Hermite (Fritsch-Carlson),
// the strictly-increasing spline of KG04 (Kovar-Gleicher 2004) section 4.1: it
// passes through every knot, never overshoots, and is C1 (continuous phase
// velocity), so blended clips have no velocity kink at the contact knots.
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

    // Interior anchors without the pinned endpoints; both empty for identity.
    // FromAnchors(InteriorFromPhases(), InteriorToPhases()) reproduces this
    // warp exactly, which is what serialization relies on.
    std::vector<float> InteriorFromPhases() const;
    std::vector<float> InteriorToPhases() const;

private:
    // Full knot lists including the pinned endpoints; empty for identity.
    std::vector<float> from_knots_;
    std::vector<float> to_knots_;
    // Monotone Hermite tangents (dy/dx) at each knot; same length as the knot
    // lists, empty for identity. Precomputed by FromAnchors.
    std::vector<float> tangents_;
};

}  // namespace pmg
