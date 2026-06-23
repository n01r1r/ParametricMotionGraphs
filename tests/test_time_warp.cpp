#include "pmg/TimeWarp.h"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {

bool Near(float left, float right, float tolerance = 1.0e-5f) {
    return std::abs(left - right) < tolerance;
}

void TestIdentity() {
    const pmg::TimeWarp identity;
    assert(identity.IsIdentity());
    assert(identity.NumAnchors() == 0);
    assert(Near(identity.Evaluate(0.0f), 0.0f));
    assert(Near(identity.Evaluate(0.37f), 0.37f));
    assert(Near(identity.Evaluate(1.0f), 1.0f));

    const pmg::TimeWarp empty_anchors = pmg::TimeWarp::FromAnchors({}, {});
    assert(empty_anchors.IsIdentity());
}

void TestSingleAnchor() {
    // Canonical mid-cycle contact at 0.5 maps to example contact at 0.6.
    const pmg::TimeWarp warp = pmg::TimeWarp::FromAnchors({0.5f}, {0.6f});
    assert(!warp.IsIdentity());
    assert(warp.NumAnchors() == 1);

    // Knots are interpolated exactly.
    assert(Near(warp.Evaluate(0.0f), 0.0f));
    assert(Near(warp.Evaluate(0.5f), 0.6f));
    assert(Near(warp.Evaluate(1.0f), 1.0f));

    // Monotone cubic Hermite between knots (not piecewise-linear): the mid
    // points sit near, but not on, the straight-line values 0.3 / 0.8.
    assert(Near(warp.Evaluate(0.25f), 0.3125f, 1.0e-3f));
    assert(Near(warp.Evaluate(0.75f), 0.8125f, 1.0e-3f));
}

void TestC1Continuity() {
    // Piecewise-linear warps jump in slope at each knot; the C1 Hermite warp
    // does not. Compare the one-sided finite-difference slopes across a knot.
    const pmg::TimeWarp warp = pmg::TimeWarp::FromAnchors({0.5f}, {0.6f});
    const float knot = 0.5f;
    const float h = 2.0e-3f;
    const float slope_below =
        (warp.Evaluate(knot) - warp.Evaluate(knot - h)) / h;
    const float slope_above =
        (warp.Evaluate(knot + h) - warp.Evaluate(knot)) / h;
    // PWL would give 1.2 vs 0.8 here; C1 keeps them equal.
    assert(Near(slope_below, slope_above, 2.0e-2f));
}

void TestMultipleAnchors() {
    const pmg::TimeWarp warp =
        pmg::TimeWarp::FromAnchors({0.25f, 0.5f, 0.75f}, {0.2f, 0.55f, 0.9f});
    assert(warp.NumAnchors() == 3);
    assert(Near(warp.Evaluate(0.25f), 0.2f));
    assert(Near(warp.Evaluate(0.5f), 0.55f));
    assert(Near(warp.Evaluate(0.75f), 0.9f));
    // Between anchors 2 and 3 the spline curves off the straight line 0.725.
    assert(Near(warp.Evaluate(0.625f), 0.740625f, 1.0e-3f));
}

void TestMonotonicity() {
    const pmg::TimeWarp warp =
        pmg::TimeWarp::FromAnchors({0.3f, 0.6f}, {0.5f, 0.55f});
    float previous = -1.0f;
    for (int step = 0; step <= 100; ++step) {
        const float phase = static_cast<float>(step) / 100.0f;
        const float warped = warp.Evaluate(phase);
        assert(warped >= previous);
        assert(warped >= 0.0f && warped <= 1.0f);
        previous = warped;
    }
}

void TestClamping() {
    const pmg::TimeWarp warp = pmg::TimeWarp::FromAnchors({0.5f}, {0.6f});
    assert(Near(warp.Evaluate(-0.5f), 0.0f));
    assert(Near(warp.Evaluate(1.5f), 1.0f));
}

void TestValidation() {
    bool threw = false;
    try {
        pmg::TimeWarp::FromAnchors({0.5f}, {0.6f, 0.7f});  // length mismatch
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);

    threw = false;
    try {
        pmg::TimeWarp::FromAnchors({0.5f, 0.4f}, {0.5f, 0.6f});  // not increasing
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);

    threw = false;
    try {
        pmg::TimeWarp::FromAnchors({0.0f}, {0.5f});  // endpoint not interior
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);

    threw = false;
    try {
        pmg::TimeWarp::FromAnchors({0.5f}, {1.0f});  // endpoint not interior
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

}  // namespace

int main() {
    TestIdentity();
    TestSingleAnchor();
    TestC1Continuity();
    TestMultipleAnchors();
    TestMonotonicity();
    TestClamping();
    TestValidation();
    return 0;
}
