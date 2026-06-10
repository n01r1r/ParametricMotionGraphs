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

    assert(Near(warp.Evaluate(0.0f), 0.0f));
    assert(Near(warp.Evaluate(0.5f), 0.6f));
    assert(Near(warp.Evaluate(1.0f), 1.0f));

    // Linear within segments: halfway to the anchor maps halfway to 0.6.
    assert(Near(warp.Evaluate(0.25f), 0.3f));
    assert(Near(warp.Evaluate(0.75f), 0.8f));
}

void TestMultipleAnchors() {
    const pmg::TimeWarp warp =
        pmg::TimeWarp::FromAnchors({0.25f, 0.5f, 0.75f}, {0.2f, 0.55f, 0.9f});
    assert(warp.NumAnchors() == 3);
    assert(Near(warp.Evaluate(0.25f), 0.2f));
    assert(Near(warp.Evaluate(0.5f), 0.55f));
    assert(Near(warp.Evaluate(0.75f), 0.9f));
    // Between anchors 2 and 3.
    assert(Near(warp.Evaluate(0.625f), 0.725f));
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
    TestMultipleAnchors();
    TestMonotonicity();
    TestClamping();
    TestValidation();
    return 0;
}
