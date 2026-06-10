#include "pmg/MotionRegistration.h"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {

bool Near(float left, float right, float tolerance = 1.0e-5f) {
    return std::abs(left - right) < tolerance;
}

void TestContactAnchorPhases() {
    // 31 frames -> phase = frame / 30.
    const std::vector<pmg::ContactInterval> intervals = {
        {0, 0, 6},     // strike at phase 0 (dropped), lift at 0.2
        {0, 18, 30},   // strike at 0.6, lift at phase 1 (dropped)
        {1, 18, 24},   // strike at 0.6 (duplicate, collapsed), lift at 0.8
    };

    const std::vector<float> anchors = pmg::ContactAnchorPhases(intervals, 31);
    assert(anchors.size() == 3);
    assert(Near(anchors[0], 0.2f));
    assert(Near(anchors[1], 0.6f));
    assert(Near(anchors[2], 0.8f));
}

void TestBuildRegistrationWarps() {
    // Two examples with one matched anchor each; canonical = mean = 0.5.
    const std::vector<pmg::TimeWarp> warps =
        pmg::BuildRegistrationWarps({{0.4f}, {0.6f}});
    assert(warps.size() == 2);
    assert(Near(warps[0].Evaluate(0.5f), 0.4f));
    assert(Near(warps[1].Evaluate(0.5f), 0.6f));
    assert(Near(warps[0].Evaluate(0.0f), 0.0f));
    assert(Near(warps[1].Evaluate(1.0f), 1.0f));
}

void TestAnchorlessExamplesGetIdentityWarps() {
    // No contacts anywhere (e.g. an in-air motion): identity registration.
    const std::vector<pmg::TimeWarp> warps = pmg::BuildRegistrationWarps({{}, {}});
    assert(warps.size() == 2);
    assert(warps[0].IsIdentity());
    assert(warps[1].IsIdentity());
}

void TestMismatchedContactStructureThrows() {
    bool threw = false;
    try {
        pmg::BuildRegistrationWarps({{0.4f}, {0.3f, 0.7f}});
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

}  // namespace

int main() {
    TestContactAnchorPhases();
    TestBuildRegistrationWarps();
    TestAnchorlessExamplesGetIdentityWarps();
    TestMismatchedContactStructureThrows();
    return 0;
}
