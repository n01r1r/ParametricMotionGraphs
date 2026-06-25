#include "pmg/MotionRegistration.h"

#include <cassert>
#include <cmath>
#include <cstddef>
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

void TestBlendableMotionFamilyAcceptsUniformCycles() {
    // Two cyclic examples (two intervals each), four anchors each: blendable.
    const std::vector<std::vector<pmg::ContactInterval>> intervals = {
        {{0, 3, 7}, {1, 18, 24}},
        {{0, 4, 8}, {1, 19, 25}},
    };
    pmg::RequireBlendableMotionFamily(intervals, {31, 31});  // no throw
}

void TestBlendableMotionFamilyRejectsAcyclicExample() {
    bool threw = false;
    try {
        // Second example has a single contact interval -> non-looping (vault).
        pmg::RequireBlendableMotionFamily(
            {{{0, 3, 7}, {1, 18, 24}}, {{0, 5, 9}}}, {31, 31});
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

void TestBlendableMotionFamilyRejectsStructureMismatch() {
    bool threw = false;
    try {
        // Both cyclic but different anchor counts (4 vs 6).
        pmg::RequireBlendableMotionFamily(
            {{{0, 3, 7}, {1, 18, 24}},
             {{0, 3, 7}, {1, 12, 16}, {0, 20, 26}}},
            {31, 31});
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

}  // namespace

void TestMatchedContactAnchorsTolerant() {
    // Example A: 4 contact intervals (an EXTRA joint-0 contact at phase 0.5/0.6).
    // Example B: 3 intervals. Legacy registration would throw on the count
    // mismatch; tolerant matching keeps the contacts common to both.
    const std::vector<std::vector<pmg::ContactInterval>> intervals = {
        {{0, 3, 6}, {0, 15, 18}, {1, 9, 12}},  // A: (0,s/l)=0.1/0.2 & 0.5/0.6, (1)=0.3/0.4
        {{0, 4, 7}, {1, 10, 13}},               // B: (0,s/l)=0.133/0.233, (1)=0.333/0.433
    };
    const std::vector<int> frame_counts = {31, 31};
    const std::vector<std::vector<float>> matched =
        pmg::MatchedContactAnchors(intervals, frame_counts);

    assert(matched.size() == 2);
    // Common structure = one (0,strike),(0,lift),(1,strike),(1,lift) each = 4.
    assert(matched[0].size() == 4);
    assert(matched[1].size() == 4);
    // Canonical order is by mean phase; A keeps its small-phase contacts and the
    // extra 0.5/0.6 is dropped (max stays 0.4).
    assert(Near(matched[0][0], 0.1f));
    assert(Near(matched[0][3], 0.4f));
    assert(matched[1][3] < 0.45f);
    // The result feeds BuildRegistrationWarps without throwing.
    const std::vector<pmg::TimeWarp> warps = pmg::BuildRegistrationWarps(matched);
    assert(warps.size() == 2);
}

void TestMatchedContactAnchorsHandlesOrderConflict() {
    // The two examples disagree on contact ORDER: in A joint-0 strikes before
    // joint-1; in B the reverse. Mean-phase order cannot be monotone in both, so
    // tolerant matching must DROP the conflicting anchors and still emit a
    // per-example strictly-increasing list -- otherwise BuildRegistrationWarps
    // throws "to anchors must be strictly increasing" (the failure real CMU
    // walks hit, which uniform synthetic data never exercised).
    const std::vector<std::vector<pmg::ContactInterval>> intervals = {
        {{0, 6, 9}, {1, 18, 21}},   // A: joint0 @0.2/0.3, joint1 @0.6/0.7
        {{0, 21, 24}, {1, 9, 12}},  // B: joint0 @0.7/0.8, joint1 @0.3/0.4
    };
    const std::vector<std::vector<float>> matched =
        pmg::MatchedContactAnchors(intervals, {31, 31});

    assert(matched.size() == 2);
    assert(matched[0].size() == matched[1].size());
    assert(!matched[0].empty());  // a common monotone chain survives
    for (const std::vector<float>& list : matched) {
        for (std::size_t i = 0; i + 1 < list.size(); ++i) {
            assert(list[i] < list[i + 1]);
        }
    }
    const std::vector<pmg::TimeWarp> warps = pmg::BuildRegistrationWarps(matched);
    assert(warps.size() == 2);
}

int main() {
    TestContactAnchorPhases();
    TestMatchedContactAnchorsTolerant();
    TestMatchedContactAnchorsHandlesOrderConflict();
    TestBuildRegistrationWarps();
    TestAnchorlessExamplesGetIdentityWarps();
    TestMismatchedContactStructureThrows();
    TestBlendableMotionFamilyAcceptsUniformCycles();
    TestBlendableMotionFamilyRejectsAcyclicExample();
    TestBlendableMotionFamilyRejectsStructureMismatch();
    return 0;
}
