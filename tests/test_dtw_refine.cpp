#include "pmg/MotionRegistration.h"
#include "pmg/ParametricMotionSpace.h"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {

constexpr int kFrameCount = 41;

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.name = "root";
    root.parent_index = -1;
    pmg::Joint child;
    child.name = "child";
    child.parent_index = 0;
    skeleton.joints = {root, child};
    return skeleton;
}

// One underlying motion, root height = s for s in [0, 1]. Height is immune to
// the metric's yaw + floor-translation alignment, so DTW must match frames of
// equal height. `timing(phase)` re-times how the clip plays the motion.
template <typename Timing>
pmg::MotionClip MakeClip(const char* name, Timing timing) {
    pmg::MotionClip clip;
    clip.name = name;
    clip.frames_per_second = 30.0f;
    for (int frame = 0; frame < kFrameCount; ++frame) {
        const float phase = static_cast<float>(frame) / static_cast<float>(kFrameCount - 1);
        pmg::Pose pose;
        pose.root_position = {0.0f, timing(phase), 0.0f};
        pose.local_rotations = {pmg::Quaternion::Identity(), pmg::Quaternion::Identity()};
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::ParametricMotionSpace MakeSpace() {
    pmg::ParametricMotionSpace space("retimed", 1);
    // Reference plays the motion uniformly; the second example plays it slow
    // then fast (height = phase^2), so true correspondence is sqrt(canonical).
    space.AddExample({0.0f}, MakeClip("uniform", [](float phase) { return phase; }));
    space.AddExample({1.0f}, MakeClip("retimed", [](float phase) { return phase * phase; }));
    return space;
}

void TestRequiresInstalledWarps() {
    pmg::ParametricMotionSpace space = MakeSpace();
    bool threw = false;
    try {
        pmg::RefineRegistrationByDtw(space, MakeSkeleton(), {});
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

void TestRecoversNonlinearTiming() {
    pmg::ParametricMotionSpace space = MakeSpace();
    // No contact anchors: a single canonical segment spanning the whole clip.
    space.SetExampleTimeWarps({pmg::TimeWarp(), pmg::TimeWarp()});

    pmg::DtwRefineSettings settings;
    settings.window_size = 3;
    pmg::RefineRegistrationByDtw(space, MakeSkeleton(), settings);

    const std::vector<pmg::TimeWarp>& warps = space.ExampleTimeWarps();
    // Centroid is equidistant from both examples; the first wins the tie and
    // keeps its (identity) warp as the timing reference.
    assert(warps[0].IsIdentity());
    assert(!warps[1].IsIdentity());

    // The refined warp should map canonical phase c near sqrt(c), where the
    // re-timed example reaches the same height as the reference.
    assert(std::abs(warps[1].Evaluate(0.5f) - std::sqrt(0.5f)) < 0.05f);
    assert(std::abs(warps[1].Evaluate(0.25f) - 0.5f) < 0.05f);
    assert(std::abs(warps[1].Evaluate(0.0f)) < 1.0e-6f);
    assert(std::abs(warps[1].Evaluate(1.0f) - 1.0f) < 1.0e-6f);

    // Blending at the midpoint now combines frames of equal height instead of
    // averaging heights 0.5 and 0.25.
    const pmg::Pose blended = space.EvaluatePose({0.5f}, 0.5f);
    assert(std::abs(blended.root_position.y - 0.5f) < 0.05f);
}

void TestContactAnchorsStayPinned() {
    pmg::ParametricMotionSpace space = MakeSpace();
    // Deliberately pin canonical 0.5 to example phase 0.5 in both warps, even
    // though DTW would prefer sqrt(0.5) for the re-timed example. Anchors are
    // hard constraints; refinement may only move timing between them.
    space.SetExampleTimeWarps({
        pmg::TimeWarp::FromAnchors({0.5f}, {0.5f}),
        pmg::TimeWarp::FromAnchors({0.5f}, {0.5f}),
    });

    pmg::DtwRefineSettings settings;
    settings.window_size = 3;
    pmg::RefineRegistrationByDtw(space, MakeSkeleton(), settings);

    const std::vector<pmg::TimeWarp>& warps = space.ExampleTimeWarps();
    assert(std::abs(warps[0].Evaluate(0.5f) - 0.5f) < 1.0e-6f);
    assert(std::abs(warps[1].Evaluate(0.5f) - 0.5f) < 1.0e-6f);
    // Interior of each segment still re-times monotonically.
    for (const float phase : {0.1f, 0.3f, 0.45f, 0.55f, 0.7f, 0.9f}) {
        const float warped = warps[1].Evaluate(phase);
        assert(warped >= 0.0f && warped <= 1.0f);
    }
    assert(warps[1].Evaluate(0.3f) > warps[1].Evaluate(0.1f));
    assert(warps[1].Evaluate(0.9f) > warps[1].Evaluate(0.55f));
}

void TestSingleExampleIsNoOp() {
    pmg::ParametricMotionSpace space("solo", 1);
    space.AddExample({0.0f}, MakeClip("uniform", [](float phase) { return phase; }));
    space.SetExampleTimeWarps({pmg::TimeWarp()});
    pmg::RefineRegistrationByDtw(space, MakeSkeleton(), {});
    assert(space.ExampleTimeWarps().size() == 1);
    assert(space.ExampleTimeWarps()[0].IsIdentity());
}

}  // namespace

int main() {
    TestRequiresInstalledWarps();
    TestRecoversNonlinearTiming();
    TestContactAnchorsStayPinned();
    TestSingleExampleIsNoOp();
    return 0;
}
