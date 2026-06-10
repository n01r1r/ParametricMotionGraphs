#include "pmg/MotionRegistration.h"
#include "pmg/ParametricMotionSpace.h"

#include <cassert>
#include <cmath>

namespace {

// Root with one child "foot" joint at zero offset and identity rotations, so
// the foot's world position equals the pose's root position.
pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.name = "root";
    root.parent_index = -1;
    pmg::Joint foot;
    foot.name = "foot";
    foot.parent_index = 0;
    skeleton.joints = {root, foot};
    return skeleton;
}

pmg::Pose MakePose(const pmg::Vec3& position) {
    pmg::Pose pose;
    pose.root_position = position;
    pose.local_rotations = {pmg::Quaternion::Identity(), pmg::Quaternion::Identity()};
    return pose;
}

// One footstep over 30 frames: planted at x=0 until swing_first, a sinusoidal
// swing to x=2 through swing_last, planted at x=2 afterwards. Different
// (swing_first, swing_last) values shift the step timing within the clip.
pmg::MotionClip MakeStepClip(int swing_first, int swing_last) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;

    const int swing_span = swing_last - swing_first + 2;
    for (int frame = 0; frame < 30; ++frame) {
        pmg::Vec3 position;
        if (frame < swing_first) {
            position = {0.0f, 0.0f, 0.0f};
        } else if (frame <= swing_last) {
            const float swing =
                static_cast<float>(frame - swing_first + 1) / static_cast<float>(swing_span);
            position = {2.0f * swing, 0.5f * std::sin(swing * 3.14159265f), 0.0f};
        } else {
            position = {2.0f, 0.0f, 0.0f};
        }
        clip.frames.push_back(MakePose(position));
    }
    return clip;
}

pmg::ParametricMotionSpace MakeStepSpace() {
    // Same step, but the early example swings frames 6-15 and the late one
    // swings frames 14-23. Unregistered blends mix stance with swing.
    pmg::ParametricMotionSpace space("step", 1);
    space.AddExample({0.0f}, MakeStepClip(6, 15));
    space.AddExample({1.0f}, MakeStepClip(14, 23));
    return space;
}

void TestUnregisteredBlendFloatsTheFoot() {
    const pmg::ParametricMotionSpace space = MakeStepSpace();

    // Phase 10/29: the early example is mid-swing (foot high), the late one
    // is still planted. The naive blend leaves the foot floating mid-air.
    const pmg::Pose blended = space.EvaluatePose({0.5f}, 10.0f / 29.0f);
    assert(blended.root_position.y > 0.1f);
    assert(blended.root_position.y < 0.4f);
}

void TestRegisteredBlendPreservesContactsAndSwing() {
    pmg::ParametricMotionSpace space = MakeStepSpace();

    pmg::ContactDetectionSettings settings;
    settings.height_threshold = 0.1f;
    settings.speed_threshold = 0.5f;
    settings.min_contact_frames = 2;

    pmg::RegisterSpaceByContacts(space, MakeSkeleton(), {1}, settings);
    assert(space.HasExampleTimeWarps());

    // Early canonical phase: both examples planted at x=0 -> foot on floor.
    const pmg::Pose early = space.EvaluatePose({0.5f}, 0.05f);
    assert(early.root_position.y < 0.05f);
    assert(early.root_position.x < 0.1f);

    // Canonical mid-swing: both examples sample their own swing apex, so the
    // blend keeps the full step height instead of averaging against stance.
    const pmg::Pose apex = space.EvaluatePose({0.5f}, 0.5f);
    assert(apex.root_position.y > 0.4f);

    // Late canonical phase: both planted at x=2 -> contact restored, no slide.
    const pmg::Pose late = space.EvaluatePose({0.5f}, 0.95f);
    assert(late.root_position.y < 0.05f);
    assert(late.root_position.x > 1.9f);

    // Exact-parameter evaluation still reproduces the example clip exactly:
    // its warp maps canonical anchors back onto its own anchors monotonically,
    // and at parameter 0 only example 0 has weight.
    const pmg::Pose exact_start = space.EvaluatePose({0.0f}, 0.0f);
    assert(exact_start.root_position.y < 1.0e-5f);
    assert(std::abs(exact_start.root_position.x) < 1.0e-5f);
}

}  // namespace

int main() {
    TestUnregisteredBlendFloatsTheFoot();
    TestRegisteredBlendPreservesContactsAndSwing();
    return 0;
}
