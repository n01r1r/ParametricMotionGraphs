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

// Root motion clip: 11 frames, constant forward step of 0.1 per frame along
// the current heading, with heading advancing `turn_degrees_per_frame`.
pmg::MotionClip MakeTurnClip(float turn_degrees_per_frame) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;

    pmg::Vec3 position{0.0f, 0.0f, 0.0f};
    for (int frame = 0; frame < 11; ++frame) {
        const float heading =
            static_cast<float>(frame) * turn_degrees_per_frame * 3.14159265f / 180.0f;

        pmg::Pose pose;
        pose.root_position = position;
        pose.local_rotations = {
            pmg::Quaternion::FromAxisAngle({0.0f, 1.0f, 0.0f}, heading),
            pmg::Quaternion::Identity()};
        clip.frames.push_back(pose);

        position = position + pmg::Vec3{0.1f * std::sin(heading), 0.0f, 0.1f * std::cos(heading)};
    }
    return clip;
}

float HeadingOf(const pmg::Pose& pose) {
    const pmg::Vec3 forward = pmg::Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

// Blending a straight walk with a 9 deg/frame turn must produce an
// intermediate arc with full-length steps. Blending absolute root positions
// would shorten the steps (chord of the diverging directions) and bend the
// path toward the average of the two arcs instead.
void TestRootDeltaBlendKeepsStepLengthAndHeading() {
    pmg::ParametricMotionSpace space("turn", 1);
    space.AddExample({0.0f}, MakeTurnClip(0.0f));
    space.AddExample({1.0f}, MakeTurnClip(9.0f));

    const pmg::MotionClip blended = space.GenerateClip({0.5f}, 11, 30.0f);
    assert(blended.NumFrames() == 11);

    const float degrees = 3.14159265f / 180.0f;
    for (int frame = 1; frame < 11; ++frame) {
        const pmg::Vec3 step =
            blended.frames[frame].root_position - blended.frames[frame - 1].root_position;
        assert(std::abs(step.Norm() - 0.1f) < 1.0e-3f);

        const float heading = HeadingOf(blended.frames[frame]);
        assert(std::abs(heading - static_cast<float>(frame) * 4.5f * degrees) < 0.1f * degrees);
    }
}

}  // namespace

int main() {
    TestUnregisteredBlendFloatsTheFoot();
    TestRegisteredBlendPreservesContactsAndSwing();
    TestRootDeltaBlendKeepsStepLengthAndHeading();
    return 0;
}
