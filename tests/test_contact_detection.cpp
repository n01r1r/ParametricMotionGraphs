#include "pmg/ContactDetection.h"

#include <cassert>
#include <cmath>
#include <stdexcept>

namespace {

// Root with one child "foot" joint at zero offset and identity rotations, so
// the foot's world position equals the root position frame by frame.
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

// Stance (frames 0-9, planted at x=0), swing (frames 10-19, foot travels to
// x=2 with a sinusoidal height bump), stance (frames 20-29, planted at x=2).
pmg::MotionClip MakeStepClip() {
    pmg::MotionClip clip;
    clip.name = "step";
    clip.frames_per_second = 30.0f;

    for (int frame = 0; frame < 30; ++frame) {
        pmg::Vec3 position;
        if (frame < 10) {
            position = {0.0f, 0.0f, 0.0f};
        } else if (frame < 20) {
            const float swing = static_cast<float>(frame - 9) / 11.0f;
            position = {2.0f * swing, 0.5f * std::sin(swing * 3.14159265f), 0.0f};
        } else {
            position = {2.0f, 0.0f, 0.0f};
        }
        clip.frames.push_back(MakePose(position));
    }
    return clip;
}

void TestDetectTwoContacts() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeStepClip();

    pmg::ContactDetectionSettings settings;
    settings.height_threshold = 0.1f;
    settings.speed_threshold = 0.5f;
    settings.min_contact_frames = 2;

    const std::vector<pmg::ContactInterval> intervals =
        pmg::DetectContacts(skeleton, clip, {1}, settings);

    assert(intervals.size() == 2);
    assert(intervals[0].joint_index == 1);
    assert(intervals[1].joint_index == 1);

    // First stance starts at the clip start; the boundary frame may fall out
    // because central-difference speed already sees the swing.
    assert(intervals[0].first_frame == 0);
    assert(intervals[0].last_frame >= 7 && intervals[0].last_frame <= 9);

    // Second stance runs to the clip end.
    assert(intervals[1].first_frame >= 20 && intervals[1].first_frame <= 22);
    assert(intervals[1].last_frame == 29);

    // Phases follow frames.
    const int frame_count = clip.NumFrames();
    assert(std::abs(intervals[0].StrikePhase(frame_count)) < 1.0e-6f);
    assert(std::abs(intervals[1].LiftPhase(frame_count) - 1.0f) < 1.0e-6f);
    const float second_strike = intervals[1].StrikePhase(frame_count);
    assert(second_strike > 0.6f && second_strike < 0.8f);
}

void TestEstimatedSettings() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeStepClip();

    const pmg::ContactDetectionSettings estimated =
        pmg::EstimateContactSettings(skeleton, clip, {1});
    assert(estimated.height_threshold > 0.0f);
    assert(estimated.speed_threshold > 0.0f);

    const std::vector<pmg::ContactInterval> intervals =
        pmg::DetectContacts(skeleton, clip, {1}, estimated);
    assert(intervals.size() == 2);
}

void TestMinContactFramesFilter() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeStepClip();

    pmg::ContactDetectionSettings settings;
    settings.height_threshold = 0.1f;
    settings.speed_threshold = 0.5f;
    settings.min_contact_frames = 100;  // longer than any run

    const std::vector<pmg::ContactInterval> intervals =
        pmg::DetectContacts(skeleton, clip, {1}, settings);
    assert(intervals.empty());
}

void TestValidation() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::MotionClip clip = MakeStepClip();
    const pmg::ContactDetectionSettings settings;

    bool threw = false;
    try {
        pmg::DetectContacts(skeleton, clip, {}, settings);  // no joints
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);

    threw = false;
    try {
        pmg::DetectContacts(skeleton, clip, {5}, settings);  // bad joint index
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

}  // namespace

int main() {
    TestDetectTwoContacts();
    TestEstimatedSettings();
    TestMinContactFramesFilter();
    TestValidation();
    return 0;
}
