#include "pmg/ParametricMotionSpace.h"

#include <cassert>
#include <cmath>

namespace {

pmg::MotionClip MakeClip(float root_x) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;

    pmg::Pose pose;
    pose.root_position = {root_x, 0.0f, 0.0f};
    pose.local_rotations.push_back(pmg::Quaternion::Identity());
    clip.frames.push_back(pose);
    return clip;
}

}  // namespace

int main() {
    pmg::ParametricMotionSpace space("test", 1);
    space.AddExample({0.0f}, MakeClip(0.0f));
    space.AddExample({1.0f}, MakeClip(10.0f));

    const pmg::Pose exact = space.EvaluatePose({1.0f}, 0.0f);
    assert(std::abs(exact.root_position.x - 10.0f) < 1.0e-5f);

    const pmg::Pose middle = space.EvaluatePose({0.5f}, 0.0f);
    assert(middle.root_position.x > 0.0f);
    assert(middle.root_position.x < 10.0f);
    return 0;
}
