#include "pmg/MotionClip.h"

#include <cassert>
#include <cmath>

int main() {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;

    pmg::Pose start;
    start.root_position = {0.0f, 0.0f, 0.0f};
    start.local_rotations.push_back(pmg::Quaternion::Identity());

    pmg::Pose end;
    end.root_position = {10.0f, 0.0f, 0.0f};
    end.local_rotations.push_back(pmg::Quaternion::Identity());

    clip.frames.push_back(start);
    clip.frames.push_back(end);

    const pmg::Pose middle = clip.SampleNormalizedPhase(0.5f);
    assert(std::abs(middle.root_position.x - 5.0f) < 1.0e-5f);
    return 0;
}
