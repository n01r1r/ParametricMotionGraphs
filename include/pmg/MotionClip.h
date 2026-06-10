#pragma once

#include "pmg/Pose.h"

#include <string>
#include <vector>

namespace pmg {

struct MotionClip {
    std::string name;
    float frames_per_second = 30.0f;
    std::vector<Pose> frames;

    int NumFrames() const;
    int NumJoints() const;
    float DurationSeconds() const;

    Pose SampleByFrame(float frame_index) const;
    Pose SampleNormalizedPhase(float phase) const;

    void RequireNotEmpty(const char* context) const;
};

}  // namespace pmg
