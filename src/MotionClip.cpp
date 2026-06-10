#include "pmg/MotionClip.h"
#include "pmg/PoseBlend.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pmg {

int MotionClip::NumFrames() const {
    return static_cast<int>(frames.size());
}

int MotionClip::NumJoints() const {
    if (frames.empty()) {
        return 0;
    }
    return frames.front().NumJoints();
}

float MotionClip::DurationSeconds() const {
    RequireNotEmpty("MotionClip::DurationSeconds");
    if (frames_per_second <= 0.0f) {
        throw std::runtime_error("MotionClip::DurationSeconds: frames_per_second must be positive");
    }
    return static_cast<float>(NumFrames() - 1) / frames_per_second;
}

void MotionClip::RequireNotEmpty(const char* context) const {
    if (frames.empty()) {
        throw std::runtime_error(std::string(context) + ": motion clip is empty");
    }
}

Pose MotionClip::SampleByFrame(float frame_index) const {
    RequireNotEmpty("MotionClip::SampleByFrame");

    if (frames.size() == 1) {
        return frames.front();
    }

    frame_index = std::clamp(frame_index, 0.0f, static_cast<float>(frames.size() - 1));
    const int lower_frame = static_cast<int>(std::floor(frame_index));
    const int upper_frame = std::min(lower_frame + 1, NumFrames() - 1);
    const float alpha = frame_index - static_cast<float>(lower_frame);

    if (lower_frame == upper_frame) {
        return frames[lower_frame];
    }

    return BlendPose(frames[lower_frame], frames[upper_frame], alpha);
}

Pose MotionClip::SampleNormalizedPhase(float phase) const {
    RequireNotEmpty("MotionClip::SampleNormalizedPhase");
    phase = std::clamp(phase, 0.0f, 1.0f);
    const float frame_index = phase * static_cast<float>(NumFrames() - 1);
    return SampleByFrame(frame_index);
}

}  // namespace pmg
