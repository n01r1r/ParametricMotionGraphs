#include "pmg/legacy/FrameCountClipGeneration.h"

#include "pmg/ParametricMotionSpace.h"

#include <stdexcept>

namespace pmg::legacy {

MotionClip GenerateClipWithFrameCount(
    const ParametricMotionSpace& space,
    const ParameterVector& parameter,
    int frame_count,
    float frames_per_second) {
    if (frame_count <= 0) {
        throw std::runtime_error(
            "legacy::GenerateClipWithFrameCount: frame_count must be positive");
    }
    if (frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "legacy::GenerateClipWithFrameCount: frames_per_second must be positive");
    }

    const ParameterVector clamped_parameter = space.ClampToDomain(parameter);
    return space.GenerateClipFromWeights(
        space.ComputeLocalBlendWeights(clamped_parameter),
        frame_count,
        frames_per_second);
}

}  // namespace pmg::legacy
