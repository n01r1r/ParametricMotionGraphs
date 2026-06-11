#pragma once

#include "pmg/MotionClip.h"
#include "pmg/ParameterVector.h"

namespace pmg {

class ParametricMotionSpace;

namespace legacy {

// Generates a frame-aligned diagnostic clip with caller-selected length.
// This bypasses duration-derived timing and must not be used for runtime
// playback or paper-path edge construction.
MotionClip GenerateClipWithFrameCount(
    const ParametricMotionSpace& space,
    const ParameterVector& parameter,
    int frame_count,
    float frames_per_second);

}  // namespace legacy
}  // namespace pmg
