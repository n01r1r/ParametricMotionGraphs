#pragma once

#include "pmg/MotionClip.h"

namespace pmg {

inline Pose SampleClipNormalizedPhase(const MotionClip& clip, float phase) {
    return clip.SampleNormalizedPhase(phase);
}

}  // namespace pmg
