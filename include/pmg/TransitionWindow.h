#pragma once

#include <vector>

namespace pmg {

// Interpretation of the source/target reference frames stored in a transition.
//
// Kovar directional:
//   source reference = first blend frame
//   target reference = last blend frame
//
// PMG centered:
//   both references = center of their blend windows
enum class TransitionWindowConvention {
    kKovarDirectional,
    kPmgCentered,
};

struct FrameWindow {
    // Requested support before endpoint clamping.
    int first_unclamped_frame = 0;
    int last_unclamped_frame = 0;
    // First and last sampled frames after endpoint clamping.
    int first_frame = 0;
    int last_frame = 0;
    std::vector<int> sampled_frames;
};

struct TransitionFrameWindows {
    FrameWindow source;
    FrameWindow target;
};

// Resolve the exact sampled frame indices for one transition. Out-of-range
// indices are endpoint-clamped, matching MotionDistance point-cloud sampling.
TransitionFrameWindows ResolveTransitionFrameWindows(
    int source_frame_count,
    int target_frame_count,
    int source_reference_frame,
    int target_reference_frame,
    int window_size,
    TransitionWindowConvention convention);

// k sampled frames cover k-1 frame intervals.
float TransitionWindowSpanSeconds(int window_size, float frames_per_second);

const char* TransitionWindowConventionName(
    TransitionWindowConvention convention);

}  // namespace pmg
