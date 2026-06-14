#include "pmg/TransitionWindow.h"

#include <algorithm>
#include <stdexcept>

namespace pmg {

namespace {

FrameWindow ResolveFrameWindow(
    int frame_count,
    int first_frame,
    int window_size) {
    if (frame_count <= 0) {
        throw std::runtime_error(
            "ResolveTransitionFrameWindows: frame count must be positive");
    }
    if (window_size <= 0) {
        throw std::runtime_error(
            "ResolveTransitionFrameWindows: window_size must be positive");
    }

    FrameWindow window;
    window.first_unclamped_frame = first_frame;
    window.last_unclamped_frame = first_frame + window_size - 1;
    window.sampled_frames.reserve(static_cast<std::size_t>(window_size));
    for (int offset = 0; offset < window_size; ++offset) {
        window.sampled_frames.push_back(
            std::clamp(first_frame + offset, 0, frame_count - 1));
    }
    window.first_frame = window.sampled_frames.front();
    window.last_frame = window.sampled_frames.back();
    return window;
}

}  // namespace

TransitionFrameWindows ResolveTransitionFrameWindows(
    int source_frame_count,
    int target_frame_count,
    int source_reference_frame,
    int target_reference_frame,
    int window_size,
    TransitionWindowConvention convention) {
    int source_first_frame = 0;
    int target_first_frame = 0;
    switch (convention) {
        case TransitionWindowConvention::kKovarDirectional:
            source_first_frame = source_reference_frame;
            target_first_frame = target_reference_frame - window_size + 1;
            break;
        case TransitionWindowConvention::kPmgCentered: {
            const int half_window = window_size / 2;
            source_first_frame = source_reference_frame - half_window;
            target_first_frame = target_reference_frame - half_window;
            break;
        }
        default:
            throw std::runtime_error(
                "ResolveTransitionFrameWindows: unsupported convention");
    }

    return {
        ResolveFrameWindow(
            source_frame_count, source_first_frame, window_size),
        ResolveFrameWindow(
            target_frame_count, target_first_frame, window_size),
    };
}

float TransitionWindowSpanSeconds(
    int window_size,
    float frames_per_second) {
    if (window_size <= 0) {
        throw std::runtime_error(
            "TransitionWindowSpanSeconds: window_size must be positive");
    }
    if (frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "TransitionWindowSpanSeconds: frames_per_second must be positive");
    }
    return static_cast<float>(window_size - 1) / frames_per_second;
}

const char* TransitionWindowConventionName(
    TransitionWindowConvention convention) {
    switch (convention) {
        case TransitionWindowConvention::kKovarDirectional:
            return "kovar_directional";
        case TransitionWindowConvention::kPmgCentered:
            return "pmg_centered";
    }
    throw std::runtime_error(
        "TransitionWindowConventionName: unsupported convention");
}

}  // namespace pmg
