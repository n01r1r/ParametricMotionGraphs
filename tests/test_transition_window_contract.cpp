#include "pmg/TransitionWindow.h"

#include <cassert>
#include <cmath>
#include <stdexcept>

int main() {
    constexpr int kSourceFrames = 20;
    constexpr int kTargetFrames = 30;
    constexpr int kSourceReference = 8;
    constexpr int kTargetReference = 12;
    constexpr int kWindowSize = 5;

    const pmg::TransitionFrameWindows directional =
        pmg::ResolveTransitionFrameWindows(
            kSourceFrames, kTargetFrames,
            kSourceReference, kTargetReference, kWindowSize,
            pmg::TransitionWindowConvention::kKovarDirectional);
    assert(directional.source.sampled_frames ==
           (std::vector<int>{8, 9, 10, 11, 12}));
    assert(directional.target.sampled_frames ==
           (std::vector<int>{8, 9, 10, 11, 12}));

    const pmg::TransitionFrameWindows centered =
        pmg::ResolveTransitionFrameWindows(
            kSourceFrames, kTargetFrames,
            kSourceReference, kTargetReference, kWindowSize,
            pmg::TransitionWindowConvention::kPmgCentered);
    assert(centered.source.sampled_frames ==
           (std::vector<int>{6, 7, 8, 9, 10}));
    assert(centered.target.sampled_frames ==
           (std::vector<int>{10, 11, 12, 13, 14}));

    // Regression for the former runtime mismatch: stored directional
    // references select different support if reinterpreted as centered points.
    assert(directional.source.sampled_frames !=
           centered.source.sampled_frames);
    assert(directional.target.sampled_frames !=
           centered.target.sampled_frames);

    const pmg::TransitionFrameWindows clamped =
        pmg::ResolveTransitionFrameWindows(
            /*source_frame_count=*/3, /*target_frame_count=*/3,
            /*source_reference_frame=*/0, /*target_reference_frame=*/0,
            kWindowSize,
            pmg::TransitionWindowConvention::kPmgCentered);
    assert(clamped.source.sampled_frames ==
           (std::vector<int>{0, 0, 0, 1, 2}));
    assert(clamped.target.sampled_frames ==
           (std::vector<int>{0, 0, 0, 1, 2}));

    assert(std::abs(
               pmg::TransitionWindowSpanSeconds(
                   /*window_size=*/5, /*frames_per_second=*/30.0f) -
               (4.0f / 30.0f)) <
           1.0e-6f);

    bool rejected_invalid_rate = false;
    try {
        (void)pmg::TransitionWindowSpanSeconds(5, 0.0f);
    } catch (const std::runtime_error&) {
        rejected_invalid_rate = true;
    }
    assert(rejected_invalid_rate);

    bool rejected_invalid_convention = false;
    try {
        (void)pmg::ResolveTransitionFrameWindows(
            kSourceFrames, kTargetFrames,
            kSourceReference, kTargetReference, kWindowSize,
            static_cast<pmg::TransitionWindowConvention>(99));
    } catch (const std::runtime_error&) {
        rejected_invalid_convention = true;
    }
    assert(rejected_invalid_convention);

    return 0;
}
