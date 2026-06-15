#pragma once

#include "pmg/ContactDetection.h"
#include "pmg/ParametricMotionSpace.h"
#include "pmg/TimeWarp.h"

#include <vector>

namespace pmg {

// Interior anchor phases of a clip's contact structure: the strike and lift
// phase of every contact interval, sorted, with phases at exactly 0 or 1
// dropped (those are already pinned by TimeWarp) and duplicates collapsed.
std::vector<float> ContactAnchorPhases(
    const std::vector<ContactInterval>& intervals,
    int clip_frame_count);

// Build one TimeWarp per example from matched anchor lists. Every example
// must expose the same number of anchors (same contact structure, e.g. all
// walk cycles). The canonical domain places each anchor at the mean of the
// examples' phases; warp[i] maps canonical phase -> example i phase.
std::vector<TimeWarp> BuildRegistrationWarps(
    const std::vector<std::vector<float>>& example_anchor_phases);

// Enforce the motion-family premise for a parametric blend space. Heck-Gleicher
// and KG04 blend only structurally similar, registered example motions, so a
// blend space must hold a single motion family. Each example must be one looping
// cycle (at least two contact intervals -- a non-looping motion such as a vault
// has too few) and every example must expose the same ContactAnchorPhases count.
// Throws std::runtime_error naming the offending example otherwise. Different
// motion families belong in separate graph nodes joined by a transition edge,
// not as examples of one blend space.
//
// Pure structural overload: takes each example's detected contact intervals and
// frame count, so it is checkable without forward kinematics.
void RequireBlendableMotionFamily(
    const std::vector<std::vector<ContactInterval>>& example_intervals,
    const std::vector<int>& example_frame_counts);

// Convenience overload: detects contacts on every example, then applies the
// structural check above. Used by motion-space preparation and interactive
// authoring so both reject cross-family blends the same way.
void RequireBlendableMotionFamily(
    const Skeleton& skeleton,
    const std::vector<MotionClip>& examples,
    const std::vector<int>& contact_joints,
    const ContactDetectionSettings& settings);

// Convenience: detect contacts on every example of `space`, derive anchors,
// and install the registration warps so EvaluatePose blends structurally
// corresponding moments. Throws if the examples' contact structures differ.
void RegisterSpaceByContacts(
    ParametricMotionSpace& space,
    const Skeleton& skeleton,
    const std::vector<int>& contact_joints,
    const ContactDetectionSettings& settings);

struct DtwRefineSettings {
    int window_size = 5;            // frames per point cloud in the DTW cost
    int max_knots_per_segment = 8;  // interior warp knots sampled per anchor segment
    float smoothing_strength = 4.0f;  // cubic smoothing-spline penalty (lambda)
                                      // applied to the DTW correspondence; 0
                                      // keeps the raw staircase correspondence
};

// Refine installed contact-anchor warps with dynamic time warping. Contact
// anchors stay fixed; inside each canonical segment, the example closest to
// the parameter centroid serves as the timing reference and every other
// example is matched to it frame by frame (aligned point-cloud distance, DTW
// with matched segment endpoints). The per-frame DTW correspondence is then
// denoised with a cubic smoothing spline (KG04-style registration curve,
// smoothing_strength = lambda) and sampled into up to max_knots_per_segment
// interior knots, so within-segment timing differences (e.g. swing-leg
// acceleration) line up along a smooth monotone curve instead of the raw DTW
// steps or a straight line between contacts. Requires
// space.HasExampleTimeWarps(); no-op below two examples.
void RefineRegistrationByDtw(
    ParametricMotionSpace& space,
    const Skeleton& skeleton,
    const DtwRefineSettings& settings = {});

// Cut `clip` at the strikes of `cycle_joint` and return the first complete
// gait cycle (strike to next strike, inclusive). Normalizes multi-cycle
// corpus clips into single-cycle examples so their contact structures match
// across a motion space. Throws if fewer than two strikes are found.
MotionClip ExtractFirstCycle(
    const Skeleton& skeleton,
    const MotionClip& clip,
    int cycle_joint,
    const ContactDetectionSettings& settings);

}  // namespace pmg
