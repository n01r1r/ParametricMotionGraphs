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
    int max_knots_per_segment = 3;  // interior warp knots added per anchor segment
};

// Refine installed contact-anchor warps with dynamic time warping. Contact
// anchors stay fixed; inside each canonical segment, the example closest to
// the parameter centroid serves as the timing reference and every other
// example is matched to it frame by frame (aligned point-cloud distance, DTW
// with matched segment endpoints). Up to max_knots_per_segment knots are
// inserted per segment so within-segment timing differences, e.g. swing-leg
// acceleration, line up instead of being interpolated linearly between
// contacts. Requires space.HasExampleTimeWarps(); no-op below two examples.
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
