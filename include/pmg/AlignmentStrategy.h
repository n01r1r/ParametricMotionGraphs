#pragma once

#include "pmg/MotionClip.h"
#include "pmg/ParametricMotionGraph.h"  // InterpolatedTransition
#include "pmg/RigidTransform2D.h"
#include "pmg/Skeleton.h"
#include "pmg/TransitionWindow.h"

namespace pmg {

// Everything a strategy needs to resolve the target->source rigid floor
// transform at a scheduled transition. `live_source_phase` is the controller's
// CurrentPhase() at schedule time; `transition` carries the interpolated
// source/target transition phases. Alignment is intentionally recomputed at
// runtime and is not stored on PMG edge samples.
struct AlignmentContext {
    const MotionClip& source_clip;
    const MotionClip& target_clip;
    float live_source_phase;
    const InterpolatedTransition& transition;
    int window_size = 5;
    TransitionWindowConvention convention =
        TransitionWindowConvention::kKovarDirectional;
};

// Seam: how a RuntimeController aligns the chosen target clip onto the current
// clip at a transition.
class AlignmentStrategy {
public:
    virtual ~AlignmentStrategy() = default;
    virtual RigidTransform2D Resolve(const AlignmentContext& context) const = 0;
};

// Paper-faithful (PMG §3.2 / §5.2.1): recompute the point-cloud alignment
// between the current and chosen target clips at the selected transition phases.
class PointCloudAlignment : public AlignmentStrategy {
public:
    explicit PointCloudAlignment(const Skeleton& skeleton, int window = 5)
        : skeleton_(&skeleton), window_(window) {}
    RigidTransform2D Resolve(const AlignmentContext& context) const override;

private:
    const Skeleton* skeleton_;
    int window_;
};

// Legacy/debug: root-only alignment recomputed from the root pose alone. This is
// not the paper path; it remains useful for synthetic tests with no Skeleton.
class RootOnlyAlignment : public AlignmentStrategy {
public:
    RigidTransform2D Resolve(const AlignmentContext& context) const override;
};

}  // namespace pmg
