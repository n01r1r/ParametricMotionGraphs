#pragma once

#include "pmg/MotionClip.h"
#include "pmg/ParametricMotionGraph.h"  // InterpolatedTransition
#include "pmg/RigidTransform2D.h"
#include "pmg/Skeleton.h"

namespace pmg {

// Everything a strategy needs to resolve the target->source rigid floor
// transform at a scheduled transition. `live_source_phase` is the controller's
// CurrentPhase() at schedule time; `transition` carries the (interpolated)
// source/target transition phases and the stored alignment.
struct AlignmentContext {
    const MotionClip& source_clip;
    const MotionClip& target_clip;
    float live_source_phase;
    const InterpolatedTransition& transition;
};

// Seam: how a RuntimeController aligns the chosen target clip onto the current
// clip at a transition. Three adapters live behind it; pass one at construction.
class AlignmentStrategy {
public:
    virtual ~AlignmentStrategy() = default;
    virtual RigidTransform2D Resolve(const AlignmentContext& context) const = 0;
};

// Paper-faithful (paper Sec 3.2 / Sec 5.2.1): recompute the exact point-cloud
// alignment between the current and chosen target clips at the transition point.
class PointCloudAlignment : public AlignmentStrategy {
public:
    explicit PointCloudAlignment(const Skeleton& skeleton, int window = 5)
        : skeleton_(&skeleton), window_(window) {}
    RigidTransform2D Resolve(const AlignmentContext& context) const override;

private:
    const Skeleton* skeleton_;
    int window_;
};

// Use the (averaged) alignment baked onto the edge sample at build time.
class StoredAlignment : public AlignmentStrategy {
public:
    RigidTransform2D Resolve(const AlignmentContext& context) const override;
};

// Legacy/debug: root-only alignment recomputed from the root pose alone.
class RootOnlyAlignment : public AlignmentStrategy {
public:
    RigidTransform2D Resolve(const AlignmentContext& context) const override;
};

}  // namespace pmg
