#pragma once

#include "pmg/MotionClip.h"
#include "pmg/ParameterVector.h"
#include "pmg/ParameterDomain.h"
#include "pmg/TimeWarp.h"

#include <string>
#include <vector>

namespace pmg {

struct ExampleMotion {
    ParameterVector parameter;
    MotionClip clip;
};

class ParametricMotionSpace {
public:
    ParametricMotionSpace() = default;
    ParametricMotionSpace(std::string space_name, int parameter_dimension);

    const std::string& Name() const;
    int ParameterDimension() const;
    int NumExamples() const;

    void AddExample(const ParameterVector& parameter, MotionClip clip);

    // Registration warps, one per example, mapping the space's canonical
    // phase onto each example's own phase (see MotionRegistration). When set,
    // EvaluatePose samples example i at warps[i](phase) so blends combine
    // structurally corresponding moments instead of the same raw phase.
    void SetExampleTimeWarps(std::vector<TimeWarp> warps);
    void ClearExampleTimeWarps();
    bool HasExampleTimeWarps() const;
    // Empty when unregistered; otherwise one warp per example.
    const std::vector<TimeWarp>& ExampleTimeWarps() const;

    std::vector<float> ComputeLocalBlendWeights(
        const ParameterVector& parameter) const;

    Pose EvaluatePose(const ParameterVector& parameter, float normalized_phase) const;

    // Generates a clip by blending joints/height per frame (EvaluatePose) but
    // integrating the root's floor motion from blended per-frame root deltas
    // expressed in each example's own heading frame. Blending absolute root
    // positions would average arcs of different curvature into a distorted
    // path and drag planted feet sideways; delta integration follows an
    // intermediate arc instead.
    MotionClip GenerateClip(
        const ParameterVector& parameter,
        int frame_count,
        float frames_per_second) const;

    std::vector<ParameterVector> ExampleParameters() const;
    const std::vector<ExampleMotion>& Examples() const;
    std::vector<float> MinParameter() const;
    std::vector<float> MaxParameter() const;
    ParameterDomain Domain() const;
    ParameterVector ClampToDomain(const ParameterVector& parameter) const;

private:
    std::string name_ = "unnamed_space";
    int parameter_dimension_ = 0;
    std::vector<ExampleMotion> examples_;
    // Empty = unregistered (every example sampled at the raw phase).
    std::vector<TimeWarp> example_time_warps_;
};

}  // namespace pmg
