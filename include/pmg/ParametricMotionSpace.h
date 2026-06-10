#pragma once

#include "pmg/MotionClip.h"
#include "pmg/ParameterVector.h"

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

    std::vector<float> ComputeInverseDistanceWeights(
        const ParameterVector& parameter) const;

    Pose EvaluatePose(const ParameterVector& parameter, float normalized_phase) const;

    MotionClip GenerateClip(
        const ParameterVector& parameter,
        int frame_count,
        float frames_per_second) const;

    std::vector<ParameterVector> ExampleParameters() const;
    std::vector<float> MinParameter() const;
    std::vector<float> MaxParameter() const;

private:
    std::string name_ = "unnamed_space";
    int parameter_dimension_ = 0;
    std::vector<ExampleMotion> examples_;
};

}  // namespace pmg
