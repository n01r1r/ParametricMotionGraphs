#include "pmg/ParametricMotionSpace.h"
#include "pmg/PoseBlend.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pmg {

ParametricMotionSpace::ParametricMotionSpace(std::string space_name, int parameter_dimension)
    : name_(std::move(space_name)), parameter_dimension_(parameter_dimension) {
    if (parameter_dimension_ <= 0) {
        throw std::runtime_error("ParametricMotionSpace: parameter_dimension must be positive");
    }
}

const std::string& ParametricMotionSpace::Name() const {
    return name_;
}

int ParametricMotionSpace::ParameterDimension() const {
    return parameter_dimension_;
}

int ParametricMotionSpace::NumExamples() const {
    return static_cast<int>(examples_.size());
}

void ParametricMotionSpace::AddExample(const ParameterVector& parameter, MotionClip clip) {
    if (static_cast<int>(parameter.size()) != parameter_dimension_) {
        throw std::runtime_error("ParametricMotionSpace::AddExample: parameter dimension mismatch");
    }
    clip.RequireNotEmpty("ParametricMotionSpace::AddExample");

    if (!examples_.empty() && clip.NumJoints() != examples_.front().clip.NumJoints()) {
        throw std::runtime_error("ParametricMotionSpace::AddExample: example joint count mismatch");
    }

    examples_.push_back({parameter, std::move(clip)});
}

std::vector<float> ParametricMotionSpace::ComputeInverseDistanceWeights(
    const ParameterVector& parameter) const {
    if (static_cast<int>(parameter.size()) != parameter_dimension_) {
        throw std::runtime_error("ParametricMotionSpace::ComputeInverseDistanceWeights: parameter dimension mismatch");
    }
    if (examples_.empty()) {
        throw std::runtime_error("ParametricMotionSpace::ComputeInverseDistanceWeights: no examples available");
    }

    std::vector<float> weights(examples_.size(), 0.0f);

    constexpr float kExactParameterThreshold = 1.0e-6f;
    for (std::size_t example_index = 0; example_index < examples_.size(); ++example_index) {
        const float distance = Distance(parameter, examples_[example_index].parameter);
        if (distance <= kExactParameterThreshold) {
            weights.assign(examples_.size(), 0.0f);
            weights[example_index] = 1.0f;
            return weights;
        }
    }

    float weight_sum = 0.0f;
    for (std::size_t example_index = 0; example_index < examples_.size(); ++example_index) {
        const float distance = Distance(parameter, examples_[example_index].parameter);
        weights[example_index] = 1.0f / std::max(distance, kExactParameterThreshold);
        weight_sum += weights[example_index];
    }

    if (weight_sum <= kSmallEpsilon) {
        throw std::runtime_error("ParametricMotionSpace::ComputeInverseDistanceWeights: invalid weight sum");
    }

    for (float& weight : weights) {
        weight /= weight_sum;
    }

    return weights;
}

Pose ParametricMotionSpace::EvaluatePose(
    const ParameterVector& parameter,
    float normalized_phase) const {
    const std::vector<float> weights = ComputeInverseDistanceWeights(parameter);

    std::vector<Pose> poses;
    poses.reserve(examples_.size());
    for (const ExampleMotion& example : examples_) {
        poses.push_back(example.clip.SampleNormalizedPhase(normalized_phase));
    }

    return BlendPoseN(poses, weights);
}

MotionClip ParametricMotionSpace::GenerateClip(
    const ParameterVector& parameter,
    int frame_count,
    float frames_per_second) const {
    if (frame_count <= 0) {
        throw std::runtime_error("ParametricMotionSpace::GenerateClip: frame_count must be positive");
    }
    if (frames_per_second <= 0.0f) {
        throw std::runtime_error("ParametricMotionSpace::GenerateClip: frames_per_second must be positive");
    }

    MotionClip generated_clip;
    generated_clip.name = name_ + "_generated";
    generated_clip.frames_per_second = frames_per_second;
    generated_clip.frames.reserve(static_cast<std::size_t>(frame_count));

    for (int frame_index = 0; frame_index < frame_count; ++frame_index) {
        const float phase =
            frame_count == 1
                ? 0.0f
                : static_cast<float>(frame_index) / static_cast<float>(frame_count - 1);
        generated_clip.frames.push_back(EvaluatePose(parameter, phase));
    }

    return generated_clip;
}

std::vector<ParameterVector> ParametricMotionSpace::ExampleParameters() const {
    std::vector<ParameterVector> parameters;
    parameters.reserve(examples_.size());
    for (const ExampleMotion& example : examples_) {
        parameters.push_back(example.parameter);
    }
    return parameters;
}

std::vector<float> ParametricMotionSpace::MinParameter() const {
    if (examples_.empty()) {
        throw std::runtime_error("ParametricMotionSpace::MinParameter: no examples available");
    }

    ParameterVector minimum(parameter_dimension_, std::numeric_limits<float>::infinity());
    for (const ExampleMotion& example : examples_) {
        for (int dimension = 0; dimension < parameter_dimension_; ++dimension) {
            minimum[dimension] = std::min(minimum[dimension], example.parameter[dimension]);
        }
    }
    return minimum;
}

std::vector<float> ParametricMotionSpace::MaxParameter() const {
    if (examples_.empty()) {
        throw std::runtime_error("ParametricMotionSpace::MaxParameter: no examples available");
    }

    ParameterVector maximum(parameter_dimension_, -std::numeric_limits<float>::infinity());
    for (const ExampleMotion& example : examples_) {
        for (int dimension = 0; dimension < parameter_dimension_; ++dimension) {
            maximum[dimension] = std::max(maximum[dimension], example.parameter[dimension]);
        }
    }
    return maximum;
}

}  // namespace pmg
