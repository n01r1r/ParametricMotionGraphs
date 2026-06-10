#include "pmg/ParametricMotionSpace.h"
#include "pmg/PoseBlend.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace pmg {

namespace {

constexpr float kExactParameterThreshold = 1.0e-6f;

}  // namespace

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
    if (!example_time_warps_.empty()) {
        throw std::runtime_error(
            "ParametricMotionSpace::AddExample: add all examples before registering time warps");
    }

    examples_.push_back({parameter, std::move(clip)});
}

void ParametricMotionSpace::SetExampleTimeWarps(std::vector<TimeWarp> warps) {
    if (static_cast<int>(warps.size()) != NumExamples()) {
        throw std::runtime_error(
            "ParametricMotionSpace::SetExampleTimeWarps: need exactly one warp per example");
    }
    example_time_warps_ = std::move(warps);
}

void ParametricMotionSpace::ClearExampleTimeWarps() {
    example_time_warps_.clear();
}

bool ParametricMotionSpace::HasExampleTimeWarps() const {
    return !example_time_warps_.empty();
}

std::vector<float> ParametricMotionSpace::ComputeLocalBlendWeights(
    const ParameterVector& parameter) const {
    if (static_cast<int>(parameter.size()) != parameter_dimension_) {
        throw std::runtime_error("ParametricMotionSpace::ComputeLocalBlendWeights: parameter dimension mismatch");
    }
    if (examples_.empty()) {
        throw std::runtime_error("ParametricMotionSpace::ComputeLocalBlendWeights: no examples available");
    }

    std::vector<float> weights(examples_.size(), 0.0f);
    std::vector<std::size_t> order(examples_.size());
    std::vector<float> distances(examples_.size(), 0.0f);
    std::iota(order.begin(), order.end(), std::size_t{0});

    for (std::size_t example_index = 0; example_index < examples_.size(); ++example_index) {
        distances[example_index] = Distance(parameter, examples_[example_index].parameter);
        if (distances[example_index] <= kExactParameterThreshold) {
            weights[example_index] = 1.0f;
            return weights;
        }
    }

    std::sort(order.begin(), order.end(), [&](std::size_t left, std::size_t right) {
        return distances[left] < distances[right];
    });

    // Local interpolation over k = dimension + 1 nearest authored examples. This
    // mirrors the PMG edge lookup's Allen-style compact interpolation and avoids
    // global inverse-distance "ghosting" from far-away examples.
    const std::size_t neighbor_count =
        std::min<std::size_t>(static_cast<std::size_t>(parameter_dimension_) + 1, examples_.size());

    if (neighbor_count == 1) {
        weights[order.front()] = 1.0f;
        return weights;
    }

    // Shepard weights over the local stencil. Unlike PMG edge lookup, pose
    // synthesis must give the farthest member of a two-sample 1-D stencil a
    // nonzero weight; otherwise midpoints collapse to the nearest example.
    float weight_sum = 0.0f;
    for (std::size_t neighbor = 0; neighbor < neighbor_count; ++neighbor) {
        const std::size_t example_index = order[neighbor];
        const float distance = std::max(distances[example_index], kExactParameterThreshold);
        const float weight = 1.0f / distance;
        weights[example_index] = weight;
        weight_sum += weight;
    }

    if (weight_sum <= kSmallEpsilon) {
        weights.assign(examples_.size(), 0.0f);
        weights[order.front()] = 1.0f;
        return weights;
    }

    for (float& weight : weights) {
        weight /= weight_sum;
    }
    return weights;
}

std::vector<float> ParametricMotionSpace::ComputeInverseDistanceWeights(
    const ParameterVector& parameter) const {
    return ComputeLocalBlendWeights(parameter);
}

Pose ParametricMotionSpace::EvaluatePose(
    const ParameterVector& parameter,
    float normalized_phase) const {
    const ParameterVector clamped_parameter = ClampToDomain(parameter);
    const std::vector<float> weights = ComputeLocalBlendWeights(clamped_parameter);

    std::vector<Pose> poses;
    poses.reserve(examples_.size());
    for (std::size_t example_index = 0; example_index < examples_.size(); ++example_index) {
        const float example_phase =
            example_time_warps_.empty()
                ? normalized_phase
                : example_time_warps_[example_index].Evaluate(normalized_phase);
        poses.push_back(examples_[example_index].clip.SampleNormalizedPhase(example_phase));
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

    const ParameterVector clamped_parameter = ClampToDomain(parameter);
    for (int frame_index = 0; frame_index < frame_count; ++frame_index) {
        const float phase =
            frame_count == 1
                ? 0.0f
                : static_cast<float>(frame_index) / static_cast<float>(frame_count - 1);
        generated_clip.frames.push_back(EvaluatePose(clamped_parameter, phase));
    }

    return generated_clip;
}

const std::vector<ExampleMotion>& ParametricMotionSpace::Examples() const {
    return examples_;
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
    return Domain().Bounds().min_corner;
}

std::vector<float> ParametricMotionSpace::MaxParameter() const {
    return Domain().Bounds().max_corner;
}

ParameterDomain ParametricMotionSpace::Domain() const {
    return ParameterDomain::FromExamples(ExampleParameters());
}

ParameterVector ParametricMotionSpace::ClampToDomain(const ParameterVector& parameter) const {
    if (static_cast<int>(parameter.size()) != parameter_dimension_) {
        throw std::runtime_error("ParametricMotionSpace::ClampToDomain: parameter dimension mismatch");
    }
    return Domain().Clamp(parameter);
}

}  // namespace pmg
