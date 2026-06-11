#include "pmg/ParametricMotionSpace.h"
#include "pmg/PoseBlend.h"
#include "pmg/RigidTransform2D.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace pmg {

namespace {

constexpr float kExactParameterThreshold = 1.0e-6f;

float WrapPi(float angle_radians) {
    return angle_radians - 2.0f * kPi * std::round(angle_radians / (2.0f * kPi));
}

// Heading of a root rotation: yaw of the rotated +Z axis, in the same
// convention as RigidTransform2D::RotateFloor (rotating (0,0,1) by yaw gives
// (sin(yaw), cos(yaw))). The reference axis only fixes a per-space constant
// offset; root-delta integration uses heading differences, so any consistent
// axis choice cancels out.
float RootHeading(const Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const Vec3 forward = Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

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

const std::vector<TimeWarp>& ParametricMotionSpace::ExampleTimeWarps() const {
    return example_time_warps_;
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
    const std::vector<float> weights = ComputeLocalBlendWeights(clamped_parameter);

    const auto sample_example = [this](std::size_t example_index, float canonical_phase) {
        const float example_phase =
            example_time_warps_.empty()
                ? canonical_phase
                : example_time_warps_[example_index].Evaluate(canonical_phase);
        return examples_[example_index].clip.SampleNormalizedPhase(example_phase);
    };

    Pose first_frame = EvaluatePose(clamped_parameter, 0.0f);
    float world_x = first_frame.root_position.x;
    float world_z = first_frame.root_position.z;
    float world_heading = RootHeading(first_frame);
    generated_clip.frames.push_back(std::move(first_frame));

    for (int frame_index = 1; frame_index < frame_count; ++frame_index) {
        const float previous_phase =
            static_cast<float>(frame_index - 1) / static_cast<float>(frame_count - 1);
        const float phase =
            static_cast<float>(frame_index) / static_cast<float>(frame_count - 1);

        // Blend each example's root step expressed in its own heading frame.
        float delta_x = 0.0f;
        float delta_z = 0.0f;
        float delta_heading = 0.0f;
        for (std::size_t example_index = 0; example_index < examples_.size(); ++example_index) {
            if (weights[example_index] <= 0.0f) {
                continue;
            }
            const Pose previous_pose = sample_example(example_index, previous_phase);
            const Pose current_pose = sample_example(example_index, phase);

            const float previous_heading = RootHeading(previous_pose);
            const RigidTransform2D into_heading_frame{-previous_heading, 0.0f, 0.0f};
            float local_dx = 0.0f;
            float local_dz = 0.0f;
            into_heading_frame.RotateFloor(
                current_pose.root_position.x - previous_pose.root_position.x,
                current_pose.root_position.z - previous_pose.root_position.z,
                local_dx, local_dz);

            delta_x += weights[example_index] * local_dx;
            delta_z += weights[example_index] * local_dz;
            delta_heading += weights[example_index] *
                             WrapPi(RootHeading(current_pose) - previous_heading);
        }

        const RigidTransform2D out_of_heading_frame{world_heading, 0.0f, 0.0f};
        float step_x = 0.0f;
        float step_z = 0.0f;
        out_of_heading_frame.RotateFloor(delta_x, delta_z, step_x, step_z);
        world_x += step_x;
        world_z += step_z;
        world_heading += delta_heading;

        // Joints and height come from the registered pose blend; only the
        // root's floor placement and heading are replaced by the integral.
        Pose frame = EvaluatePose(clamped_parameter, phase);
        frame.root_position.x = world_x;
        frame.root_position.z = world_z;
        if (!frame.local_rotations.empty()) {
            const float heading_fix = WrapPi(world_heading - RootHeading(frame));
            frame.local_rotations.front() =
                Quaternion::FromAxisAngle({0.0f, 1.0f, 0.0f}, heading_fix) *
                frame.local_rotations.front();
        }
        generated_clip.frames.push_back(std::move(frame));
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
