#include "pmg/ParametricMotionSpace.h"
#include "pmg/PoseBlend.h"
#include "pmg/RigidTransform2D.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace pmg {

namespace {

constexpr float kExactParameterThreshold = 1.0e-6f;
constexpr std::size_t kMaximumCalibrationSampleCount = 100000;

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

float NormalizedMetricDistance(
    const ParameterVector& left,
    const ParameterVector& right,
    const ParameterVector& scales) {
    RequireSameParameterDimension(left, right, "NormalizedMetricDistance");
    RequireSameParameterDimension(left, scales, "NormalizedMetricDistance");

    float squared_distance = 0.0f;
    for (std::size_t dimension = 0; dimension < left.size(); ++dimension) {
        const float normalized_delta =
            (left[dimension] - right[dimension]) / scales[dimension];
        squared_distance += normalized_delta * normalized_delta;
    }
    return std::sqrt(squared_distance);
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

void ParametricMotionSpace::SetParameterCalibration(ParameterCalibration calibration) {
    if (calibration.metrics.empty()) {
        throw std::runtime_error(
            "ParametricMotionSpace::SetParameterCalibration: metrics must not be empty");
    }
    if (static_cast<int>(calibration.metrics.size()) != parameter_dimension_) {
        throw std::runtime_error(
            "ParametricMotionSpace::SetParameterCalibration: metric count must match "
            "parameter dimension");
    }
    for (const ParameterMetric metric : calibration.metrics) {
        if (metric == ParameterMetric::kNone) {
            throw std::runtime_error(
                "ParametricMotionSpace::SetParameterCalibration: metrics must not "
                "contain kNone");
        }
    }
    for (std::size_t metric_index = 0;
         metric_index < calibration.metrics.size(); ++metric_index) {
        for (std::size_t previous_index = 0;
             previous_index < metric_index; ++previous_index) {
            if (calibration.metrics[previous_index] ==
                calibration.metrics[metric_index]) {
                throw std::runtime_error(
                    "ParametricMotionSpace::SetParameterCalibration: metrics "
                    "must be unique");
            }
        }
    }
    if (static_cast<int>(calibration.example_measured.size()) != NumExamples() ||
        calibration.metric_scales.size() != calibration.metrics.size() ||
        calibration.samples.empty() ||
        calibration.samples_per_axis < 0) {
        throw std::runtime_error(
            "ParametricMotionSpace::SetParameterCalibration: table does not match examples");
    }
    for (const ParameterVector& measured : calibration.example_measured) {
        if (measured.size() != calibration.metrics.size()) {
            throw std::runtime_error(
                "ParametricMotionSpace::SetParameterCalibration: invalid example "
                "measurement dimension");
        }
        for (const float value : measured) {
            if (!std::isfinite(value)) {
                throw std::runtime_error(
                    "ParametricMotionSpace::SetParameterCalibration: example "
                    "measurements must be finite");
            }
        }
    }
    for (const float scale : calibration.metric_scales) {
        if (!std::isfinite(scale) || scale <= 0.0f) {
            throw std::runtime_error(
                "ParametricMotionSpace::SetParameterCalibration: metric scales "
                "must be finite and positive");
        }
    }
    for (const CalibrationSample& sample : calibration.samples) {
        if (sample.measured_parameter.size() != calibration.metrics.size() ||
            sample.blend_weights.size() != examples_.size()) {
            throw std::runtime_error(
                "ParametricMotionSpace::SetParameterCalibration: invalid sample "
                "dimension");
        }
        for (const float value : sample.measured_parameter) {
            if (!std::isfinite(value)) {
                throw std::runtime_error(
                    "ParametricMotionSpace::SetParameterCalibration: sample "
                    "measurements must be finite");
            }
        }
        float weight_sum = 0.0f;
        for (const float weight : sample.blend_weights) {
            if (!std::isfinite(weight) || weight < 0.0f) {
                throw std::runtime_error(
                    "ParametricMotionSpace::SetParameterCalibration: sample weights "
                    "must be finite and nonnegative");
            }
            weight_sum += weight;
        }
        if (std::abs(weight_sum - 1.0f) > 1.0e-4f) {
            throw std::runtime_error(
                "ParametricMotionSpace::SetParameterCalibration: sample weights "
                "must sum to one");
        }
    }
    parameter_calibration_ = std::move(calibration);
}

void ParametricMotionSpace::ClearParameterCalibration() {
    parameter_calibration_ = ParameterCalibration{};
}

bool ParametricMotionSpace::HasParameterCalibration() const {
    return !parameter_calibration_.metrics.empty();
}

const ParameterCalibration& ParametricMotionSpace::ParameterCalibrationData() const {
    return parameter_calibration_;
}

std::vector<float> ParametricMotionSpace::CalibratedBlendWeights(
    const ParameterVector& parameter) const {
    const ParameterCalibration& calibration = parameter_calibration_;
    const std::vector<float> authored_weights =
        UncalibratedBlendWeights(parameter);

    ParameterVector target_measured(calibration.metrics.size(), 0.0f);
    for (std::size_t example_index = 0;
         example_index < examples_.size(); ++example_index) {
        for (std::size_t metric_index = 0;
             metric_index < calibration.metrics.size(); ++metric_index) {
            target_measured[metric_index] +=
                authored_weights[example_index] *
                calibration.example_measured[example_index][metric_index];
        }
    }

    std::vector<float> distances(calibration.samples.size(), 0.0f);
    std::vector<std::size_t> order(calibration.samples.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    for (std::size_t sample_index = 0;
         sample_index < calibration.samples.size(); ++sample_index) {
        distances[sample_index] = NormalizedMetricDistance(
            target_measured,
            calibration.samples[sample_index].measured_parameter,
            calibration.metric_scales);
    }
    std::sort(order.begin(), order.end(),
              [&](std::size_t left, std::size_t right) {
                  return distances[left] < distances[right];
              });

    std::vector<float> result(examples_.size(), 0.0f);
    std::size_t exact_count = 0;
    while (exact_count < order.size() &&
           distances[order[exact_count]] <= kExactParameterThreshold) {
        ++exact_count;
    }
    if (exact_count > 0) {
        const float exact_weight = 1.0f / static_cast<float>(exact_count);
        for (std::size_t exact_index = 0;
             exact_index < exact_count; ++exact_index) {
            const CalibrationSample& sample =
                calibration.samples[order[exact_index]];
            for (std::size_t example_index = 0;
                 example_index < examples_.size(); ++example_index) {
                result[example_index] +=
                    exact_weight * sample.blend_weights[example_index];
            }
        }
        return result;
    }

    const std::size_t neighbor_count = std::min<std::size_t>(
        calibration.metrics.size() + 1, calibration.samples.size());
    float inverse_distance_sum = 0.0f;
    for (std::size_t neighbor = 0; neighbor < neighbor_count; ++neighbor) {
        inverse_distance_sum += 1.0f / distances[order[neighbor]];
    }
    if (!std::isfinite(inverse_distance_sum) ||
        inverse_distance_sum <= kSmallEpsilon) {
        return calibration.samples[order.front()].blend_weights;
    }

    for (std::size_t neighbor = 0; neighbor < neighbor_count; ++neighbor) {
        const CalibrationSample& sample =
            calibration.samples[order[neighbor]];
        const float sample_weight =
            (1.0f / distances[order[neighbor]]) / inverse_distance_sum;
        for (std::size_t example_index = 0;
             example_index < examples_.size(); ++example_index) {
            result[example_index] +=
                sample_weight * sample.blend_weights[example_index];
        }
    }
    return result;
}

float MeasureParameterMetric(ParameterMetric metric, const MotionClip& clip) {
    clip.RequireNotEmpty("MeasureParameterMetric");
    const float duration = clip.DurationSeconds();
    if (duration <= kSmallEpsilon) {
        return 0.0f;
    }

    if (metric == ParameterMetric::kTurnRate) {
        float unwrapped_heading = 0.0f;
        for (int frame = 0; frame + 1 < clip.NumFrames(); ++frame) {
            unwrapped_heading += WrapPi(RootHeading(clip.frames[frame + 1]) -
                                        RootHeading(clip.frames[frame]));
        }
        return unwrapped_heading / duration;
    }
    if (metric == ParameterMetric::kTravelSpeed) {
        float path_length = 0.0f;
        for (int frame = 0; frame + 1 < clip.NumFrames(); ++frame) {
            const float delta_x =
                clip.frames[frame + 1].root_position.x -
                clip.frames[frame].root_position.x;
            const float delta_z =
                clip.frames[frame + 1].root_position.z -
                clip.frames[frame].root_position.z;
            path_length += std::sqrt(delta_x * delta_x + delta_z * delta_z);
        }
        return path_length / duration;
    }
    throw std::runtime_error("MeasureParameterMetric: unsupported metric");
}

ParameterCalibration CalibrateParameterMetrics(
    const ParametricMotionSpace& space,
    const std::vector<ParameterMetric>& metrics,
    float frames_per_second,
    int samples_per_axis) {
    if (static_cast<int>(metrics.size()) != space.ParameterDimension()) {
        throw std::runtime_error(
            "CalibrateParameterMetrics: metric count must match parameter dimension");
    }
    for (const ParameterMetric metric : metrics) {
        if (metric == ParameterMetric::kNone) {
            throw std::runtime_error(
                "CalibrateParameterMetrics: metrics must not contain kNone");
        }
    }
    for (std::size_t metric_index = 0;
         metric_index < metrics.size(); ++metric_index) {
        for (std::size_t previous_index = 0;
             previous_index < metric_index; ++previous_index) {
            if (metrics[previous_index] == metrics[metric_index]) {
                throw std::runtime_error(
                    "CalibrateParameterMetrics: metrics must be unique");
            }
        }
    }
    if (space.NumExamples() == 0) {
        throw std::runtime_error("CalibrateParameterMetrics: space has no examples");
    }
    if (frames_per_second <= 0.0f || samples_per_axis < 2) {
        throw std::runtime_error(
            "CalibrateParameterMetrics: invalid sampling settings");
    }

    std::size_t grid_sample_count = 1;
    for (int dimension = 0;
         dimension < space.ParameterDimension(); ++dimension) {
        if (grid_sample_count >
            kMaximumCalibrationSampleCount /
                static_cast<std::size_t>(samples_per_axis)) {
            throw std::runtime_error(
                "CalibrateParameterMetrics: calibration grid exceeds sample limit");
        }
        grid_sample_count *= static_cast<std::size_t>(samples_per_axis);
    }

    const auto generate_and_measure =
        [&](const std::vector<float>& weights) {
        float duration = 0.0f;
        for (std::size_t example_index = 0;
             example_index < space.Examples().size(); ++example_index) {
            duration +=
                weights[example_index] *
                space.Examples()[example_index].clip.DurationSeconds();
        }
        const int frame_count = std::max(
            2, static_cast<int>(std::lround(duration * frames_per_second)) + 1);
        const MotionClip clip = space.GenerateClipFromWeights(
            weights, frame_count, frames_per_second);
        ParameterVector measured;
        measured.reserve(metrics.size());
        for (const ParameterMetric metric : metrics) {
            measured.push_back(MeasureParameterMetric(metric, clip));
        }
        return measured;
    };

    ParameterCalibration calibration;
    calibration.metrics = metrics;
    calibration.samples_per_axis = samples_per_axis;
    calibration.example_measured.reserve(space.Examples().size());
    for (std::size_t example_index = 0;
         example_index < space.Examples().size(); ++example_index) {
        std::vector<float> weights(space.Examples().size(), 0.0f);
        weights[example_index] = 1.0f;
        calibration.example_measured.push_back(generate_and_measure(weights));
        calibration.samples.push_back(
            {calibration.example_measured.back(), std::move(weights)});
    }

    const auto compute_metric_scales = [&]() {
        calibration.metric_scales.assign(metrics.size(), 1.0f);
        for (std::size_t metric_index = 0;
             metric_index < metrics.size(); ++metric_index) {
            float minimum = std::numeric_limits<float>::infinity();
            float maximum = -std::numeric_limits<float>::infinity();
            for (const CalibrationSample& sample : calibration.samples) {
                minimum = std::min(
                    minimum, sample.measured_parameter[metric_index]);
                maximum = std::max(
                    maximum, sample.measured_parameter[metric_index]);
            }
            const float range = maximum - minimum;
            calibration.metric_scales[metric_index] =
                range > kSmallEpsilon ? range : 1.0f;
        }
    };

    if (space.ParameterDimension() == 1) {
        std::vector<int> example_order(space.NumExamples());
        std::iota(example_order.begin(), example_order.end(), 0);
        std::sort(
            example_order.begin(), example_order.end(),
            [&](int left, int right) {
                return space.Examples()[left].parameter.front() <
                       space.Examples()[right].parameter.front();
            });
        for (std::size_t pair = 0;
             pair + 1 < example_order.size(); ++pair) {
            const int left_example = example_order[pair];
            const int right_example = example_order[pair + 1];
            std::vector<CalibrationSample> segment_samples;
            segment_samples.reserve(
                static_cast<std::size_t>(samples_per_axis));
            for (int sample_index = 0;
                 sample_index < samples_per_axis; ++sample_index) {
                const float blend_t =
                    static_cast<float>(sample_index) /
                    static_cast<float>(samples_per_axis - 1);
                std::vector<float> weights(
                    space.Examples().size(), 0.0f);
                weights[left_example] = 1.0f - blend_t;
                weights[right_example] = blend_t;
                ParameterVector measured =
                    sample_index == 0
                        ? calibration.example_measured[left_example]
                        : sample_index + 1 == samples_per_axis
                              ? calibration.example_measured[right_example]
                              : generate_and_measure(weights);
                segment_samples.push_back(
                    {std::move(measured), std::move(weights)});
            }

            const bool increasing =
                segment_samples.back().measured_parameter.front() >=
                segment_samples.front().measured_parameter.front();
            for (std::size_t sample_index = 1;
                 sample_index < segment_samples.size(); ++sample_index) {
                float& measured =
                    segment_samples[sample_index].measured_parameter.front();
                const float previous =
                    segment_samples[sample_index - 1]
                        .measured_parameter.front();
                measured = increasing
                               ? std::max(measured, previous)
                               : std::min(measured, previous);
            }
            calibration.samples.insert(
                calibration.samples.end(),
                std::make_move_iterator(segment_samples.begin()),
                std::make_move_iterator(segment_samples.end()));
        }
        compute_metric_scales();
        return calibration;
    }

    const ParameterDomain domain = space.Domain();
    const ParameterAabb& bounds = domain.Bounds();
    for (std::size_t grid_index = 0;
         grid_index < grid_sample_count; ++grid_index) {
        std::size_t remaining_index = grid_index;
        ParameterVector parameter(
            static_cast<std::size_t>(space.ParameterDimension()), 0.0f);
        for (int dimension = 0;
             dimension < space.ParameterDimension(); ++dimension) {
            const int coordinate =
                static_cast<int>(
                    remaining_index %
                    static_cast<std::size_t>(samples_per_axis));
            remaining_index /= static_cast<std::size_t>(samples_per_axis);
            const float alpha =
                static_cast<float>(coordinate) /
                static_cast<float>(samples_per_axis - 1);
            parameter[static_cast<std::size_t>(dimension)] =
                bounds.min_corner[static_cast<std::size_t>(dimension)] +
                alpha *
                    (bounds.max_corner[static_cast<std::size_t>(dimension)] -
                     bounds.min_corner[static_cast<std::size_t>(dimension)]);
        }
        std::vector<float> weights =
            space.UncalibratedBlendWeights(parameter);
        calibration.samples.push_back(
            {generate_and_measure(weights), std::move(weights)});
    }

    compute_metric_scales();
    return calibration;
}

ParameterCalibration CalibrateParameterMetric(
    const ParametricMotionSpace& space,
    ParameterMetric metric,
    float frames_per_second,
    int samples_per_segment) {
    if (space.ParameterDimension() != 1) {
        throw std::runtime_error(
            "CalibrateParameterMetric: requires a 1-D space");
    }
    return CalibrateParameterMetrics(
        space, {metric}, frames_per_second, samples_per_segment);
}

std::vector<float> ParametricMotionSpace::ComputeLocalBlendWeights(
    const ParameterVector& parameter) const {
    if (static_cast<int>(parameter.size()) != parameter_dimension_) {
        throw std::runtime_error("ParametricMotionSpace::ComputeLocalBlendWeights: parameter dimension mismatch");
    }
    if (examples_.empty()) {
        throw std::runtime_error("ParametricMotionSpace::ComputeLocalBlendWeights: no examples available");
    }

    if (HasParameterCalibration()) {
        return CalibratedBlendWeights(parameter);
    }
    return UncalibratedBlendWeights(parameter);
}

std::vector<float> ParametricMotionSpace::UncalibratedBlendWeights(
    const ParameterVector& parameter) const {
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

Pose ParametricMotionSpace::EvaluatePose(
    const ParameterVector& parameter,
    float normalized_phase) const {
    const ParameterVector clamped_parameter = ClampToDomain(parameter);
    return EvaluatePoseFromWeights(
        ComputeLocalBlendWeights(clamped_parameter), normalized_phase);
}

Pose ParametricMotionSpace::EvaluatePoseFromWeights(
    const std::vector<float>& weights, float normalized_phase) const {
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

float ParametricMotionSpace::BlendedDurationSeconds(
    const ParameterVector& parameter) const {
    const ParameterVector clamped_parameter = ClampToDomain(parameter);
    const std::vector<float> weights = ComputeLocalBlendWeights(clamped_parameter);

    float duration = 0.0f;
    for (std::size_t example_index = 0; example_index < examples_.size(); ++example_index) {
        duration += weights[example_index] *
                    examples_[example_index].clip.DurationSeconds();
    }
    return duration;
}

MotionClip ParametricMotionSpace::GenerateClip(
    const ParameterVector& parameter,
    float frames_per_second) const {
    if (frames_per_second <= 0.0f) {
        throw std::runtime_error("ParametricMotionSpace::GenerateClip: frames_per_second must be positive");
    }
    const ParameterVector clamped_parameter = ClampToDomain(parameter);
    const float duration = BlendedDurationSeconds(clamped_parameter);
    const int frame_count = std::max(
        2, static_cast<int>(std::lround(duration * frames_per_second)) + 1);
    return GenerateClipFromWeights(
        ComputeLocalBlendWeights(clamped_parameter), frame_count, frames_per_second);
}

MotionClip ParametricMotionSpace::GenerateClipFromWeights(
    const std::vector<float>& weights,
    int frame_count,
    float frames_per_second) const {
    MotionClip generated_clip;
    generated_clip.name = name_ + "_generated";
    generated_clip.frames_per_second = frames_per_second;
    generated_clip.frames.reserve(static_cast<std::size_t>(frame_count));

    const auto sample_example = [this](std::size_t example_index, float canonical_phase) {
        const float example_phase =
            example_time_warps_.empty()
                ? canonical_phase
                : example_time_warps_[example_index].Evaluate(canonical_phase);
        return examples_[example_index].clip.SampleNormalizedPhase(example_phase);
    };

    Pose first_frame = EvaluatePoseFromWeights(weights, 0.0f);
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
        Pose frame = EvaluatePoseFromWeights(weights, phase);
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
