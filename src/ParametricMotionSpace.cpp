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

void ParametricMotionSpace::SetParameterCalibration(ParameterCalibration calibration) {
    if (calibration.metric == ParameterMetric::kNone) {
        throw std::runtime_error(
            "ParametricMotionSpace::SetParameterCalibration: metric must not be kNone");
    }
    if (parameter_dimension_ != 1) {
        throw std::runtime_error(
            "ParametricMotionSpace::SetParameterCalibration: requires a 1-D space");
    }
    if (static_cast<int>(calibration.example_order.size()) != NumExamples() ||
        calibration.example_measured.size() != calibration.example_order.size() ||
        calibration.segments.size() + 1 != calibration.example_order.size()) {
        throw std::runtime_error(
            "ParametricMotionSpace::SetParameterCalibration: table does not match examples");
    }
    for (const CalibrationSegment& segment : calibration.segments) {
        if (segment.left_example < 0 || segment.left_example >= NumExamples() ||
            segment.right_example < 0 || segment.right_example >= NumExamples() ||
            segment.samples.size() < 2) {
            throw std::runtime_error(
                "ParametricMotionSpace::SetParameterCalibration: invalid segment");
        }
    }
    parameter_calibration_ = std::move(calibration);
}

void ParametricMotionSpace::ClearParameterCalibration() {
    parameter_calibration_ = ParameterCalibration{};
}

bool ParametricMotionSpace::HasParameterCalibration() const {
    return parameter_calibration_.metric != ParameterMetric::kNone;
}

const ParameterCalibration& ParametricMotionSpace::ParameterCalibrationData() const {
    return parameter_calibration_;
}

std::vector<float> ParametricMotionSpace::CalibratedBlendWeights(
    const ParameterVector& parameter) const {
    const ParameterCalibration& calibration = parameter_calibration_;
    std::vector<float> weights(examples_.size(), 0.0f);
    if (calibration.example_order.size() == 1) {
        weights[calibration.example_order.front()] = 1.0f;
        return weights;
    }

    // Locate the parameter-adjacent example pair bracketing the request.
    const float requested = parameter.front();
    std::size_t segment_index = 0;
    while (segment_index + 1 < calibration.segments.size() &&
           requested >
               examples_[calibration.example_order[segment_index + 1]].parameter.front()) {
        ++segment_index;
    }
    const CalibrationSegment& segment = calibration.segments[segment_index];
    const float parameter_left = examples_[segment.left_example].parameter.front();
    const float parameter_right = examples_[segment.right_example].parameter.front();

    // Target measured value: the requested parameter linearly interpolates the
    // examples' measured anchors, so authored parameters keep their meaning
    // while blends land on the measured curve.
    const float span = parameter_right - parameter_left;
    const float anchor_alpha =
        span > kSmallEpsilon
            ? std::clamp((requested - parameter_left) / span, 0.0f, 1.0f)
            : 0.0f;
    const float measured_left = calibration.example_measured[segment_index];
    const float measured_right = calibration.example_measured[segment_index + 1];
    const float target_measured =
        measured_left + anchor_alpha * (measured_right - measured_left);

    // Invert the sampled (t, measured) curve. Samples are monotone by
    // construction (CalibrateParameterMetric enforces it).
    float blend_t = anchor_alpha;
    for (std::size_t sample = 0; sample + 1 < segment.samples.size(); ++sample) {
        const float measured_a = segment.samples[sample].measured;
        const float measured_b = segment.samples[sample + 1].measured;
        if ((target_measured - measured_a) * (target_measured - measured_b) > 0.0f) {
            continue;
        }
        const float measured_span = measured_b - measured_a;
        const float local_alpha =
            std::abs(measured_span) > kSmallEpsilon
                ? (target_measured - measured_a) / measured_span
                : 0.0f;
        blend_t = segment.samples[sample].blend_t +
                  local_alpha * (segment.samples[sample + 1].blend_t -
                                 segment.samples[sample].blend_t);
        break;
    }
    blend_t = std::clamp(blend_t, 0.0f, 1.0f);

    weights[segment.left_example] = 1.0f - blend_t;
    weights[segment.right_example] = blend_t;
    return weights;
}

float MeasureParameterMetric(ParameterMetric metric, const MotionClip& clip) {
    if (metric != ParameterMetric::kTurnRate) {
        throw std::runtime_error("MeasureParameterMetric: unsupported metric");
    }
    clip.RequireNotEmpty("MeasureParameterMetric");
    const float duration = clip.DurationSeconds();
    if (duration <= kSmallEpsilon) {
        return 0.0f;
    }
    float unwrapped_heading = 0.0f;
    for (int frame = 0; frame + 1 < clip.NumFrames(); ++frame) {
        unwrapped_heading += WrapPi(RootHeading(clip.frames[frame + 1]) -
                                    RootHeading(clip.frames[frame]));
    }
    return unwrapped_heading / duration;
}

ParameterCalibration CalibrateParameterMetric(
    const ParametricMotionSpace& space,
    ParameterMetric metric,
    float frames_per_second,
    int samples_per_segment) {
    if (metric == ParameterMetric::kNone) {
        throw std::runtime_error("CalibrateParameterMetric: metric must not be kNone");
    }
    if (space.ParameterDimension() != 1) {
        throw std::runtime_error("CalibrateParameterMetric: requires a 1-D space");
    }
    if (space.NumExamples() == 0) {
        throw std::runtime_error("CalibrateParameterMetric: space has no examples");
    }
    if (frames_per_second <= 0.0f || samples_per_segment < 2) {
        throw std::runtime_error("CalibrateParameterMetric: invalid sampling settings");
    }

    ParameterCalibration calibration;
    calibration.metric = metric;
    calibration.example_order.resize(space.NumExamples());
    std::iota(calibration.example_order.begin(), calibration.example_order.end(), 0);
    std::sort(calibration.example_order.begin(), calibration.example_order.end(),
              [&](int left, int right) {
                  return space.Examples()[left].parameter.front() <
                         space.Examples()[right].parameter.front();
              });

    const auto generate_and_measure = [&](int left_example, int right_example,
                                          float blend_t) {
        std::vector<float> weights(space.Examples().size(), 0.0f);
        weights[left_example] += 1.0f - blend_t;
        weights[right_example] += blend_t;
        const float duration =
            (1.0f - blend_t) *
                space.Examples()[left_example].clip.DurationSeconds() +
            blend_t * space.Examples()[right_example].clip.DurationSeconds();
        const int frame_count = std::max(
            2, static_cast<int>(std::lround(duration * frames_per_second)) + 1);
        return MeasureParameterMetric(
            metric,
            space.GenerateClipFromWeights(weights, frame_count, frames_per_second));
    };

    for (const int example_index : calibration.example_order) {
        calibration.example_measured.push_back(
            generate_and_measure(example_index, example_index, 0.0f));
    }

    for (std::size_t pair = 0; pair + 1 < calibration.example_order.size(); ++pair) {
        CalibrationSegment segment;
        segment.left_example = calibration.example_order[pair];
        segment.right_example = calibration.example_order[pair + 1];
        segment.samples.push_back({0.0f, calibration.example_measured[pair]});
        for (int sample = 1; sample + 1 < samples_per_segment; ++sample) {
            const float blend_t = static_cast<float>(sample) /
                                  static_cast<float>(samples_per_segment - 1);
            segment.samples.push_back(
                {blend_t,
                 generate_and_measure(segment.left_example, segment.right_example,
                                      blend_t)});
        }
        segment.samples.push_back({1.0f, calibration.example_measured[pair + 1]});

        // Force the curve monotone toward its endpoint so inversion is well
        // defined; small non-monotone wiggles come from contact-phase noise in
        // the generated blends, not from a real direction change.
        const bool increasing =
            segment.samples.back().measured >= segment.samples.front().measured;
        for (std::size_t sample = 1; sample < segment.samples.size(); ++sample) {
            float& measured = segment.samples[sample].measured;
            const float previous = segment.samples[sample - 1].measured;
            if (increasing) {
                measured = std::max(measured, previous);
            } else {
                measured = std::min(measured, previous);
            }
        }
        calibration.segments.push_back(std::move(segment));
    }

    return calibration;
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
