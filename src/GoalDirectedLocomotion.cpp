#include "pmg/GoalDirectedLocomotion.h"

#include "pmg/AlignmentStrategy.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pmg {

float WrapAngleRadians(float angle_radians) {
    return angle_radians -
           2.0f * kPi * std::round(angle_radians / (2.0f * kPi));
}

float PoseFacingYaw(const Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const Vec3 forward =
        Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

float EstimateTravelHeadingOffset(const MotionClip& clip) {
    clip.RequireNotEmpty("EstimateTravelHeadingOffset");
    float weighted_sine = 0.0f;
    float weighted_cosine = 0.0f;
    for (int frame = 0; frame + 1 < clip.NumFrames(); ++frame) {
        const Vec3 step =
            clip.frames[frame + 1].root_position -
            clip.frames[frame].root_position;
        const float step_length =
            std::sqrt(step.x * step.x + step.z * step.z);
        if (step_length <= kSmallEpsilon) {
            continue;
        }
        const float travel_heading = std::atan2(step.x, step.z);
        const float offset = WrapAngleRadians(
            travel_heading - PoseFacingYaw(clip.frames[frame]));
        weighted_sine += step_length * std::sin(offset);
        weighted_cosine += step_length * std::cos(offset);
    }
    if (std::abs(weighted_sine) <= kSmallEpsilon &&
        std::abs(weighted_cosine) <= kSmallEpsilon) {
        throw std::runtime_error(
            "EstimateTravelHeadingOffset: clip has no floor displacement");
    }
    return std::atan2(weighted_sine, weighted_cosine);
}

const SteeringAxis& SteeringCalibration::HeadingAxis() const {
    if (heading_axis < 0 ||
        heading_axis >= static_cast<int>(axes.size())) {
        throw std::runtime_error(
            "SteeringCalibration: no heading (turn_rate) axis");
    }
    return axes[static_cast<std::size_t>(heading_axis)];
}

float SteeringCalibration::LowestRate() const {
    return HeadingAxis().lowest;
}

float SteeringCalibration::HighestRate() const {
    return HeadingAxis().highest;
}

namespace {

// Resolves one ParameterMetric per axis: explicit config override, else the
// node's parameter calibration, else a one-dimensional turn_rate default.
std::vector<ParameterMetric> ResolveAxisMetrics(
    const ParametricMotionSpace& space,
    const std::vector<ParameterMetric>& override_metrics) {
    const int dimension = space.ParameterDimension();
    if (!override_metrics.empty()) {
        if (static_cast<int>(override_metrics.size()) != dimension) {
            throw std::runtime_error(
                "GoalDirectedLocomotion: axis_metrics size must match the node "
                "parameter dimension");
        }
        return override_metrics;
    }
    if (space.HasParameterCalibration() &&
        static_cast<int>(space.ParameterCalibrationData().metrics.size()) ==
            dimension) {
        return space.ParameterCalibrationData().metrics;
    }
    if (dimension == 1) {
        return {ParameterMetric::kTurnRate};
    }
    throw std::runtime_error(
        "GoalDirectedLocomotion: multidimensional node needs declared "
        "parameter metrics to steer");
}

}  // namespace

GoalDirectedLocomotion::GoalDirectedLocomotion(
    const ParametricMotionGraph& graph,
    const Skeleton& skeleton,
    int node_index,
    float frames_per_second,
    GoalDirectedLocomotionConfig config)
    : graph_(graph),
      skeleton_(skeleton),
      node_index_(node_index),
      frames_per_second_(frames_per_second),
      config_(config) {
    const ParametricMotionSpace& space = graph_.Node(node_index_).motion_space;
    const int dimension = space.ParameterDimension();
    if (dimension < 1) {
        throw std::runtime_error(
            "GoalDirectedLocomotion: node has no parameter axis");
    }
    if (frames_per_second_ <= 0.0f) {
        throw std::runtime_error(
            "GoalDirectedLocomotion: invalid runtime frame configuration");
    }
    if (config_.calibration.sample_count < 2 ||
        config_.calibration.sample_seconds <= 0.0f ||
        config_.orientation_blend_distance <= 0.0f ||
        config_.arrival_speed_distance <= 0.0f) {
        throw std::runtime_error(
            "GoalDirectedLocomotion: invalid calibration/control settings");
    }

    const std::vector<ParameterMetric> metrics =
        ResolveAxisMetrics(space, config_.axis_metrics);
    calibration_.heading_axis = -1;
    for (int axis = 0; axis < dimension; ++axis) {
        if (metrics[static_cast<std::size_t>(axis)] ==
            ParameterMetric::kTurnRate) {
            calibration_.heading_axis = axis;
            break;
        }
    }
    if (calibration_.heading_axis < 0) {
        throw std::runtime_error(
            "GoalDirectedLocomotion: node needs a turn_rate axis to steer "
            "heading");
    }

    const std::vector<float> parameter_min = space.MinParameter();
    const std::vector<float> parameter_max = space.MaxParameter();
    const ParameterVector midpoint = DomainMidpoint();
    calibration_.axes.resize(static_cast<std::size_t>(dimension));
    for (int axis = 0; axis < dimension; ++axis) {
        SteeringAxis& axis_calibration =
            calibration_.axes[static_cast<std::size_t>(axis)];
        axis_calibration.metric = metrics[static_cast<std::size_t>(axis)];
        for (int sample = 0; sample < config_.calibration.sample_count;
             ++sample) {
            const float alpha =
                static_cast<float>(sample) /
                static_cast<float>(config_.calibration.sample_count - 1);
            const float value =
                parameter_min[static_cast<std::size_t>(axis)] +
                alpha * (parameter_max[static_cast<std::size_t>(axis)] -
                         parameter_min[static_cast<std::size_t>(axis)]);
            ParameterVector held = midpoint;
            held[static_cast<std::size_t>(axis)] = value;
            axis_calibration.parameters.push_back(value);
            axis_calibration.achieved_metric.push_back(
                axis_calibration.metric == ParameterMetric::kNone
                    ? 0.0f
                    : MeasureAchievedMetric(held, axis_calibration.metric));
        }
        axis_calibration.lowest = *std::min_element(
            axis_calibration.achieved_metric.begin(),
            axis_calibration.achieved_metric.end());
        axis_calibration.highest = *std::max_element(
            axis_calibration.achieved_metric.begin(),
            axis_calibration.achieved_metric.end());
    }

    const SteeringAxis& heading = calibration_.HeadingAxis();
    if (heading.highest - heading.lowest <
        config_.calibration.minimum_rate_range) {
        throw std::runtime_error(
            "GoalDirectedLocomotion: node has no achievable turn-rate variation");
    }
    calibration_.travel_heading_offset =
        EstimateTravelHeadingOffset(space.Examples().front().clip);
    // Cycle time follows the blended example durations (D2); use the domain
    // midpoint as the representative steering cadence.
    calibration_.cycle_seconds = std::max(
        1.0f / frames_per_second_, space.BlendedDurationSeconds(midpoint));
}

const SteeringCalibration& GoalDirectedLocomotion::Calibration() const {
    return calibration_;
}

ParameterVector GoalDirectedLocomotion::DomainMidpoint() const {
    const ParametricMotionSpace& space = graph_.Node(node_index_).motion_space;
    const std::vector<float> parameter_min = space.MinParameter();
    const std::vector<float> parameter_max = space.MaxParameter();
    ParameterVector midpoint(parameter_min.size(), 0.0f);
    for (std::size_t axis = 0; axis < parameter_min.size(); ++axis) {
        midpoint[axis] = 0.5f * (parameter_min[axis] + parameter_max[axis]);
    }
    return midpoint;
}

float GoalDirectedLocomotion::MeasureAchievedMetric(
    const ParameterVector& parameter, ParameterMetric metric) const {
    PointCloudAlignment alignment(
        skeleton_, config_.runtime.transition_blend_frames);
    RuntimeController controller(graph_, alignment, config_.runtime);
    controller.Start(node_index_, parameter, frames_per_second_);

    RuntimeControlRequest request;
    request.desired_node = node_index_;
    request.desired_parameter = parameter;

    const float delta_seconds = 1.0f / frames_per_second_;
    const int frame_count = std::max(
        1, static_cast<int>(
               config_.calibration.sample_seconds * frames_per_second_));

    if (metric == ParameterMetric::kTurnRate) {
        float unwrapped_heading = 0.0f;
        float previous_heading = PoseFacingYaw(controller.CurrentPose());
        for (int frame = 0; frame < frame_count; ++frame) {
            controller.Update(delta_seconds, request);
            const float heading = PoseFacingYaw(controller.CurrentPose());
            unwrapped_heading += WrapAngleRadians(heading - previous_heading);
            previous_heading = heading;
        }
        return unwrapped_heading /
               (static_cast<float>(frame_count) * delta_seconds);
    }
    if (metric == ParameterMetric::kTravelSpeed) {
        float floor_distance = 0.0f;
        Pose previous_pose = controller.CurrentPose();
        for (int frame = 0; frame < frame_count; ++frame) {
            controller.Update(delta_seconds, request);
            const Pose pose = controller.CurrentPose();
            const float step_x =
                pose.root_position.x - previous_pose.root_position.x;
            const float step_z =
                pose.root_position.z - previous_pose.root_position.z;
            floor_distance += std::sqrt(step_x * step_x + step_z * step_z);
            previous_pose = pose;
        }
        return floor_distance /
               (static_cast<float>(frame_count) * delta_seconds);
    }
    throw std::runtime_error(
        "GoalDirectedLocomotion::MeasureAchievedMetric: unsupported metric");
}

float GoalDirectedLocomotion::AxisValueForMetric(
    int axis, float desired_metric) const {
    const SteeringAxis& axis_calibration =
        calibration_.axes[static_cast<std::size_t>(axis)];
    const float clamped_metric = std::clamp(
        desired_metric, axis_calibration.lowest, axis_calibration.highest);
    for (std::size_t segment = 0;
         segment + 1 < axis_calibration.achieved_metric.size(); ++segment) {
        const float metric_a = axis_calibration.achieved_metric[segment];
        const float metric_b = axis_calibration.achieved_metric[segment + 1];
        if ((clamped_metric - metric_a) * (clamped_metric - metric_b) <= 0.0f &&
            std::abs(metric_b - metric_a) > kSmallEpsilon) {
            const float alpha =
                (clamped_metric - metric_a) / (metric_b - metric_a);
            return axis_calibration.parameters[segment] +
                   alpha * (axis_calibration.parameters[segment + 1] -
                            axis_calibration.parameters[segment]);
        }
    }

    std::size_t closest = 0;
    for (std::size_t sample = 1;
         sample < axis_calibration.achieved_metric.size(); ++sample) {
        if (std::abs(axis_calibration.achieved_metric[sample] -
                     clamped_metric) <
            std::abs(axis_calibration.achieved_metric[closest] -
                     clamped_metric)) {
            closest = sample;
        }
    }
    return axis_calibration.parameters[closest];
}

RuntimeControlRequest GoalDirectedLocomotion::RequestForPose(
    const Pose& world_pose, const GoalRequest& goal) {
    const float delta_x =
        goal.target_position.x - world_pose.root_position.x;
    const float delta_z =
        goal.target_position.z - world_pose.root_position.z;
    const float distance =
        std::sqrt(delta_x * delta_x + delta_z * delta_z);
    float desired_heading = std::atan2(delta_x, delta_z);
    if (goal.final_facing_yaw.has_value()) {
        const float orientation_weight = std::clamp(
            1.0f - distance / config_.orientation_blend_distance,
            0.0f, 1.0f);
        desired_heading += orientation_weight * WrapAngleRadians(
            *goal.final_facing_yaw - desired_heading);
    }

    const float travel_heading = WrapAngleRadians(
        PoseFacingYaw(world_pose) + calibration_.travel_heading_offset);
    const float heading_error =
        WrapAngleRadians(desired_heading - travel_heading);
    const float desired_rate =
        heading_error / calibration_.cycle_seconds;

    const SteeringAxis& heading = calibration_.HeadingAxis();
    const float tightest_rate =
        std::abs(heading.lowest) > std::abs(heading.highest)
            ? heading.lowest
            : heading.highest;
    const float same_direction_limit =
        std::clamp(desired_rate, heading.lowest, heading.highest);
    if (!swinging_long_way_ &&
        std::abs(heading_error) > config_.swing_enter_error_radians &&
        (desired_rate > heading.highest || desired_rate < heading.lowest) &&
        std::abs(same_direction_limit) < 0.5f * std::abs(tightest_rate)) {
        swinging_long_way_ = true;
    }
    if (swinging_long_way_ &&
        std::abs(heading_error) < config_.swing_exit_error_radians) {
        swinging_long_way_ = false;
    }
    const float commanded_rate =
        swinging_long_way_ ? tightest_rate : same_direction_limit;

    const ParametricMotionSpace& space =
        graph_.Node(node_index_).motion_space;
    ParameterVector parameter = DomainMidpoint();
    for (int axis = 0; axis < static_cast<int>(parameter.size()); ++axis) {
        const SteeringAxis& axis_calibration =
            calibration_.axes[static_cast<std::size_t>(axis)];
        if (axis == calibration_.heading_axis) {
            parameter[static_cast<std::size_t>(axis)] =
                AxisValueForMetric(axis, commanded_rate);
        } else if (axis_calibration.metric == ParameterMetric::kTravelSpeed) {
            // Cruise far from the goal, ease toward the slowest pace on arrival.
            const float arrival = std::clamp(
                distance / config_.arrival_speed_distance, 0.0f, 1.0f);
            const float desired_speed =
                axis_calibration.lowest +
                arrival * (axis_calibration.highest - axis_calibration.lowest);
            parameter[static_cast<std::size_t>(axis)] =
                AxisValueForMetric(axis, desired_speed);
        }
        // kNone axes keep the domain-midpoint value.
    }

    RuntimeControlRequest request;
    request.desired_node = node_index_;
    request.desired_parameter = space.ClampToDomain(parameter);
    return request;
}

bool GoalDirectedLocomotion::Reached(
    const Pose& world_pose,
    const GoalRequest& goal,
    float position_tolerance,
    float facing_tolerance_radians) const {
    if (position_tolerance < 0.0f || facing_tolerance_radians < 0.0f) {
        throw std::runtime_error(
            "GoalDirectedLocomotion::Reached: tolerances must be non-negative");
    }
    const float delta_x =
        goal.target_position.x - world_pose.root_position.x;
    const float delta_z =
        goal.target_position.z - world_pose.root_position.z;
    if (std::sqrt(delta_x * delta_x + delta_z * delta_z) >
        position_tolerance) {
        return false;
    }
    if (!goal.final_facing_yaw.has_value()) {
        return true;
    }
    return std::abs(WrapAngleRadians(
               *goal.final_facing_yaw - PoseFacingYaw(world_pose))) <=
           facing_tolerance_radians;
}

void GoalDirectedLocomotion::Reset() {
    swinging_long_way_ = false;
}

RandomTransitionChoice ChooseRandomOutgoingTransition(
    const ParametricMotionGraph& graph,
    int current_node,
    std::mt19937& random) {
    const std::vector<int> outgoing =
        graph.OutgoingEdgeIndices(current_node);
    if (outgoing.empty()) {
        throw std::runtime_error(
            "ChooseRandomOutgoingTransition: current node has no outgoing edge");
    }
    std::uniform_int_distribution<std::size_t> edge_picker(
        0, outgoing.size() - 1);
    RandomTransitionChoice choice;
    choice.edge_index = outgoing[edge_picker(random)];
    const PmgEdge& edge = graph.Edge(choice.edge_index);
    choice.request.desired_node = edge.target_node;
    choice.request.desired_parameter =
        graph.Node(edge.target_node).motion_space.Domain().SampleUniform(random);
    return choice;
}

}  // namespace pmg
