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

GoalDirectedLocomotion::GoalDirectedLocomotion(
    const ParametricMotionGraph& graph,
    const Skeleton& skeleton,
    int node_index,
    int generated_frame_count,
    float frames_per_second,
    GoalDirectedLocomotionConfig config)
    : graph_(graph),
      skeleton_(skeleton),
      node_index_(node_index),
      generated_frame_count_(generated_frame_count),
      frames_per_second_(frames_per_second),
      config_(config) {
    const ParametricMotionSpace& space = graph_.Node(node_index_).motion_space;
    if (space.ParameterDimension() != 1) {
        throw std::runtime_error(
            "GoalDirectedLocomotion: node must have one parameter");
    }
    if (generated_frame_count_ <= 1 || frames_per_second_ <= 0.0f) {
        throw std::runtime_error(
            "GoalDirectedLocomotion: invalid runtime frame configuration");
    }
    if (config_.calibration.sample_count < 2 ||
        config_.calibration.sample_seconds <= 0.0f ||
        config_.orientation_blend_distance <= 0.0f) {
        throw std::runtime_error(
            "GoalDirectedLocomotion: invalid calibration/control settings");
    }

    const float parameter_min = space.MinParameter().front();
    const float parameter_max = space.MaxParameter().front();
    for (int sample = 0; sample < config_.calibration.sample_count; ++sample) {
        const float alpha = static_cast<float>(sample) /
                            static_cast<float>(
                                config_.calibration.sample_count - 1);
        const float parameter =
            parameter_min + alpha * (parameter_max - parameter_min);
        calibration_.parameters.push_back(parameter);
        calibration_.achieved_turn_rates.push_back(
            MeasureAchievedTurnRate(parameter));
    }
    calibration_.lowest_rate = *std::min_element(
        calibration_.achieved_turn_rates.begin(),
        calibration_.achieved_turn_rates.end());
    calibration_.highest_rate = *std::max_element(
        calibration_.achieved_turn_rates.begin(),
        calibration_.achieved_turn_rates.end());
    if (calibration_.highest_rate - calibration_.lowest_rate <
        config_.calibration.minimum_rate_range) {
        throw std::runtime_error(
            "GoalDirectedLocomotion: node has no achievable turn-rate variation");
    }
    calibration_.travel_heading_offset =
        EstimateTravelHeadingOffset(space.Examples().front().clip);
    calibration_.cycle_seconds =
        static_cast<float>(generated_frame_count_) / frames_per_second_;
}

const SteeringCalibration& GoalDirectedLocomotion::Calibration() const {
    return calibration_;
}

float GoalDirectedLocomotion::MeasureAchievedTurnRate(float parameter) const {
    PointCloudAlignment alignment(skeleton_);
    RuntimeController controller(graph_, alignment);
    controller.Start(
        node_index_, {parameter}, generated_frame_count_, frames_per_second_);

    RuntimeControlRequest request;
    request.desired_node = node_index_;
    request.desired_parameter = {parameter};

    const float delta_seconds = 1.0f / frames_per_second_;
    const int frame_count = std::max(
        1, static_cast<int>(
               config_.calibration.sample_seconds * frames_per_second_));
    float unwrapped_heading = 0.0f;
    float previous_heading = PoseFacingYaw(controller.CurrentPose());
    for (int frame = 0; frame < frame_count; ++frame) {
        controller.Update(delta_seconds, request);
        const float heading = PoseFacingYaw(controller.CurrentPose());
        unwrapped_heading +=
            WrapAngleRadians(heading - previous_heading);
        previous_heading = heading;
    }
    return unwrapped_heading /
           (static_cast<float>(frame_count) * delta_seconds);
}

float GoalDirectedLocomotion::ParameterForRate(float desired_rate) const {
    const float clamped_rate = std::clamp(
        desired_rate, calibration_.lowest_rate, calibration_.highest_rate);
    for (std::size_t segment = 0;
         segment + 1 < calibration_.achieved_turn_rates.size(); ++segment) {
        const float rate_a = calibration_.achieved_turn_rates[segment];
        const float rate_b = calibration_.achieved_turn_rates[segment + 1];
        if ((clamped_rate - rate_a) * (clamped_rate - rate_b) <= 0.0f &&
            std::abs(rate_b - rate_a) > kSmallEpsilon) {
            const float alpha =
                (clamped_rate - rate_a) / (rate_b - rate_a);
            return calibration_.parameters[segment] +
                   alpha * (calibration_.parameters[segment + 1] -
                            calibration_.parameters[segment]);
        }
    }

    std::size_t closest = 0;
    for (std::size_t sample = 1;
         sample < calibration_.achieved_turn_rates.size(); ++sample) {
        if (std::abs(calibration_.achieved_turn_rates[sample] - clamped_rate) <
            std::abs(calibration_.achieved_turn_rates[closest] - clamped_rate)) {
            closest = sample;
        }
    }
    return calibration_.parameters[closest];
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
    const float tightest_rate =
        std::abs(calibration_.lowest_rate) >
                std::abs(calibration_.highest_rate)
            ? calibration_.lowest_rate
            : calibration_.highest_rate;
    const float same_direction_limit = std::clamp(
        desired_rate, calibration_.lowest_rate, calibration_.highest_rate);
    if (!swinging_long_way_ &&
        std::abs(heading_error) > config_.swing_enter_error_radians &&
        (desired_rate > calibration_.highest_rate ||
         desired_rate < calibration_.lowest_rate) &&
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
    RuntimeControlRequest request;
    request.desired_node = node_index_;
    request.desired_parameter = space.ClampToDomain(
        {ParameterForRate(commanded_rate)});
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
