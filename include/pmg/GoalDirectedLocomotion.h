#pragma once

#include "pmg/ParametricMotionGraph.h"
#include "pmg/RuntimeController.h"
#include "pmg/Skeleton.h"

#include <optional>
#include <random>
#include <vector>

namespace pmg {

float WrapAngleRadians(float angle_radians);
float PoseFacingYaw(const Pose& pose);
float EstimateTravelHeadingOffset(const MotionClip& clip);

struct SteeringCalibrationConfig {
    int sample_count = 5;
    float sample_seconds = 8.0f;
    float minimum_rate_range = 1.0e-4f;
};

struct SteeringCalibration {
    std::vector<float> parameters;
    std::vector<float> achieved_turn_rates;
    float lowest_rate = 0.0f;
    float highest_rate = 0.0f;
    float travel_heading_offset = 0.0f;
    float cycle_seconds = 1.0f;
};

struct GoalRequest {
    Vec3 target_position;
    std::optional<float> final_facing_yaw;
};

struct GoalDirectedLocomotionConfig {
    SteeringCalibrationConfig calibration;
    float swing_enter_error_radians = 0.5f;
    float swing_exit_error_radians = 0.2f;
    float orientation_blend_distance = 5.0f;
};

// Maps semantic locomotion goals to the one-dimensional parameter of a
// registered PMG node. Calibration measures achieved runtime turn rates rather
// than assuming authored clip curvature equals streamed graph behavior.
class GoalDirectedLocomotion {
public:
    GoalDirectedLocomotion(
        const ParametricMotionGraph& graph,
        const Skeleton& skeleton,
        int node_index,
        int generated_frame_count,
        float frames_per_second,
        GoalDirectedLocomotionConfig config = {});

    const SteeringCalibration& Calibration() const;
    RuntimeControlRequest RequestForPose(
        const Pose& world_pose, const GoalRequest& goal);
    bool Reached(
        const Pose& world_pose,
        const GoalRequest& goal,
        float position_tolerance,
        float facing_tolerance_radians) const;
    void Reset();

private:
    float MeasureAchievedTurnRate(float parameter) const;
    float ParameterForRate(float desired_rate) const;

    const ParametricMotionGraph& graph_;
    const Skeleton& skeleton_;
    int node_index_ = -1;
    int generated_frame_count_ = 0;
    float frames_per_second_ = 0.0f;
    GoalDirectedLocomotionConfig config_;
    SteeringCalibration calibration_;
    bool swinging_long_way_ = false;
};

struct RandomTransitionChoice {
    int edge_index = -1;
    RuntimeControlRequest request;
};

// Selects only from edges that actually leave current_node. Throws when the
// node has no outgoing edge instead of silently requesting an unreachable node.
RandomTransitionChoice ChooseRandomOutgoingTransition(
    const ParametricMotionGraph& graph,
    int current_node,
    std::mt19937& random);

}  // namespace pmg
