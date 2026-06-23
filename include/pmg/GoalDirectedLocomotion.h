#pragma once

#include "pmg/ParametricMotionGraph.h"
#include "pmg/ParametricMotionSpace.h"
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

// One node parameter axis, calibrated as a sampled inverse map from the
// achieved runtime metric back to the axis value. The turn_rate axis steers
// heading; travel_speed axes set forward pace.
struct SteeringAxis {
    ParameterMetric metric = ParameterMetric::kNone;
    std::vector<float> parameters;       // sampled axis values
    std::vector<float> achieved_metric;  // achieved runtime metric per sample
    float lowest = 0.0f;                 // min achieved metric over the sweep
    float highest = 0.0f;                // max achieved metric over the sweep
};

// Per-axis steering calibration for one node. Measures achieved runtime metrics
// rather than assuming authored clip behavior equals streamed graph behavior.
struct SteeringCalibration {
    std::vector<SteeringAxis> axes;  // one entry per node parameter axis
    int heading_axis = -1;           // axis whose metric is kTurnRate
    float travel_heading_offset = 0.0f;
    float cycle_seconds = 1.0f;

    // Heading (turn_rate) axis convenience accessors.
    const SteeringAxis& HeadingAxis() const;
    float LowestRate() const;   // HeadingAxis().lowest
    float HighestRate() const;  // HeadingAxis().highest
};

struct GoalRequest {
    Vec3 target_position;
    std::optional<float> final_facing_yaw;
};

struct GoalDirectedLocomotionConfig {
    SteeringCalibrationConfig calibration;
    RuntimeControllerConfig runtime;
    float swing_enter_error_radians = 0.5f;
    float swing_exit_error_radians = 0.2f;
    float orientation_blend_distance = 5.0f;
    // Travel-speed axes cruise at their fastest achievable pace until within
    // this floor distance of the goal, then ease toward their slowest so the
    // controller settles onto the target instead of overshooting it. Must exceed
    // the cruise turning radius (cruise speed / max turn rate, ~16 native units
    // for the walk_2d demo): below it the character keeps cruise speed all the
    // way in and orbits the goal at its turning radius instead of spiralling in.
    // Per-corpus calibration knob in native BVH distance units; also exposed as
    // `pmg_cli --goto --arrival-distance`.
    float arrival_speed_distance = 18.0f;
    // Optional per-axis metric override (size must equal the node dimension).
    // Empty = read from the node's parameter calibration; a 1-D node with no
    // calibration defaults to a single turn_rate axis.
    std::vector<ParameterMetric> axis_metrics;
};

// Maps semantic locomotion goals to the parameter vector of a registered PMG
// node. Drives every declared axis: the turn_rate axis tracks the goal heading
// (with a swing-through branch for targets behind), and travel_speed axes cruise
// at their fastest achievable pace and ease in on arrival. Reduces exactly to
// the prior single-axis steering for a one-dimensional turn_rate node.
class GoalDirectedLocomotion {
public:
    GoalDirectedLocomotion(
        const ParametricMotionGraph& graph,
        const Skeleton& skeleton,
        int node_index,
        float frames_per_second,
        GoalDirectedLocomotionConfig config = {});

    const SteeringCalibration& Calibration() const;
    RuntimeControlRequest RequestForPose(
        const Pose& world_pose, const GoalRequest& goal);
    RuntimeControlRequest RequestForDirectionSpeed(
        const Pose& world_pose,
        float desired_heading_radians,
        std::optional<float> desired_speed = std::nullopt) const;
    bool Reached(
        const Pose& world_pose,
        const GoalRequest& goal,
        float position_tolerance,
        float facing_tolerance_radians) const;
    void Reset();

private:
    // Streams the graph at a held parameter and measures the achieved runtime
    // metric (turn_rate or travel_speed) over the calibration window.
    float MeasureAchievedMetric(
        const ParameterVector& parameter, ParameterMetric metric) const;
    // Piecewise-linear inversion of one axis's sampled (achieved metric -> axis
    // value) map, clamped to the measured metric range.
    float AxisValueForMetric(int axis, float desired_metric) const;
    ParameterVector DomainMidpoint() const;

    const ParametricMotionGraph& graph_;
    const Skeleton& skeleton_;
    int node_index_ = -1;
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
