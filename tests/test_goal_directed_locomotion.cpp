#include "pmg/GoalDirectedLocomotion.h"

#include <cassert>
#include <cmath>
#include <random>
#include <stdexcept>

namespace {

constexpr int kFrameCount = 24;
constexpr float kFramesPerSecond = 30.0f;

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.name = "Hips";
    root.parent_index = -1;
    skeleton.joints.push_back(root);
    pmg::Joint child;
    child.name = "Marker";
    child.parent_index = 0;
    child.offset = {1.0f, 0.0f, 0.0f};
    skeleton.joints.push_back(child);
    return skeleton;
}

pmg::MotionClip MakeClip(float turn_degrees) {
    pmg::MotionClip clip;
    clip.frames_per_second = kFramesPerSecond;
    for (int frame = 0; frame < kFrameCount; ++frame) {
        const float phase =
            static_cast<float>(frame) / static_cast<float>(kFrameCount - 1);
        pmg::Pose pose;
        pose.root_position = {0.0f, 0.0f, 3.0f * phase};
        pose.local_rotations.push_back(
            pmg::EulerAxisRotation('Y', turn_degrees * phase));
        pose.local_rotations.push_back(pmg::Quaternion::Identity());
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::ParametricMotionSpace MakeSpace(const std::string& name) {
    pmg::ParametricMotionSpace space(name, 1);
    space.AddExample({0.0f}, MakeClip(-25.0f));
    space.AddExample({1.0f}, MakeClip(25.0f));
    return space;
}

// Axis 0 varies heading (turn_degrees), axis 1 varies forward pace
// (total_travel over the clip), so a 2-D node exposes one turn_rate axis and
// one travel_speed axis to steer independently.
pmg::MotionClip MakeClip2D(float turn_degrees, float total_travel) {
    pmg::MotionClip clip;
    clip.frames_per_second = kFramesPerSecond;
    for (int frame = 0; frame < kFrameCount; ++frame) {
        const float phase =
            static_cast<float>(frame) / static_cast<float>(kFrameCount - 1);
        pmg::Pose pose;
        pose.root_position = {0.0f, 0.0f, total_travel * phase};
        pose.local_rotations.push_back(
            pmg::EulerAxisRotation('Y', turn_degrees * phase));
        pose.local_rotations.push_back(pmg::Quaternion::Identity());
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::ParametricMotionSpace MakeSpace2D(const std::string& name) {
    pmg::ParametricMotionSpace space(name, 2);
    space.AddExample({0.0f, 0.0f}, MakeClip2D(-25.0f, 3.0f));
    space.AddExample({1.0f, 0.0f}, MakeClip2D(25.0f, 3.0f));
    space.AddExample({0.0f, 1.0f}, MakeClip2D(-25.0f, 9.0f));
    return space;
}

pmg::PmgEdge MakeEdge2D(int source, int target) {
    pmg::ParameterAabb box;
    box.min_corner = {0.0f, 0.0f};
    box.max_corner = {1.0f, 1.0f};
    pmg::PmgEdge edge;
    edge.source_node = source;
    edge.target_node = target;
    edge.samples.push_back({{0.0f, 0.0f}, box, 0.8f, 0.1f});
    edge.samples.push_back({{1.0f, 1.0f}, box, 0.8f, 0.1f});
    return edge;
}

pmg::PmgEdge MakeEdge(int source, int target) {
    pmg::ParameterAabb box;
    box.min_corner = {0.0f};
    box.max_corner = {1.0f};
    pmg::PmgEdge edge;
    edge.source_node = source;
    edge.target_node = target;
    edge.samples.push_back({{0.0f}, box, 0.8f, 0.1f});
    edge.samples.push_back({{1.0f}, box, 0.8f, 0.1f});
    return edge;
}

}  // namespace

int main() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    pmg::ParametricMotionGraph graph;
    const int walk = graph.AddNode("walk", MakeSpace("walk"));
    const int jog = graph.AddNode("jog", MakeSpace("jog"));
    const int dead_end = graph.AddNode("dead", MakeSpace("dead"));
    graph.AddEdge(MakeEdge(walk, walk));
    graph.AddEdge(MakeEdge(walk, jog));
    graph.AddEdge(MakeEdge(jog, walk));

    std::mt19937 random(7);
    for (int iteration = 0; iteration < 50; ++iteration) {
        const pmg::RandomTransitionChoice choice =
            pmg::ChooseRandomOutgoingTransition(graph, jog, random);
        assert(choice.request.desired_node == walk);
        assert(choice.edge_index == 2);
    }
    bool dead_end_threw = false;
    try {
        (void)pmg::ChooseRandomOutgoingTransition(graph, dead_end, random);
    } catch (const std::runtime_error&) {
        dead_end_threw = true;
    }
    assert(dead_end_threw);

    pmg::GoalDirectedLocomotionConfig config;
    config.calibration.sample_count = 3;
    config.calibration.sample_seconds = 3.0f;
    pmg::GoalDirectedLocomotion steering(
        graph, skeleton, walk, kFramesPerSecond, config);
    assert(steering.Calibration().axes.size() == 1);
    assert(steering.Calibration().heading_axis == 0);
    assert(steering.Calibration().HeadingAxis().parameters.size() == 3);
    assert(steering.Calibration().HighestRate() >
           steering.Calibration().LowestRate());

    pmg::Pose pose = MakeClip(0.0f).frames.front();
    pmg::GoalRequest goal;
    goal.target_position = {10.0f, 0.0f, 10.0f};
    const pmg::RuntimeControlRequest request =
        steering.RequestForPose(pose, goal);
    assert(request.desired_node == walk);
    assert(request.desired_parameter.size() == 1);
    assert(request.desired_parameter[0] >= 0.0f);
    assert(request.desired_parameter[0] <= 1.0f);
    assert(!steering.Reached(pose, goal, 1.0f, 0.1f));

    goal.target_position = pose.root_position;
    assert(steering.Reached(pose, goal, 0.01f, 0.1f));
    goal.final_facing_yaw = pmg::kPi;
    assert(!steering.Reached(pose, goal, 0.01f, 0.1f));
    goal.final_facing_yaw = 0.0f;
    assert(steering.Reached(pose, goal, 0.01f, 0.1f));

    // Multidimensional control: a 2-D (turn_rate, travel_speed) node must steer
    // both axes, not only axis 0. The heading axis tracks the goal; the speed
    // axis cruises far from the goal and eases in on arrival.
    pmg::ParametricMotionGraph graph2d;
    const int node2d = graph2d.AddNode("walk_2d", MakeSpace2D("walk_2d"));
    graph2d.AddEdge(MakeEdge2D(node2d, node2d));

    pmg::GoalDirectedLocomotionConfig config2d;
    config2d.calibration.sample_count = 3;
    config2d.calibration.sample_seconds = 2.0f;
    config2d.arrival_speed_distance = 5.0f;
    config2d.axis_metrics = {
        pmg::ParameterMetric::kTurnRate, pmg::ParameterMetric::kTravelSpeed};
    pmg::GoalDirectedLocomotion steering2d(
        graph2d, skeleton, node2d, kFramesPerSecond, config2d);
    assert(steering2d.Calibration().axes.size() == 2);
    assert(steering2d.Calibration().heading_axis == 0);
    assert(steering2d.Calibration().axes[1].metric ==
           pmg::ParameterMetric::kTravelSpeed);
    // The speed axis must register a non-degenerate achievable range.
    assert(steering2d.Calibration().axes[1].highest >
           steering2d.Calibration().axes[1].lowest);

    const pmg::Pose pose2d = MakeClip2D(0.0f, 3.0f).frames.front();
    pmg::GoalRequest far_goal;
    far_goal.target_position = {50.0f, 0.0f, 50.0f};
    const pmg::RuntimeControlRequest far_request =
        steering2d.RequestForPose(pose2d, far_goal);
    assert(far_request.desired_parameter.size() == 2);

    pmg::GoalRequest near_goal;
    near_goal.target_position = {pose2d.root_position.x + 1.0f, 0.0f,
                                 pose2d.root_position.z + 1.0f};
    const pmg::RuntimeControlRequest near_request =
        steering2d.RequestForPose(pose2d, near_goal);
    assert(near_request.desired_parameter.size() == 2);
    // Cruise (far) commands a faster speed-axis value than arrival (near).
    assert(far_request.desired_parameter[1] >
           near_request.desired_parameter[1] + 0.05f);
    return 0;
}
