#include "pmg/AlignmentStrategy.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/ParametricMotionSpace.h"
#include "pmg/PmgBuilder.h"
#include "pmg/RuntimeController.h"
#include "pmg/Skeleton.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr int kRuntimeFrameCount = 24;
constexpr float kFramesPerSecond = 30.0f;
constexpr float kStrideUnits = 3.0f;      // forward (local +z) advance per clip
constexpr float kTurnDegrees = 20.0f;     // gentle curve per clip

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.parent_index = -1;
    skeleton.joints.push_back(root);
    pmg::Joint hip;
    hip.parent_index = 0;
    hip.offset = {0.6f, 0.0f, 0.0f};
    skeleton.joints.push_back(hip);
    pmg::Joint knee;
    knee.parent_index = 1;
    knee.offset = {0.0f, -1.5f, 0.0f};
    skeleton.joints.push_back(knee);
    return skeleton;
}

// A looping, gently curving walk: legs swing periodically (so the leg
// configuration matches at phase 0 and 1), the root marches forward in local
// +z, and the root yaw curves steadily.
pmg::MotionClip MakeWalkClip(float parameter) {
    pmg::MotionClip clip;
    clip.frames_per_second = kFramesPerSecond;
    const float swing_amplitude = 20.0f + 2.0f * parameter;
    for (int frame_index = 0; frame_index < kRuntimeFrameCount; ++frame_index) {
        const float phase = static_cast<float>(frame_index) /
                            static_cast<float>(kRuntimeFrameCount - 1);
        pmg::Pose pose;
        pose.root_position = {0.0f, 0.1f, phase * kStrideUnits};
        pose.local_rotations.push_back(pmg::EulerAxisRotation('Y', phase * kTurnDegrees));
        pose.local_rotations.push_back(
            pmg::EulerAxisRotation('X', swing_amplitude * std::sin(kTwoPi * phase)));
        pose.local_rotations.push_back(
            pmg::EulerAxisRotation('X', -swing_amplitude * std::sin(kTwoPi * phase)));
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::ParametricMotionGraph MakeSelfGraph(const pmg::Skeleton& skeleton, int& node_out) {
    pmg::ParametricMotionSpace space("walk", 1);
    space.AddExample({0.0f}, MakeWalkClip(0.0f));
    space.AddExample({1.0f}, MakeWalkClip(1.0f));

    pmg::ParametricMotionGraph graph;
    node_out = graph.AddNode("walk", space);

    pmg::PmgBuilderConfig config;
    config.source_sample_count = 4;
    config.target_sample_count = 8;
    config.generated_frame_count = kRuntimeFrameCount;
    config.good_transition_threshold = 1.0e9f;  // accept all (smooth self-space)
    config.bad_transition_threshold = 2.0e9f;
    pmg::PmgEdge edge =
        pmg::PmgBuilder::BuildEdge(skeleton, node_out, node_out, space, space, config);
    assert(!edge.samples.empty());
    graph.AddEdge(edge);
    return graph;
}

float WorldFacingYaw(const pmg::Pose& world_pose) {
    const pmg::Vec3 forward = pmg::Rotate(world_pose.local_rotations[0], {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

float WrappedDelta(float a, float b) {
    float delta = a - b;
    while (delta > kTwoPi * 0.5f) delta -= kTwoPi;
    while (delta < -kTwoPi * 0.5f) delta += kTwoPi;
    return std::abs(delta);
}

}  // namespace

int main() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    int node = -1;
    const pmg::ParametricMotionGraph graph = MakeSelfGraph(skeleton, node);

    const float source_phase_gate =
        graph.Edge(0).LookupInterpolated({0.5f})->source_transition_phase;

    // Paper-faithful path: recompute the exact point-cloud alignment per
    // transition (paper Sec 3.2 / Sec 5.2.1) instead of the stored/root-only one.
    pmg::PointCloudAlignment alignment(skeleton);
    pmg::RuntimeController controller(graph, alignment);
    controller.Start(node, {0.5f}, kRuntimeFrameCount, kFramesPerSecond);

    pmg::RuntimeControlRequest request;
    request.desired_node = node;
    request.desired_parameter = {0.5f};

    const float delta_seconds = 1.0f / kFramesPerSecond;
    std::vector<pmg::Vec3> world_positions;
    std::vector<float> facing_yaws;
    float first_transition_phase = -1.0f;

    for (int step = 0; step < 360; ++step) {
        const bool was_transitioning = controller.IsTransitioning();

        controller.Update(delta_seconds, request);

        // The frame a transition is scheduled, CurrentPhase() reports the source
        // clip phase that passed the gate (post-increment, pre-finalize).
        if (!was_transitioning && controller.IsTransitioning() && first_transition_phase < 0.0f) {
            first_transition_phase = controller.CurrentPhase();
        }

        const pmg::Pose pose = controller.CurrentPose();
        world_positions.push_back(pose.root_position);
        facing_yaws.push_back(WorldFacingYaw(pose));
    }

    // At least a couple of self-transitions occurred over the run.
    assert(controller.CompletedTransitions() >= 2);

    // The blend window is centered on the optimal transition point (F3): the
    // transition begins by the optimal phase (so its midpoint lands on it), and
    // no earlier than ~half a window before it (blend 0.20 -> ~0.10, plus a frame).
    assert(first_transition_phase <= source_phase_gate + 1.0e-4f);
    assert(first_transition_phase >= source_phase_gate - 0.15f);

    // Root trajectory is continuous: no per-frame jump far above the typical step.
    float total_step = 0.0f;
    float max_step = 0.0f;
    for (std::size_t i = 1; i < world_positions.size(); ++i) {
        const pmg::Vec3 delta = world_positions[i] - world_positions[i - 1];
        const float step = delta.Norm();
        total_step += step;
        max_step = std::max(max_step, step);
    }
    const float mean_step = total_step / static_cast<float>(world_positions.size() - 1);

    // TEMP debug: locate the worst step.
    {
        std::size_t worst = 0;
        float worst_step = 0.0f;
        for (std::size_t i = 1; i < world_positions.size(); ++i) {
            const float step = (world_positions[i] - world_positions[i - 1]).Norm();
            if (step > worst_step) { worst_step = step; worst = i; }
        }
        std::fprintf(stderr, "mean_step=%.4f max_step=%.4f worst_idx=%zu transitions=%d\n",
                     mean_step, max_step, worst, controller.CompletedTransitions());
        for (std::size_t i = (worst >= 3 ? worst - 3 : 0); i <= worst + 2 && i < world_positions.size(); ++i) {
            std::fprintf(stderr, "  [%zu] x=%.3f y=%.3f z=%.3f\n", i,
                         world_positions[i].x, world_positions[i].y, world_positions[i].z);
        }
    }

    assert(mean_step > 1.0e-4f);          // the character actually moved
    assert(max_step < 6.0f * mean_step);  // a pop would be ~one stride (>> mean)

    // Facing direction is continuous across transitions (no yaw pop).
    float max_facing_delta = 0.0f;
    for (std::size_t i = 1; i < facing_yaws.size(); ++i) {
        max_facing_delta = std::max(max_facing_delta, WrappedDelta(facing_yaws[i], facing_yaws[i - 1]));
    }
    assert(max_facing_delta < 0.2f);  // radians; a misaligned turn would jump far more

    return 0;
}
