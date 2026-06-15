#include "pmg/AlignmentStrategy.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/ParametricMotionSpace.h"
#include "pmg/PmgBuilder.h"
#include "pmg/RuntimeController.h"
#include "pmg/Skeleton.h"

#include <cassert>
#include <cmath>
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

pmg::ParametricMotionGraph MakeBoundaryCrossingGraph(int& node_out) {
    pmg::ParametricMotionSpace space("boundary_walk", 1);
    space.AddExample({0.0f}, MakeWalkClip(0.0f));
    space.AddExample({1.0f}, MakeWalkClip(1.0f));

    pmg::ParametricMotionGraph graph;
    node_out = graph.AddNode("boundary_walk", space);

    pmg::PmgEdge edge;
    edge.source_node = node_out;
    edge.target_node = node_out;
    edge.samples.push_back({
        {0.5f},
        {{0.0f}, {1.0f}},
        0.95f,
        0.50f,
    });
    graph.AddEdge(std::move(edge));
    return graph;
}

// A self-edge whose target transition phase is at the clip start, so the
// centered pre-roll lands before phase 0 (negative). Whether that pre-roll wraps
// or clamps is decided purely by node cyclicity.
pmg::ParametricMotionGraph MakePreRollWrapGraph(int& node_out) {
    pmg::ParametricMotionSpace space("preroll_walk", 1);
    space.AddExample({0.0f}, MakeWalkClip(0.0f));
    space.AddExample({1.0f}, MakeWalkClip(1.0f));

    pmg::ParametricMotionGraph graph;
    node_out = graph.AddNode("preroll_walk", space);

    pmg::PmgEdge edge;
    edge.source_node = node_out;
    edge.target_node = node_out;
    edge.samples.push_back({{0.5f}, {{0.0f}, {1.0f}}, 0.5f, 0.0f});
    graph.AddEdge(std::move(edge));
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
        graph.Edge(0).LookupInterpolated(
            {0.5f}, {0.5f})->source_transition_phase;

    // Paper-core path: recompute the Kovar-derived joint point-cloud alignment
    // per transition instead of using the root-only debug adapter.
    pmg::PointCloudAlignment alignment(skeleton);
    // The walk node is a cyclic locomotion space, so its self-edge wraps the
    // centered pre-roll into the previous cycle tail (kWrapCyclicSelfEdges).
    pmg::RuntimeControllerConfig walk_config;
    walk_config.cyclic_nodes.assign(graph.NumNodes(), true);
    pmg::RuntimeController controller(graph, alignment, walk_config);
    controller.Start(node, {0.5f}, kFramesPerSecond);

    pmg::RuntimeControlRequest request;
    request.desired_node = node;
    request.desired_parameter = {0.5f};

    const float delta_seconds = 1.0f / kFramesPerSecond;
    std::vector<pmg::Vec3> world_positions;
    std::vector<float> facing_yaws;
    float first_transition_phase = -1.0f;
    bool observed_transition_diagnostics = false;

    for (int step = 0; step < 360; ++step) {
        const bool was_transitioning = controller.IsTransitioning();

        controller.Update(delta_seconds, request);

        // The frame a transition is scheduled, CurrentPhase() reports the source
        // clip phase that passed the gate (post-increment, pre-finalize).
        if (!was_transitioning && controller.IsTransitioning() && first_transition_phase < 0.0f) {
            first_transition_phase = controller.CurrentPhase();
        }

        assert(controller.CurrentParameter().size() == 1);
        if (controller.IsTransitioning()) {
            const std::optional<pmg::RuntimeTransitionDiagnostics> diagnostics =
                controller.ActiveTransitionDiagnostics();
            assert(diagnostics.has_value());
            assert(diagnostics->source_node == node);
            assert(diagnostics->target_node == node);
            assert(diagnostics->source_parameter.size() == 1);
            assert(diagnostics->requested_target_parameter == request.desired_parameter);
            assert(diagnostics->actual_target_parameter.size() == 1);
            assert(diagnostics->reachable_target_box.Contains(
                diagnostics->actual_target_parameter));
            assert(diagnostics->blend_duration_seconds > 0.0f);
            assert(diagnostics->blend_progress >= 0.0f);
            assert(diagnostics->blend_progress <= 1.0f);
            assert(std::abs(
                       diagnostics->metric_window_span_seconds -
                       (4.0f / kFramesPerSecond)) <
                   1.0e-6f);
            assert(
                diagnostics->runtime_windows
                    .source.sampled_frames.size() == 5);
            assert(
                diagnostics->runtime_windows
                    .target.sampled_frames.size() == 5);
            // Default runtime blend placement is PMG centered (§5.2.1): the
            // blend window is centered on the stored optimal transition point.
            assert(
                diagnostics->transition_window_convention ==
                pmg::TransitionWindowConvention::kPmgCentered);

            const int source_reference_frame = static_cast<int>(std::lround(
                diagnostics->source_transition_phase *
                static_cast<float>(kRuntimeFrameCount - 1)));
            const int target_reference_frame = static_cast<int>(std::lround(
                diagnostics->target_transition_phase *
                static_cast<float>(kRuntimeFrameCount - 1)));
            const pmg::TransitionFrameWindows expected_windows =
                pmg::ResolveTransitionFrameWindows(
                    kRuntimeFrameCount, kRuntimeFrameCount,
                    source_reference_frame, target_reference_frame,
                    /*window_size=*/5,
                    pmg::TransitionWindowConvention::kPmgCentered);
            assert(
                diagnostics->runtime_windows.source.sampled_frames ==
                expected_windows.source.sampled_frames);
            assert(
                diagnostics->runtime_windows.target.sampled_frames ==
                expected_windows.target.sampled_frames);
            observed_transition_diagnostics = true;
        } else {
            assert(!controller.ActiveTransitionDiagnostics().has_value());
        }

        const pmg::Pose pose = controller.CurrentPose();
        world_positions.push_back(pose.root_position);
        facing_yaws.push_back(WorldFacingYaw(pose));
    }

    // At least a couple of self-transitions occurred over the run.
    assert(controller.CompletedTransitions() >= 2);
    assert(observed_transition_diagnostics);

    // PMG centered blend (§5.2.1): the blend window is centered on the stored
    // optimal transition point, so scheduling gates about half a window before
    // it (then lands within one sampled frame of that gate).
    const int kHalfWindow = 5 / 2;  // default transition_blend_frames / 2
    const float half_window_phase =
        static_cast<float>(kHalfWindow) /
        static_cast<float>(kRuntimeFrameCount - 1);
    const float centered_gate = source_phase_gate - half_window_phase;
    assert(first_transition_phase >= centered_gate - 1.0e-4f);
    assert(first_transition_phase <= centered_gate + 0.05f);

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

    assert(mean_step > 1.0e-4f);          // the character actually moved
    assert(max_step < 6.0f * mean_step);  // a pop would be ~one stride (>> mean)

    // Facing direction is continuous across transitions (no yaw pop).
    float max_facing_delta = 0.0f;
    for (std::size_t i = 1; i < facing_yaws.size(); ++i) {
        max_facing_delta = std::max(max_facing_delta, WrappedDelta(facing_yaws[i], facing_yaws[i - 1]));
    }
    assert(max_facing_delta < 0.2f);  // radians; a misaligned turn would jump far more

    // Free-running loop continuity: with no transition requested, completed
    // cycles fold into the world placement, so the clip-local wrap never
    // teleports the root back to the start and the character keeps advancing.
    {
        pmg::RuntimeController free_runner(graph, alignment, walk_config);
        free_runner.Start(node, {0.5f}, kFramesPerSecond);
        const pmg::RuntimeControlRequest no_request;  // desired_node = -1

        std::vector<pmg::Vec3> free_positions;
        free_positions.push_back(free_runner.CurrentPose().root_position);
        for (int step = 0; step < 120; ++step) {  // ~5 clip cycles
            free_runner.Update(delta_seconds, no_request);
            free_positions.push_back(free_runner.CurrentPose().root_position);
        }

        float free_total = 0.0f;
        float free_max = 0.0f;
        for (std::size_t i = 1; i < free_positions.size(); ++i) {
            const float step = (free_positions[i] - free_positions[i - 1]).Norm();
            free_total += step;
            free_max = std::max(free_max, step);
        }
        const float free_mean =
            free_total / static_cast<float>(free_positions.size() - 1);
        assert(free_mean > 1.0e-4f);
        assert(free_max < 6.0f * free_mean);  // a wrap teleport would be ~one clip span

        // Net displacement keeps growing across loops (no snap-back).
        const float net = (free_positions.back() - free_positions.front()).Norm();
        assert(net > 2.0f * kStrideUnits);  // several cycles of forward travel
    }

    // D3/D4 regression: a 9-frame directional transition starting near source
    // phase 0.95 must cross the source cycle boundary without freezing or
    // popping. Nine sampled frames span eight runtime frame intervals.
    {
        int boundary_node = -1;
        const pmg::ParametricMotionGraph boundary_graph =
            MakeBoundaryCrossingGraph(boundary_node);
        pmg::RootOnlyAlignment boundary_alignment;
        pmg::RuntimeControllerConfig runtime_config;
        runtime_config.transition_blend_frames = 9;
        runtime_config.cyclic_nodes.assign(boundary_graph.NumNodes(), true);
        pmg::RuntimeController boundary_controller(
            boundary_graph, boundary_alignment, runtime_config);
        boundary_controller.Start(
            boundary_node, {0.5f}, kFramesPerSecond);

        pmg::RuntimeControlRequest boundary_request;
        boundary_request.desired_node = boundary_node;
        boundary_request.desired_parameter = {0.5f};

        std::vector<pmg::Vec3> boundary_positions;
        boundary_positions.push_back(
            boundary_controller.CurrentPose().root_position);
        bool crossed_source_boundary_during_blend = false;
        int transition_update_count = 0;
        float previous_phase = boundary_controller.CurrentPhase();
        for (int step = 0; step < 60; ++step) {
            const bool was_transitioning =
                boundary_controller.IsTransitioning();
            boundary_controller.Update(delta_seconds, boundary_request);
            const bool is_transitioning =
                boundary_controller.IsTransitioning();
            const float phase = boundary_controller.CurrentPhase();
            if (was_transitioning && phase < previous_phase) {
                crossed_source_boundary_during_blend = true;
            }
            if (was_transitioning) {
                ++transition_update_count;
            }
            boundary_positions.push_back(
                boundary_controller.CurrentPose().root_position);
            previous_phase = phase;
            if (!is_transitioning &&
                boundary_controller.CompletedTransitions() > 0) {
                break;
            }
        }

        assert(crossed_source_boundary_during_blend);
        assert(transition_update_count >= 8);
        assert(transition_update_count <= 9);
        assert(boundary_controller.CompletedTransitions() == 1);

        float boundary_total_step = 0.0f;
        float boundary_max_step = 0.0f;
        for (std::size_t i = 1; i < boundary_positions.size(); ++i) {
            const float step =
                (boundary_positions[i] - boundary_positions[i - 1]).Norm();
            boundary_total_step += step;
            boundary_max_step = std::max(boundary_max_step, step);
        }
        const float boundary_mean_step =
            boundary_total_step /
            static_cast<float>(boundary_positions.size() - 1);
        assert(boundary_mean_step > 1.0e-4f);
        assert(boundary_max_step < 6.0f * boundary_mean_step);
    }

    // P2: cyclic metadata, not self-edge topology, decides pre-roll wrap. The
    // same self-edge with a negative centered pre-roll wraps when the node is
    // marked cyclic and clamps when it is not.
    {
        int wrap_node = -1;
        const pmg::ParametricMotionGraph wrap_graph =
            MakePreRollWrapGraph(wrap_node);
        pmg::RootOnlyAlignment wrap_alignment;

        auto first_transition_diagnostics =
            [&](bool cyclic) -> pmg::RuntimeTransitionDiagnostics {
            pmg::RuntimeControllerConfig cfg;  // default kWrapCyclicSelfEdges
            if (cyclic) {
                cfg.cyclic_nodes.assign(wrap_graph.NumNodes(), true);
            }
            pmg::RuntimeController c(wrap_graph, wrap_alignment, cfg);
            c.Start(wrap_node, {0.5f}, kFramesPerSecond);
            pmg::RuntimeControlRequest req;
            req.desired_node = wrap_node;
            req.desired_parameter = {0.5f};
            for (int step = 0; step < 120; ++step) {
                c.Update(delta_seconds, req);
                if (c.IsTransitioning()) {
                    return *c.ActiveTransitionDiagnostics();
                }
            }
            assert(false && "expected a scheduled transition");
            return {};
        };

        const pmg::RuntimeTransitionDiagnostics cyclic_diag =
            first_transition_diagnostics(true);
        const pmg::RuntimeTransitionDiagnostics acyclic_diag =
            first_transition_diagnostics(false);

        // The centered pre-roll is negative for both; only cyclicity differs.
        assert(cyclic_diag.raw_target_start_seconds < 0.0f);
        assert(acyclic_diag.raw_target_start_seconds < 0.0f);
        assert(cyclic_diag.target_preroll_wrapped);
        assert(!acyclic_diag.target_preroll_wrapped);
        // The non-cyclic self-edge clamps its scheduled start at phase 0.
        assert(acyclic_diag.scheduled_target_start_seconds >= 0.0f);
    }

    return 0;
}
