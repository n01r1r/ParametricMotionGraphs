#include "ViewerRuntimeModule.h"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

constexpr float kFramesPerSecond = 30.0f;
constexpr float kDeltaSeconds = 1.0f / kFramesPerSecond;
constexpr int kFrameCount = 8;

bool NearlyEqual(float left, float right) {
    return std::abs(left - right) <= 1.0e-5f;
}

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.parent_index = -1;
    skeleton.joints.push_back(root);
    pmg::Joint hip;
    hip.parent_index = 0;
    hip.offset = {0.5f, 0.0f, 0.0f};
    skeleton.joints.push_back(hip);
    pmg::Joint knee;
    knee.parent_index = 1;
    knee.offset = {0.0f, -1.0f, 0.0f};
    skeleton.joints.push_back(knee);
    return skeleton;
}

const pmg::Skeleton& FixtureSkeleton() {
    static const pmg::Skeleton skeleton = MakeSkeleton();
    return skeleton;
}

pmg::MotionClip MakeClip(float parameter) {
    pmg::MotionClip clip;
    clip.frames_per_second = kFramesPerSecond;
    for (int frame_index = 0; frame_index < kFrameCount; ++frame_index) {
        const float phase = static_cast<float>(frame_index) /
                            static_cast<float>(kFrameCount - 1);
        pmg::Pose pose;
        pose.root_position = {parameter, 0.0f, phase};
        pose.local_rotations.push_back({});
        pose.local_rotations.push_back(
            pmg::EulerAxisRotation('X', parameter * 10.0f * phase));
        pose.local_rotations.push_back(
            pmg::EulerAxisRotation('X', -parameter * 10.0f * phase));
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::ParametricMotionGraph MakeGraph(bool with_self_edge) {
    pmg::ParametricMotionSpace space("runtime_test", 1);
    space.AddExample({0.0f}, MakeClip(0.0f));
    space.AddExample({1.0f}, MakeClip(1.0f));

    pmg::ParametricMotionGraph graph;
    const int node = graph.AddNode("node", space);

    if (with_self_edge) {
        pmg::PmgEdge edge;
        edge.source_node = node;
        edge.target_node = node;
        edge.samples.push_back({{0.5f}, {{0.0f}, {1.0f}}, 0.75f, 0.25f});
        graph.AddEdge(edge);
    }

    return graph;
}

void InstallRuntime(pmgviewer::ViewerRuntimeModule& runtime, bool with_self_edge) {
    pmg::RuntimeControllerConfig config;
    config.transition_blend_frames = 3;
    config.convention = pmg::TransitionWindowConvention::kKovarDirectional;
    runtime.Install(
        MakeGraph(with_self_edge), FixtureSkeleton(), config, kFramesPerSecond, {0.5f});
}

pmgviewer::ViewerRuntimeRequest Request(float parameter) {
    return {0, pmg::ParameterVector{parameter}};
}

pmgviewer::ViewerRuntimeSnapshot RunUntilTransition(
    pmgviewer::ViewerRuntimeModule& runtime, float parameter) {
    for (int step = 0; step < 60; ++step) {
        runtime.Update(kDeltaSeconds, Request(parameter));
        pmgviewer::ViewerRuntimeSnapshot snapshot = runtime.Snapshot();
        if (snapshot.status == pmgviewer::ViewerRuntimeStatus::kTransitionActive) {
            return snapshot;
        }
    }
    assert(false && "expected transition to schedule");
    return {};
}

void ViewerRuntimeModule_InstallBeforeUpdate() {
    pmgviewer::ViewerRuntimeModule runtime;
    bool threw = false;
    try {
        runtime.Update(kDeltaSeconds, Request(0.5f));
    } catch (const std::logic_error&) {
        threw = true;
    }
    assert(threw);
}

void ViewerRuntimeModule_RawProjectedActualContract() {
    pmgviewer::ViewerRuntimeModule runtime;
    InstallRuntime(runtime, false);
    runtime.Update(kDeltaSeconds, Request(2.0f));

    const pmgviewer::ViewerRuntimeSnapshot snapshot = runtime.Snapshot();
    assert(snapshot.requested_raw == pmg::ParameterVector{2.0f});
    assert(snapshot.requested_projected == pmg::ParameterVector{1.0f});
    assert(snapshot.runtime_actual == pmg::ParameterVector{0.5f});
}

void ViewerRuntimeModule_PhaseGateStatus() {
    pmgviewer::ViewerRuntimeModule runtime;
    InstallRuntime(runtime, true);
    runtime.Update(kDeltaSeconds, Request(0.8f));
    assert(runtime.Snapshot().status ==
           pmgviewer::ViewerRuntimeStatus::kWaitingForPhaseGate);
}

void ViewerRuntimeModule_SameParameterNoOp() {
    pmgviewer::ViewerRuntimeModule runtime;
    InstallRuntime(runtime, true);
    runtime.Update(kDeltaSeconds, Request(0.5f));
    assert(runtime.Snapshot().status ==
           pmgviewer::ViewerRuntimeStatus::kNoOpSameParameter);
}

void ViewerRuntimeModule_ChangedParameterSchedules() {
    pmgviewer::ViewerRuntimeModule runtime;
    InstallRuntime(runtime, true);
    const pmgviewer::ViewerRuntimeSnapshot snapshot =
        RunUntilTransition(runtime, 0.8f);
    assert(snapshot.active_transition.has_value());
    assert(snapshot.runtime_actual == pmg::ParameterVector{0.8f});
    assert(snapshot.transition_markers.size() == 1);
}

void ViewerRuntimeModule_ResetClearsRuntimeButKeepsGraph() {
    pmgviewer::ViewerRuntimeModule runtime;
    InstallRuntime(runtime, true);
    RunUntilTransition(runtime, 0.8f);

    runtime.Reset();
    const pmgviewer::ViewerRuntimeSnapshot snapshot = runtime.Snapshot();
    assert(snapshot.ready);
    assert(runtime.Graph().NumNodes() == 1);
    assert(snapshot.completed_transitions == 0);
    assert(snapshot.active_transition == std::nullopt);
    assert(snapshot.runtime_path_points.size() == 1);
}

void ViewerRuntimeModule_ClearGraphRemovesController() {
    pmgviewer::ViewerRuntimeModule runtime;
    InstallRuntime(runtime, true);
    runtime.Clear();

    const pmgviewer::ViewerRuntimeSnapshot snapshot = runtime.Snapshot();
    assert(!snapshot.ready);
    assert(runtime.Graph().NumNodes() == 0);
    assert(snapshot.status == pmgviewer::ViewerRuntimeStatus::kNotInstalled);
}

void ViewerRuntimeModule_ClearTrace() {
    pmgviewer::ViewerRuntimeModule runtime;
    InstallRuntime(runtime, false);
    for (int step = 0; step < 3; ++step) {
        runtime.Update(kDeltaSeconds, Request(0.7f));
    }
    assert(runtime.Snapshot().runtime_path_points.size() > 1);

    runtime.ClearTrace();
    assert(runtime.Snapshot().runtime_path_points.size() == 1);
}

void ViewerRuntimeModule_SnapshotOwnsVectors() {
    pmgviewer::ViewerRuntimeModule runtime;
    InstallRuntime(runtime, false);
    runtime.Update(kDeltaSeconds, Request(0.7f));

    pmgviewer::ViewerRuntimeSnapshot snapshot = runtime.Snapshot();
    const std::size_t path_size = snapshot.runtime_path_points.size();
    snapshot.runtime_path_points.push_back({99.0f, 99.0f});
    snapshot.transition_markers.push_back({{99.0f, 99.0f}, 1, 2});

    const pmgviewer::ViewerRuntimeSnapshot fresh = runtime.Snapshot();
    assert(fresh.runtime_path_points.size() == path_size);
    assert(fresh.transition_markers.empty());
}

void ViewerRuntimeModule_DeterministicForSameInput() {
    pmgviewer::ViewerRuntimeModule left;
    pmgviewer::ViewerRuntimeModule right;
    InstallRuntime(left, true);
    InstallRuntime(right, true);

    for (int step = 0; step < 30; ++step) {
        left.Update(kDeltaSeconds, Request(0.8f));
        right.Update(kDeltaSeconds, Request(0.8f));
    }

    const pmgviewer::ViewerRuntimeSnapshot a = left.Snapshot();
    const pmgviewer::ViewerRuntimeSnapshot b = right.Snapshot();
    assert(a.status == b.status);
    assert(a.current_node == b.current_node);
    assert(a.completed_transitions == b.completed_transitions);
    assert(a.runtime_actual == b.runtime_actual);
    assert(a.runtime_path_points.size() == b.runtime_path_points.size());
    for (std::size_t index = 0; index < a.runtime_path_points.size(); ++index) {
        assert(NearlyEqual(
            a.runtime_path_points[index].x, b.runtime_path_points[index].x));
        assert(NearlyEqual(
            a.runtime_path_points[index].y, b.runtime_path_points[index].y));
    }
}

}  // namespace

int main() {
    ViewerRuntimeModule_InstallBeforeUpdate();
    ViewerRuntimeModule_RawProjectedActualContract();
    ViewerRuntimeModule_PhaseGateStatus();
    ViewerRuntimeModule_SameParameterNoOp();
    ViewerRuntimeModule_ChangedParameterSchedules();
    ViewerRuntimeModule_ResetClearsRuntimeButKeepsGraph();
    ViewerRuntimeModule_ClearGraphRemovesController();
    ViewerRuntimeModule_ClearTrace();
    ViewerRuntimeModule_SnapshotOwnsVectors();
    ViewerRuntimeModule_DeterministicForSameInput();
    return 0;
}
