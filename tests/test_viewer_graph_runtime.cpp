#include "PmgViewerWorkspace.h"
#include "ViewerRuntimeModule.h"

#include <cassert>
#include <vector>

namespace {

bool NearlyEqual(float left, float right) {
    return std::abs(left - right) <= 1.0e-5f;
}

void TestDesiredParameterUsesFull2DVector() {
    pmg::ParametricMotionSpace space("test_2d", 2);
    
    // Create a 2D triangulated explicit support node
    std::vector<pmg::ParameterVector> vertices = {
        {0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}
    };
    
    // Add examples to space so Domain/MinParameter does not assert
    for (const auto& v : vertices) {
        space.AddExample(v, pmg::MotionClip{"dummy", 30.0f, {pmg::Pose()}});
    }

    std::vector<std::array<int, 3>> triangles = {{0, 1, 2}};
    space.SetParameterSupport(pmg::ParameterSupport::CreateTriangulated2D(vertices, triangles));

    // Test with full 2D vector valid
    pmg::ParameterVector desired = pmgviewer::ResolveDesiredParameterForNode(
        space, 0.5f, true, {0.15f, 0.75f});
    assert(desired.size() == 2);
    assert(NearlyEqual(desired[0], 0.15f));
    assert(NearlyEqual(desired[1], 0.75f));

    // Test fallback behavior for 1D logic without full vector
    pmg::ParameterVector fallback = pmgviewer::ResolveDesiredParameterForNode(
        space, 0.5f, false, {});
    assert(fallback.size() == 2);
    assert(NearlyEqual(fallback[0], 0.5f)); // 1D slider
}

void TestRuntimeModuleLifecycle() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.name = "root";
    skeleton.joints.push_back(root);

    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;
    pmg::Pose pose;
    pose.local_rotations.push_back({});
    clip.frames = {pose, pose};

    pmg::ParametricMotionSpace space("one_dimensional", 1);
    space.AddExample({0.0f}, clip);
    space.AddExample({1.0f}, clip);
    pmg::ParametricMotionGraph graph;
    graph.AddNode("node", space);

    pmgviewer::ViewerRuntimeModule runtime;
    runtime.Install(graph, skeleton, {}, 30.0f, {0.5f});
    runtime.Update(1.0f / 30.0f, {0, pmg::ParameterVector{0.75f}});

    const pmgviewer::ViewerRuntimeSnapshot snapshot = runtime.Snapshot();
    assert(snapshot.ready);
    assert(snapshot.current_node == 0);
    assert(snapshot.runtime_actual == pmg::ParameterVector{0.5f});
    assert(snapshot.requested_raw == pmg::ParameterVector{0.75f});
    assert(snapshot.requested_projected == pmg::ParameterVector{0.75f});
    assert(snapshot.completed_transitions == 0);
    assert(snapshot.status ==
           pmgviewer::ViewerRuntimeStatus::kNoFeasibleTransition);
    assert(!snapshot.runtime_path_points.empty());

    runtime.Update(1.0f / 30.0f, {0, pmg::ParameterVector{0.5f}});
    assert(runtime.Snapshot().status ==
           pmgviewer::ViewerRuntimeStatus::kNoOpSameParameter);
}

} // namespace

int main() {
    TestDesiredParameterUsesFull2DVector();
    TestRuntimeModuleLifecycle();
    return 0;
}
