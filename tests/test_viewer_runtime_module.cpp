#include "pmg_test.h"
#include "ViewerRuntimeModule.h"

#include <cassert>
#include <limits>
#include <type_traits>

pmg::ParametricMotionGraph Graph() {
    pmg::MotionClip clip; clip.frames_per_second = 30;
    pmg::Pose pose; pose.local_rotations.resize(1);
    clip.frames = {pose, pose};
    pmg::ParametricMotionSpace space("walk", 1); space.AddExample({0}, clip);
    pmg::ParametricMotionGraph graph; graph.AddNode("walk", space);
    return graph;
}
pmg::Skeleton Skeleton() {
    pmg::Skeleton skeleton; skeleton.joints.resize(1); return skeleton;
}
int main() {
    static_assert(!std::is_move_constructible_v<pmgviewer::ViewerRuntimeModule>);
    pmgviewer::ViewerRuntimeModule module;
    module.Install(Graph(), Skeleton(), {}, 30, {0});
    assert(module.Ready());
    pmgviewer::ViewerRuntimeRequest request; request.desired_node = 0;
    request.desired_parameter_raw = pmg::ParameterVector{0};
    module.Update(1.0f/60, request);
    assert(module.Snapshot().status == pmgviewer::ViewerRuntimeStatus::kNoOpSameParameter);
    request.desired_parameter_raw = pmg::ParameterVector{std::numeric_limits<float>::quiet_NaN()};
    module.Update(1.0f/60, request);
    assert(module.Snapshot().status == pmgviewer::ViewerRuntimeStatus::kInvalidRequest);
    assert(!module.Snapshot().requested_projected);
    bool threw = false;
    try { module.Install(Graph(), Skeleton(), {}, 0, {0}); } catch (const std::exception&) { threw = true; }
    assert(threw && !module.Ready());
    module.Install(Graph(), Skeleton(), {}, 30, {0});
    assert(!module.Snapshot().requested_raw);
    module.Clear();
    assert(!module.Snapshot().ready);
}
