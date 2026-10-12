#include "pmg_test.h"
#include "pmg/StreamingFootLock.h"
#include "pmg/RuntimeController.h"
#include "pmg/MotionSpacePreparation.h"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    pmg::Skeleton skeleton;
    skeleton.joints = {{"hip", -1, {}, {}}, {"knee", 0, {0,-1,0}, {}}, {"ankle", 1, {0,-1,0}, {}}};
    pmg::Pose raw; raw.root_position = {0, 1.825f, 0};
    raw.local_rotations = {pmg::Quaternion::Identity(), pmg::EulerAxisRotation('Z', 34.0f), pmg::Quaternion::Identity()};
    pmg::StreamingFootLock lock(skeleton, {2});
    const auto anchor = pmg::ComputeForwardKinematics(skeleton, raw)[2];
    float max_error = 0;
    for (int i = 0; i < 60; ++i) {
        raw.root_position.x = i * 0.005f;
        raw.local_rotations[2] = pmg::EulerAxisRotation('Y', static_cast<float>(i));
        auto corrected = lock.Update(raw, {true}, 1.0f/60);
        auto world = pmg::ComputeForwardKinematics(skeleton, corrected);
        max_error = std::max(max_error, (world[2].position - anchor.position).Norm());
        assert(std::abs(std::abs(pmg::QuaternionDot(world[2].rotation, anchor.rotation)) - 1) < 1e-4f);
        assert(std::abs((world[1].position-world[0].position).Norm()-1) < 1e-5f);
    }
    assert(max_error < 1e-3f);
    for (int i = 0; i < 30; ++i) {
        const auto corrected = lock.Update(raw, {false}, 1.0f/60);
        if (i == 29) assert((pmg::ComputeJointWorldPositions(skeleton, corrected)[2] -
                           pmg::ComputeJointWorldPositions(skeleton, raw)[2]).Norm() < 1e-4f);
    }
    lock.Reset();
    raw.root_position.x = 100;
    assert(lock.Update(raw, {true}, 0).root_position.x == 100);
    std::cout << "synthetic raw drift=0.295; corrected max anchor error=" << max_error << '\n';

    // Real corpus + runtime path: contact locking must run after world-space
    // blending, survive a self-transition, and leave repeated reads idempotent.
    const auto prepared = pmg::PrepareMotionSpaces(pmg::LoadGraphSpec(PMG_SOURCE_DIR "/specs/cmu_walk_1d.pmg_spec"));
    const auto& space = prepared.Node("cmu_walk").production;
    pmg::ParametricMotionGraph graph;
    graph.AddNode("walk", space);
    pmg::PmgEdge edge; edge.source_node = edge.target_node = 0;
    edge.samples.push_back({{23}, {{17.1f}, {29.9f}}, 0.8f, 0.2f});
    graph.AddEdge(edge);
    pmg::PointCloudAlignment alignment(prepared.skeleton);
    pmg::RuntimeController controller(graph, alignment);
    std::vector<int> feet;
    for (int i=0; i<prepared.skeleton.NumJoints(); ++i) {
        const auto& name = prepared.skeleton.joints[i].name;
        if (name == "LeftFoot" || name == "RightFoot") feet.push_back(i);
    }
    assert(feet.size() == 2);
    controller.Start(0, {23}, 60);
    controller.EnableFootLocking(prepared.skeleton, feet);
    float max_anchor = 0, max_pelvis = 0;
    int planted_frames = 0;
    for (int i=0; i<600; ++i) {
        controller.Update(1.0f/60, {0, {i < 300 ? 29.9f : 17.1f}});
        const auto diagnostics = controller.FootLockDiagnostics();
        planted_frames += diagnostics.planted_feet > 0;
        max_anchor = std::max(max_anchor, diagnostics.max_anchor_error);
        max_pelvis = std::max(max_pelvis, diagnostics.root_correction);
        const auto pose = controller.CurrentPose();
        assert(std::isfinite(pose.root_position.x));
        assert((pose.root_position-controller.CurrentPose().root_position).Norm() == 0);
    }
    std::cout << "CMU runtime: planted frames=" << planted_frames << "/600; transitions="
              << controller.CompletedTransitions() << "; max anchor error=" << max_anchor
              << "; max pelvis correction=" << max_pelvis << '\n';
    assert(planted_frames > 30);
    assert(controller.CompletedTransitions() >= 2);
    assert(max_anchor < 0.01f);
}
