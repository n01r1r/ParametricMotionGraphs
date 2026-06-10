#include "pmg/AlignmentStrategy.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/ParametricMotionSpace.h"
#include "pmg/PmgBuilder.h"
#include "pmg/RuntimeController.h"
#include "pmg/Skeleton.h"

#include <cassert>
#include <cmath>

namespace {

constexpr float kTwoPi = 6.28318530718f;

bool Near(float a, float b, float eps = 1.0e-4f) {
    return std::abs(a - b) < eps;
}

pmg::MotionClip SingleFramePose(const pmg::Vec3& root) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;
    pmg::Pose pose;
    pose.root_position = root;
    pose.local_rotations.push_back(pmg::Quaternion::Identity());  // faces +z
    clip.frames.push_back(pose);
    return clip;
}

// --- Fixtures for the fake-strategy controller test (a looping walk) ---------

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root; root.parent_index = -1; skeleton.joints.push_back(root);
    pmg::Joint hip; hip.parent_index = 0; hip.offset = {0.6f, 0.0f, 0.0f};
    skeleton.joints.push_back(hip);
    pmg::Joint knee; knee.parent_index = 1; knee.offset = {0.0f, -1.5f, 0.0f};
    skeleton.joints.push_back(knee);
    return skeleton;
}

pmg::MotionClip MakeWalkClip(float parameter) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;
    constexpr int kFrames = 24;
    const float swing = 20.0f + 2.0f * parameter;
    for (int i = 0; i < kFrames; ++i) {
        const float phase = static_cast<float>(i) / static_cast<float>(kFrames - 1);
        pmg::Pose pose;
        pose.root_position = {0.0f, 0.1f, phase * 3.0f};
        pose.local_rotations.push_back(pmg::EulerAxisRotation('Y', phase * 20.0f));
        pose.local_rotations.push_back(pmg::EulerAxisRotation('X', swing * std::sin(kTwoPi * phase)));
        pose.local_rotations.push_back(pmg::EulerAxisRotation('X', -swing * std::sin(kTwoPi * phase)));
        clip.frames.push_back(pose);
    }
    return clip;
}

// A fake adapter at the seam: records every call, returns a fixed transform.
// Proves the controller's transition timing is independent of how alignment is
// resolved.
struct CountingAlignment : pmg::AlignmentStrategy {
    mutable int calls = 0;
    pmg::RigidTransform2D Resolve(const pmg::AlignmentContext&) const override {
        ++calls;
        return pmg::RigidTransform2D::Identity();
    }
};

}  // namespace

int main() {
    // StoredAlignment returns exactly the alignment baked on the transition.
    {
        pmg::InterpolatedTransition transition;
        transition.alignment_yaw = 0.42f;
        transition.alignment_dx = -1.3f;
        transition.alignment_dz = 2.7f;
        const pmg::MotionClip clip = SingleFramePose({0.0f, 0.0f, 0.0f});
        const pmg::AlignmentContext context{clip, clip, 0.0f, transition};

        const pmg::StoredAlignment strategy;
        const pmg::RigidTransform2D out = strategy.Resolve(context);
        assert(Near(out.yaw, 0.42f) && Near(out.dx, -1.3f) && Near(out.dz, 2.7f));
    }

    // RootOnlyAlignment: identity-facing roots -> yaw 0, translation = src - tgt.
    {
        const pmg::MotionClip source = SingleFramePose({2.0f, 0.5f, 3.0f});
        const pmg::MotionClip target = SingleFramePose({-1.0f, 0.0f, 4.0f});
        pmg::InterpolatedTransition transition;
        transition.target_transition_phase = 0.0f;
        const pmg::AlignmentContext context{source, target, 0.0f, transition};

        const pmg::RootOnlyAlignment strategy;
        const pmg::RigidTransform2D out = strategy.Resolve(context);
        assert(Near(out.yaw, 0.0f));
        assert(Near(out.dx, 3.0f));   // 2 - (-1)
        assert(Near(out.dz, -1.0f));  // 3 - 4
    }

    // The seam decouples timing from alignment: an arbitrary adapter plugs into
    // the controller, gets called on transitions, and the run still completes.
    {
        const pmg::Skeleton skeleton = MakeSkeleton();
        pmg::ParametricMotionSpace space("walk", 1);
        space.AddExample({0.0f}, MakeWalkClip(0.0f));
        space.AddExample({1.0f}, MakeWalkClip(1.0f));

        pmg::ParametricMotionGraph graph;
        const int node = graph.AddNode("walk", space);
        pmg::PmgBuilderConfig config;
        config.source_sample_count = 4;
        config.target_sample_count = 8;
        config.generated_frame_count = 24;
        config.good_transition_threshold = 1.0e9f;
        config.bad_transition_threshold = 2.0e9f;
        pmg::PmgEdge edge =
            pmg::PmgBuilder::BuildEdge(skeleton, node, node, space, space, config);
        assert(!edge.samples.empty());
        graph.AddEdge(edge);

        CountingAlignment alignment;
        pmg::RuntimeController controller(graph, alignment);
        controller.Start(node, {0.5f}, 24, 30.0f);

        pmg::RuntimeControlRequest request;
        request.desired_node = node;
        request.desired_parameter = {0.5f};
        for (int step = 0; step < 240; ++step) {
            controller.Update(1.0f / 30.0f, request);
        }

        assert(alignment.calls >= 1);                 // the seam was exercised
        assert(controller.CompletedTransitions() >= 1);
    }

    return 0;
}
