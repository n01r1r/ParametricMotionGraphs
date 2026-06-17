#include "pmg/PmgBuilder.h"
#include "pmg/ParametricMotionSpace.h"
#include "pmg/Skeleton.h"
#include "pmg/TransitionTypes.h"

#include <cassert>
#include <cmath>

namespace {

constexpr float kTwoPi = 6.28318530718f;

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.parent_index = -1;
    skeleton.joints.push_back(root);
    pmg::Joint a;
    a.parent_index = 0;
    a.offset = {1.0f, 0.0f, 0.0f};
    skeleton.joints.push_back(a);
    pmg::Joint b;
    b.parent_index = 1;
    b.offset = {0.0f, -1.0f, 0.0f};
    skeleton.joints.push_back(b);
    return skeleton;
}

// Periodic ("looping") clip: phase 0 and phase 1 share a pose, so late-phase
// frames align well with early-phase frames (good self-transitions).
pmg::MotionClip MakeLoopClip(float parameter, char axis, float amplitude_degrees) {
    pmg::MotionClip clip;
    clip.frames_per_second = 30.0f;
    constexpr int kFrameCount = 16;
    for (int frame_index = 0; frame_index < kFrameCount; ++frame_index) {
        const float phase = static_cast<float>(frame_index) / static_cast<float>(kFrameCount - 1);
        const float angle = amplitude_degrees * parameter * std::sin(kTwoPi * phase);
        pmg::Pose pose;
        pose.root_position = {0.3f * phase, 0.1f, 0.0f};
        pose.local_rotations.push_back(pmg::Quaternion::Identity());
        pose.local_rotations.push_back(pmg::EulerAxisRotation(axis, angle));
        pose.local_rotations.push_back(pmg::EulerAxisRotation(axis, 0.5f * angle));
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::MotionClip MakeLoopClip2D(const pmg::ParameterVector& parameter) {
    return MakeLoopClip(parameter[0] + 0.5f * parameter[1], 'X', 20.0f);
}

pmg::ParametricMotionSpace MakeSpace(char axis, float amplitude_degrees) {
    pmg::ParametricMotionSpace space("space", 1);
    space.AddExample({0.0f}, MakeLoopClip(0.0f, axis, amplitude_degrees));
    space.AddExample({0.5f}, MakeLoopClip(0.5f, axis, amplitude_degrees));
    space.AddExample({1.0f}, MakeLoopClip(1.0f, axis, amplitude_degrees));
    return space;
}

pmg::ParametricMotionSpace MakeTriangleSpace() {
    pmg::ParametricMotionSpace space("triangle", 2);
    space.AddExample({0.0f, 0.0f}, MakeLoopClip2D({0.0f, 0.0f}));
    space.AddExample({1.0f, 0.0f}, MakeLoopClip2D({1.0f, 0.0f}));
    space.AddExample({0.0f, 1.0f}, MakeLoopClip2D({0.0f, 1.0f}));
    space.SetParameterSupport(pmg::ParameterSupport(space.ExampleParameters()));
    return space;
}

bool InUnitTriangle(const pmg::ParameterVector& parameter) {
    return parameter.size() == 2 &&
           parameter[0] >= -1.0e-5f &&
           parameter[1] >= -1.0e-5f &&
           parameter[0] + parameter[1] <= 1.0f + 1.0e-5f;
}

pmg::PmgBuilderConfig SmallConfig() {
    pmg::PmgBuilderConfig config;
    config.source_sample_count = 4;
    config.target_sample_count = 8;
    config.generated_frame_count = 16;
    return config;
}

bool SamplesEqual(const pmg::TransitionSample& a, const pmg::TransitionSample& b) {
    if (a.source_parameter != b.source_parameter) return false;
    if (a.target_parameter_box.min_corner != b.target_parameter_box.min_corner) return false;
    if (a.target_parameter_box.max_corner != b.target_parameter_box.max_corner) return false;
    if (a.source_transition_phase != b.source_transition_phase) return false;
    if (a.target_transition_phase != b.target_transition_phase) return false;
    if (a.target_phase_samples.size() != b.target_phase_samples.size()) return false;
    for (std::size_t index = 0; index < a.target_phase_samples.size(); ++index) {
        const pmg::TargetTransitionPhaseSample& phase_a =
            a.target_phase_samples[index];
        const pmg::TargetTransitionPhaseSample& phase_b =
            b.target_phase_samples[index];
        if (phase_a.target_parameter != phase_b.target_parameter) return false;
        if (phase_a.source_transition_phase != phase_b.source_transition_phase) return false;
        if (phase_a.target_transition_phase != phase_b.target_transition_phase) return false;
    }
    return true;
}

}  // namespace

int main() {
    const pmg::Skeleton skeleton = MakeSkeleton();
    const pmg::ParametricMotionSpace space = MakeSpace('X', 20.0f);

    // IsEmpty: default + Empty() factory bound nothing; a populated box does not.
    {
        const pmg::ParameterAabb default_box;
        assert(default_box.IsEmpty());
        assert(pmg::ParameterAabb::Empty(2).IsEmpty());

        pmg::ParameterAabb populated;
        populated.ExpandToInclude({0.0f, 0.0f});
        populated.ExpandToInclude({1.0f, 2.0f});
        assert(!populated.IsEmpty());

        // A single point is a valid zero-volume box, not empty.
        pmg::ParameterAabb point;
        point.ExpandToInclude({3.0f});
        assert(!point.IsEmpty());
    }

    // ShrinkToExclude: a planted BAD region inside a GOOD bounding box is removed.
    {
        pmg::ParameterAabb box;
        box.ExpandToInclude({0.0f, 0.0f});
        box.ExpandToInclude({10.0f, 10.0f});
        const pmg::ParameterVector bad_a = {1.0f, 5.0f};
        const pmg::ParameterVector bad_b = {9.0f, 5.0f};
        box.ShrinkToExclude(bad_a, 0.01f);
        box.ShrinkToExclude(bad_b, 0.01f);
        assert(!box.Contains(bad_a));
        assert(!box.Contains(bad_b));
        assert(box.IsValid());

        // A point already outside is a no-op.
        pmg::ParameterAabb unit;
        unit.ExpandToInclude({0.0f});
        unit.ExpandToInclude({1.0f});
        unit.ShrinkToExclude({5.0f}, 0.01f);
        assert(unit.min_corner[0] == 0.0f && unit.max_corner[0] == 1.0f);
    }

    // ShrinkToExclude must also exclude centered BAD points and degenerate boxes.
    {
        pmg::ParameterAabb centered;
        centered.ExpandToInclude({0.0f});
        centered.ExpandToInclude({1.0f});
        const pmg::ParameterVector bad = {0.5f};
        centered.ShrinkToExclude(bad, 0.01f);
        assert(!centered.Contains(bad));

        pmg::ParameterAabb degenerate;
        degenerate.ExpandToInclude({0.5f});
        degenerate.ShrinkToExclude({0.5f}, 0.01f);
        assert(degenerate.IsEmpty() || !degenerate.Contains({0.5f}));
    }

    // Self-edge on a smooth space: non-empty, every stored box valid.
    {
        pmg::PmgBuilderConfig config = SmallConfig();
        config.good_transition_threshold = 1.0e9f;  // accept all targets as GOOD
        config.bad_transition_threshold = 2.0e9f;
        const pmg::PmgEdge edge =
            pmg::PmgBuilder::BuildEdge(skeleton, 0, 0, space, space, config);
        assert(!edge.samples.empty());
        for (const pmg::TransitionSample& sample : edge.samples) {
            assert(sample.target_parameter_box.IsValid());
            assert(!sample.target_phase_samples.empty());
            assert(sample.source_transition_phase >= 0.0f &&
                   sample.source_transition_phase <= 1.0f);
            for (const pmg::TargetTransitionPhaseSample& phase_sample :
                 sample.target_phase_samples) {
                assert(sample.target_parameter_box.Contains(
                    phase_sample.target_parameter));
            }
        }
    }

    // Random edge-builder samples come from simplex support, not the enclosing
    // AABB. For this triangle support, AABB sampling could produce x+y>1.
    {
        const pmg::ParametricMotionSpace triangle_space = MakeTriangleSpace();
        pmg::PmgBuilderConfig config = SmallConfig();
        config.source_sample_count = 20;
        config.target_sample_count = 20;
        config.include_example_parameters = false;
        config.good_transition_threshold = 1.0e9f;
        config.bad_transition_threshold = 2.0e9f;
        const pmg::PmgEdge edge = pmg::PmgBuilder::BuildEdge(
            skeleton, 0, 0, triangle_space, triangle_space, config);
        assert(!edge.samples.empty());
        for (const pmg::TransitionSample& sample : edge.samples) {
            assert(InUnitTriangle(sample.source_parameter));
            for (const pmg::TargetTransitionPhaseSample& phase_sample :
                 sample.target_phase_samples) {
                assert(InUnitTriangle(phase_sample.target_parameter));
            }
        }
    }

    // Deterministic given a fixed seed.
    {
        pmg::PmgBuilderConfig config = SmallConfig();
        assert(config.transition_metric_type ==
               pmg::TransitionMetricType::kKovarDirectionalPointCloud);
        config.good_transition_threshold = 1.0e9f;
        config.bad_transition_threshold = 2.0e9f;
        const pmg::PmgEdge first = pmg::PmgBuilder::BuildEdge(skeleton, 0, 0, space, space, config);
        const pmg::PmgEdge second = pmg::PmgBuilder::BuildEdge(skeleton, 0, 0, space, space, config);
        assert(first.samples.size() == second.samples.size());
        for (std::size_t i = 0; i < first.samples.size(); ++i) {
            assert(SamplesEqual(first.samples[i], second.samples[i]));
        }
    }

    // Opt-in dynamics metric uses a separate route but remains deterministic
    // under the same sampling seed. Thresholds here are extension-specific and
    // deliberately broad; old Kovar thresholds are not reused as claims.
    {
        pmg::PmgBuilderConfig config = SmallConfig();
        config.transition_metric_type =
            pmg::TransitionMetricType::kDynamicsWindow;
        config.good_transition_threshold = 1.0e9f;
        config.bad_transition_threshold = 2.0e9f;
        config.transition_metric.root_motion_weight = 0.0f;
        const pmg::PmgEdge first =
            pmg::PmgBuilder::BuildEdge(skeleton, 0, 0, space, space, config);
        const pmg::PmgEdge second =
            pmg::PmgBuilder::BuildEdge(skeleton, 0, 0, space, space, config);
        assert(!first.samples.empty());
        assert(first.samples.size() == second.samples.size());
        for (std::size_t i = 0; i < first.samples.size(); ++i) {
            assert(SamplesEqual(first.samples[i], second.samples[i]));
        }
    }

    // Disconnected spaces (no good transition) yield an empty edge and report
    // the no-GOOD-target failure.
    {
        pmg::MotionClip far_clip = MakeLoopClip(0.0f, 'X', 20.0f);
        for (pmg::Pose& pose : far_clip.frames) {
            pose.root_position.y += 100.0f;
        }
        pmg::ParametricMotionSpace other_space("far", 1);
        other_space.AddExample({0.0f}, far_clip);
        pmg::PmgBuilderConfig config = SmallConfig();
        config.good_transition_threshold = 1.0e-6f;
        config.bad_transition_threshold = 1.0e-3f;
        const pmg::EdgeBuildResult result =
            pmg::PmgBuilder::BuildEdgeWithReport(skeleton, 0, 1, space, other_space, config);
        assert(result.edge.samples.empty());
        assert(!result.report.edge_created);
        assert(!result.report.source_reports.empty());
        assert(result.report.source_reports.front().good_count == 0);
    }

    // D1 (paper Sec 3.2 / Sec 6): an edge exists only if EVERY source sample can
    // reach the target. At one fixed threshold, a source whose extreme motions
    // cannot reach the target yields NO edge, while a narrow source whose every
    // sample can reach it yields an edge. (These synthetic clips use unit-scale
    // offsets, so distances are ~O(0.3) at the extremes -- much smaller than the
    // x100 BVH scale -- hence the small threshold here.)
    {
        // Narrow target: a single ~static example (param baked to 0 => no swing).
        pmg::ParametricMotionSpace static_target("target", 1);
        static_target.AddExample({0.0f}, MakeLoopClip(0.0f, 'X', 90.0f));

        // dist(source@p, static) grows with |p|; pick a threshold between the
        // narrow extreme (~0) and the wide extreme (~0.3).
        pmg::PmgBuilderConfig config = SmallConfig();
        config.good_transition_threshold = 0.05f;
        config.bad_transition_threshold = 20.0f;

        // Wide source: large swing at |p|~1 cannot reach the static target.
        pmg::ParametricMotionSpace wide_source("wide", 1);
        wide_source.AddExample({-1.0f}, MakeLoopClip(-1.0f, 'X', 90.0f));
        wide_source.AddExample({0.0f}, MakeLoopClip(0.0f, 'X', 90.0f));
        wide_source.AddExample({1.0f}, MakeLoopClip(1.0f, 'X', 90.0f));
        const pmg::PmgEdge rejected =
            pmg::PmgBuilder::BuildEdge(skeleton, 0, 0, wide_source, static_target, config);
        assert(rejected.samples.empty());  // some source sample cannot reach -> no edge

        // Narrow source: every sample (|p|<=0.1) is near-static -> all reach ->
        // edge IS built (confirms the rejection is the empty-box rule, not just a
        // globally disconnected pair).
        pmg::ParametricMotionSpace narrow_source("narrow", 1);
        narrow_source.AddExample({-0.1f}, MakeLoopClip(-0.1f, 'X', 90.0f));
        narrow_source.AddExample({0.0f}, MakeLoopClip(0.0f, 'X', 90.0f));
        narrow_source.AddExample({0.1f}, MakeLoopClip(0.1f, 'X', 90.0f));
        const pmg::PmgEdge built =
            pmg::PmgBuilder::BuildEdge(skeleton, 0, 0, narrow_source, static_target, config);
        assert(!built.samples.empty());
    }
    {
        // PmgBuilder_SamplesInsideTriangulatedSupport
        pmg::ParametricMotionSpace triangulated_source("walk", 2);
        triangulated_source.AddExample({-0.3f, 0.0f}, MakeLoopClip(0.0f, 'X', 90.0f));
        triangulated_source.AddExample({0.0f, 0.0f}, MakeLoopClip(0.0f, 'X', 90.0f));
        triangulated_source.AddExample({1.0f, 0.0f}, MakeLoopClip(0.0f, 'X', 90.0f));
        triangulated_source.AddExample({0.0f, 1.0f}, MakeLoopClip(0.0f, 'X', 90.0f));
        triangulated_source.AddExample({0.15f, 0.75f}, MakeLoopClip(0.0f, 'X', 90.0f));
        std::vector<pmg::ParameterVector> vertices = {
            {-0.3f, 0.0f}, {0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}, {0.15f, 0.75f}
        };
        std::vector<std::array<int, 3>> triangles = {{0, 1, 4}, {1, 2, 4}, {2, 3, 4}, {3, 0, 4}};
        const pmg::ParameterSupport support = pmg::ParameterSupport::CreateTriangulated2D(vertices, triangles);
        triangulated_source.SetParameterSupport(support);

        pmg::PmgBuilderConfig config;
        config.source_sample_count = 50;
        config.target_sample_count = 1;
        config.generated_frame_count = 1;
        config.good_transition_threshold = 1.0e9f;
        config.bad_transition_threshold = 2.0e9f;

        const pmg::PmgEdge built =
            pmg::PmgBuilder::BuildEdge(skeleton, 0, 0, triangulated_source, triangulated_source, config);
        
        assert(!built.samples.empty());
        for (const pmg::TransitionSample& sample : built.samples) {
            assert(support.Contains(sample.source_parameter));
        }
    }

    return 0;
}
