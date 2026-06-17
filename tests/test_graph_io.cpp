#include "pmg/GraphIo.h"
#include "pmg/RuntimeController.h"
#include "pmg/AlignmentStrategy.h"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace {

pmg::MotionClip MakeClip(float root_x) {
    pmg::MotionClip clip;
    clip.name = "clip with spaces";
    clip.frames_per_second = 30.0f;
    for (int frame = 0; frame < 3; ++frame) {
        pmg::Pose pose;
        pose.root_position = {
            root_x + static_cast<float>(frame), 0.0f, 0.2f * frame};
        pose.local_rotations.push_back(pmg::Quaternion::Identity());
        clip.frames.push_back(pose);
    }
    return clip;
}

pmg::ParametricMotionGraph MakeGraph(bool registered) {
    pmg::ParametricMotionSpace space("walk", 2);
    space.AddExample({0.0f, 0.0f}, MakeClip(0.0f));
    space.AddExample({1.0f, 0.0f}, MakeClip(10.0f));
    space.AddExample({0.0f, 1.0f}, MakeClip(20.0f));
    if (registered) {
        space.SetExampleTimeWarps({
            pmg::TimeWarp::FromAnchors({0.5f}, {0.35f}),
            pmg::TimeWarp::FromAnchors({0.5f}, {0.65f}),
            pmg::TimeWarp::FromAnchors({0.5f}, {0.55f}),
        });
        pmg::ParameterCalibration calibration;
        calibration.metrics = {
            pmg::ParameterMetric::kTurnRate,
            pmg::ParameterMetric::kTravelSpeed,
        };
        calibration.example_measured = {
            {-0.5f, 1.0f},
            {0.5f, 1.0f},
            {0.0f, 2.0f},
        };
        calibration.metric_scales = {1.0f, 1.0f};
        calibration.samples_per_axis = 5;
        calibration.samples = {
            {{-0.5f, 1.0f}, {1.0f, 0.0f, 0.0f}},
            {{0.5f, 1.0f}, {0.0f, 1.0f, 0.0f}},
            {{0.0f, 2.0f}, {0.0f, 0.0f, 1.0f}},
            {{0.2f, 1.4f}, {0.2f, 0.4f, 0.4f}},
        };
        space.SetParameterCalibration(calibration);
        space.SetParameterSupport(pmg::ParameterSupport({{0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}}));
    }

    pmg::ParametricMotionGraph graph;
    const int node = graph.AddNode("walk", space);
    pmg::ParameterAabb box;
    box.min_corner = {0.0f, 0.0f};
    box.max_corner = {1.0f, 1.0f};
    pmg::PmgEdge edge;
    edge.source_node = node;
    edge.target_node = node;
    edge.samples.push_back({{0.0f, 0.0f}, box, 0.8f, 0.1f});
    edge.samples.back().transition_distance = 0.42f;
    edge.samples.back().target_phase_samples = {
        {{0.0f, 0.0f}, 0.75f, 0.05f},
        {{1.0f, 1.0f}, 0.85f, 0.15f},
    };
    graph.AddEdge(edge);
    return graph;
}

pmg::Skeleton MakeSkeleton() {
    pmg::Skeleton skeleton;
    pmg::Joint root;
    root.name = "Hips";
    root.parent_index = -1;
    root.channels = {
        pmg::BvhChannelType::XPosition,
        pmg::BvhChannelType::YPosition,
        pmg::BvhChannelType::ZPosition,
        pmg::BvhChannelType::YRotation,
    };
    skeleton.joints.push_back(root);
    return skeleton;
}

pmg::BuiltPmgArtifact MakeArtifact() {
    pmg::BuiltPmgArtifact artifact;
    artifact.skeleton = MakeSkeleton();
    artifact.graph = MakeGraph(true);
    artifact.metadata.units = "BVH native units";
    artifact.metadata.generated_frame_count = 3;
    artifact.metadata.frames_per_second = 30.0f;
    artifact.metadata.source_bvh_paths = {"a path/walk_a.bvh", "walk_b.bvh"};
    artifact.metadata.node_registrations.push_back(
        {"walk", "LeftAnkle", {"LeftAnkle", "RightAnkle"}, 3, true});

    pmg::EdgeBuildMetadata edge;
    edge.source_node = "walk";
    edge.target_node = "walk";
    edge.config.source_sample_count = 7;
    edge.config.target_sample_count = 11;
    edge.config.seed = 42;
    edge.config.transition_metric_type = pmg::TransitionMetricType::kDynamicsWindow;
    edge.config.transition_metric.position_weight = 2.0f;
    edge.config.transition_metric.velocity_weight = 0.25f;
    edge.config.transition_metric.acceleration_weight = 0.05f;
    edge.config.transition_metric.root_motion_weight = 0.75f;
    edge.config.transition_metric.foot_contact_weight = 3.0f;
    edge.config.transition_metric.position_scale = 100.0f;
    edge.config.transition_metric.velocity_scale = 200.0f;
    edge.config.transition_metric.acceleration_scale = 300.0f;
    edge.config.transition_metric.root_speed_scale = 40.0f;
    edge.config.transition_metric.yaw_rate_scale = 5.0f;
    edge.config.transition_metric.foot_mismatch_penalty = 7.0f;
    edge.config.transition_metric.per_joint_weights = {1.0f};
    edge.config.transition_metric.contact_joint_indices = {0};
    edge.config.transition_metric.contact_settings =
        pmg::ContactDetectionSettings{0.5f, 2.5f, 1};
    edge.report.edge_created = true;
    pmg::SourceSampleBuildReport report;
    report.source_parameter = {0.25f, 0.25f};
    report.good_count = 8;
    report.neutral_count = 2;
    report.bad_count = 1;
    report.min_distance = 0.2f;
    report.p25_distance = 0.3f;
    report.median_distance = 0.4f;
    report.max_distance = 0.9f;
    report.target_box_before_shrink.min_corner = {0.0f, 0.0f};
    report.target_box_before_shrink.max_corner = {1.0f, 1.0f};
    report.target_box_after_shrink.min_corner = {0.0f, 0.0f};
    report.target_box_after_shrink.max_corner = {0.8f, 0.8f};
    report.accepted = true;
    edge.report.source_reports.push_back(report);
    artifact.metadata.edge_builds.push_back(edge);
    return artifact;
}

void CheckGraph(const pmg::ParametricMotionGraph& graph) {
    assert(graph.NumNodes() == 1);
    assert(graph.NumEdges() == 1);
    assert(graph.Node(0).name == "walk");
    assert(graph.Node(0).motion_space.NumExamples() == 3);
    assert(graph.Edge(0).samples.size() == 1);
}

void WriteLegacyFixture(
    const std::filesystem::path& path, const std::string& version) {
    std::ofstream output(path);
    output << version << "\n"
           << "nodes 1\n"
           << "node walk 1 1\n"
           << "example 1 0\n"
           << "clip clip 30 2\n"
           << "frame 0 0 0 1 1 0 0 0\n"
           << "frame 1 0 0 1 1 0 0 0\n";
    if (version == "PMG_GRAPH_V3") {
        output << "warps 0\n";
    }
    output << "edges 1\n"
           << "edge 0 0 1\n"
           << "sample 1 0 1 0 1 1 0.8 0.1\n";
}

// A V4 artifact (pre-calibration format): loads with an intact Skeleton but
// without parameter calibration.
void WriteV4Fixture(const std::filesystem::path& path) {
    std::ofstream output(path);
    output << "PMG_GRAPH_V4\n"
           << "runtime 3 30 \"BVH native units\"\n"
           << "sources 0\n"
           << "registrations 0\n"
           << "edge_builds 0\n"
           << "skeleton 1\n"
           << "joint \"Hips\" -1 0 0 0 0\n"
           << "nodes 1\n"
           << "node \"walk\" 1 1\n"
           << "example 1 0\n"
           << "clip \"clip\" 30 2\n"
           << "frame 0 0 0 1 1 0 0 0\n"
           << "frame 1 0 0 1 1 0 0 0\n"
           << "warps 0\n"
           << "edges 1\n"
           << "edge 0 0 1\n"
           << "sample 1 0 1 0 1 1 0.8 0.1\n";
}

void WriteV5Fixture(const std::filesystem::path& path) {
    std::ofstream output(path);
    output << "PMG_GRAPH_V5\n"
           << "runtime 3 30 \"BVH native units\"\n"
           << "sources 0\n"
           << "registrations 0\n"
           << "edge_builds 0\n"
           << "skeleton 1\n"
           << "joint \"Hips\" -1 0 0 0 0\n"
           << "nodes 1\n"
           << "node \"walk\" 1 1\n"
           << "example 1 0\n"
           << "clip \"clip\" 30 2\n"
           << "frame 0 0 0 1 1 0 0 0\n"
           << "frame 1 0 0 1 1 0 0 0\n"
           << "warps 0\n"
           << "calibration 0\n"
           << "edges 1\n"
           << "edge 0 0 1\n"
           << "sample 1 0 1 0 1 1 0.8 0.1\n";
}

void WriteV6CalibrationFixture(const std::filesystem::path& path) {
    std::ofstream output(path);
    output << "PMG_GRAPH_V6\n"
           << "runtime 3 30 \"BVH native units\"\n"
           << "sources 0\n"
           << "registrations 0\n"
           << "edge_builds 0\n"
           << "skeleton 1\n"
           << "joint \"Hips\" -1 0 0 0 0\n"
           << "nodes 1\n"
           << "node \"walk\" 1 2\n"
           << "example 1 0\n"
           << "clip \"left\" 30 2\n"
           << "frame 0 0 0 1 1 0 0 0\n"
           << "frame 1 0 0 1 1 0 0 0\n"
           << "example 1 1\n"
           << "clip \"right\" 30 2\n"
           << "frame 10 0 0 1 1 0 0 0\n"
           << "frame 11 0 0 1 1 0 0 0\n"
           << "warps 0\n"
           << "calibration 1 2 0 1 -0.5 0.5 1\n"
           << "segment 0 1 3 0 -0.5 0.5 0.2 1 0.5\n"
           << "edges 0\n";
}

void WriteV9MetriclessFixture(const std::filesystem::path& path) {
    std::ofstream output(path);
    output << "PMG_GRAPH_V9\n"
           << "runtime 3 30 \"BVH native units\"\n"
           << "sources 0\n"
           << "registrations 0\n"
           << "edge_builds 1\n"
           << "edge_build \"walk\" \"walk\" "
           << "7 11 3 30 80 110 42 0.0001 1 "
           << "5 2 2 0.7 0.95 0.05 0.3 0 "
           << "1 \"\" 0\n"
           << "skeleton 1\n"
           << "joint \"Hips\" -1 0 0 0 0\n"
           << "nodes 1\n"
           << "node \"walk\" 1 1\n"
           << "example 1 0\n"
           << "clip \"clip\" 30 2\n"
           << "frame 0 0 0 1 1 0 0 0\n"
           << "frame 1 0 0 1 1 0 0 0\n"
           << "warps 0\n"
           << "calibration 0 0\n"
           << "edges 1\n"
           << "edge 0 0 1\n"
           << "sample 1 0 1 0 1 1 0.8 0.1 12.5 0\n";
}

}  // namespace

int main() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "pmg_graph_io_test.pmg";

    const pmg::BuiltPmgArtifact original = MakeArtifact();
    {
        const pmg::RuntimeControllerConfig runtime_config =
            pmg::RuntimeControllerConfigFromArtifact(original);
        assert(runtime_config.transition_blend_frames ==
               original.metadata.edge_builds.front()
                   .config.distance_grid.window_size);

        pmg::BuiltPmgArtifact inconsistent = original;
        pmg::EdgeBuildMetadata second_edge =
            inconsistent.metadata.edge_builds.front();
        ++second_edge.config.distance_grid.window_size;
        inconsistent.metadata.edge_builds.push_back(second_edge);
        bool rejected_inconsistent_windows = false;
        try {
            (void)pmg::RuntimeControllerConfigFromArtifact(inconsistent);
        } catch (const std::runtime_error&) {
            rejected_inconsistent_windows = true;
        }
        assert(rejected_inconsistent_windows);
    }
    pmg::SavePmgArtifactText(original, path.string());
    {
        std::ifstream input(path);
        std::string header;
        input >> header;
        assert(header == "PMG_GRAPH_V11");
    }
    const pmg::BuiltPmgArtifact loaded =
        pmg::LoadPmgArtifactText(path.string());
    CheckGraph(loaded.graph);
    assert(loaded.skeleton.NumJoints() == 1);
    assert(loaded.skeleton.joints[0].name == "Hips");
    assert(loaded.metadata.generated_frame_count == 3);
    assert(loaded.metadata.source_bvh_paths[0] == "a path/walk_a.bvh");
    assert(loaded.metadata.node_registrations[0].dtw_refine);
    assert(loaded.metadata.edge_builds[0].config.seed == 42);
    assert(loaded.metadata.edge_builds[0].config.transition_metric_type ==
           pmg::TransitionMetricType::kDynamicsWindow);
    assert(std::abs(
               loaded.metadata.edge_builds[0]
                   .config.transition_metric.position_weight -
               2.0f) < 1.0e-6f);
    assert(std::abs(
               loaded.metadata.edge_builds[0]
                   .config.transition_metric.foot_mismatch_penalty -
               7.0f) < 1.0e-6f);
    assert(loaded.metadata.edge_builds[0]
               .config.transition_metric.contact_settings.has_value());
    assert(loaded.metadata.edge_builds[0]
               .config.transition_metric.contact_joint_indices[0] == 0);
    assert(loaded.metadata.edge_builds[0].report.source_reports[0].bad_count == 1);
    assert(loaded.graph.Node(0).motion_space.HasExampleTimeWarps());
    assert(loaded.graph.Edge(0).samples[0].target_phase_samples.size() == 2);
    assert(std::abs(
               loaded.graph.Edge(0).samples[0].transition_distance - 0.42f) <
           1.0e-6f);
    assert(std::abs(
               loaded.graph.Edge(0)
                       .samples[0]
                       .target_phase_samples[1]
                       .target_transition_phase -
               0.15f) < 1.0e-6f);

    // Parameter calibration round-trips with the space.
    {
        const pmg::ParameterCalibration& calibration =
            loaded.graph.Node(0).motion_space.ParameterCalibrationData();
        assert(calibration.metrics ==
               (std::vector<pmg::ParameterMetric>{
                   pmg::ParameterMetric::kTurnRate,
                   pmg::ParameterMetric::kTravelSpeed}));
        assert(calibration.example_measured.size() == 3);
        assert(calibration.samples.size() == 4);
        assert(calibration.samples_per_axis == 5);
        assert(std::abs(calibration.samples[3].measured_parameter[0] - 0.2f) <
               1.0e-6f);
        assert(std::abs(calibration.samples[3].blend_weights[1] - 0.4f) <
               1.0e-6f);
    }

    assert(loaded.graph.Node(0).motion_space.HasExplicitParameterSupport());
    assert(std::abs(loaded.graph.Node(0).motion_space.ProjectToSupport({1.0f, 1.0f})[0] - 0.5f) < 1e-4f);

    const pmg::ParametricMotionSpace& original_space =
        original.graph.Node(0).motion_space;
    const pmg::ParametricMotionSpace& loaded_space =
        loaded.graph.Node(0).motion_space;
    for (const float phase : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) {
        const pmg::Pose expected =
            original_space.EvaluatePose({0.5f, 0.25f}, phase);
        const pmg::Pose actual =
            loaded_space.EvaluatePose({0.5f, 0.25f}, phase);
        assert(std::abs(expected.root_position.x - actual.root_position.x) <
               1.0e-5f);
    }

    pmg::PointCloudAlignment original_alignment(original.skeleton);
    pmg::PointCloudAlignment loaded_alignment(loaded.skeleton);
    pmg::RuntimeController original_runtime(
        original.graph, original_alignment);
    pmg::RuntimeController loaded_runtime(loaded.graph, loaded_alignment);
    original_runtime.Start(0, {0.5f, 0.25f}, 30.0f);
    loaded_runtime.Start(0, {0.5f, 0.25f}, 30.0f);
    pmg::RuntimeControlRequest runtime_request;
    runtime_request.desired_node = 0;
    runtime_request.desired_parameter = {0.5f, 0.25f};
    for (int frame = 0; frame < 60; ++frame) {
        original_runtime.Update(1.0f / 30.0f, runtime_request);
        loaded_runtime.Update(1.0f / 30.0f, runtime_request);
        const pmg::Pose expected = original_runtime.CurrentPose();
        const pmg::Pose actual = loaded_runtime.CurrentPose();
        assert(std::abs(
                   expected.root_position.x - actual.root_position.x) <
               1.0e-5f);
        assert(std::abs(
                   expected.root_position.z - actual.root_position.z) <
               1.0e-5f);
    }

    for (const std::string version : {"PMG_GRAPH_V2", "PMG_GRAPH_V3"}) {
        WriteLegacyFixture(path, version);
        const pmg::BuiltPmgArtifact legacy =
            pmg::LoadPmgArtifactText(path.string());
        assert(legacy.skeleton.NumJoints() == 0);
        assert(legacy.graph.NumNodes() == 1);
        assert(legacy.graph.NumEdges() == 1);
    }

    {
        WriteV4Fixture(path);
        const pmg::BuiltPmgArtifact v4 = pmg::LoadPmgArtifactText(path.string());
        assert(v4.skeleton.NumJoints() == 1);
        assert(v4.graph.NumNodes() == 1);
        assert(!v4.graph.Node(0).motion_space.HasParameterCalibration());
    }

    {
        WriteV5Fixture(path);
        const pmg::BuiltPmgArtifact v5 = pmg::LoadPmgArtifactText(path.string());
        assert(v5.skeleton.NumJoints() == 1);
        assert(v5.graph.NumEdges() == 1);
        assert(v5.graph.Edge(0).samples[0].target_phase_samples.empty());
        const auto transition =
            v5.graph.Edge(0).LookupInterpolated({0.0f}, {0.7f});
        assert(transition.has_value());
        assert(std::abs(transition->source_transition_phase - 0.8f) <
               1.0e-6f);
        assert(std::abs(transition->target_transition_phase - 0.1f) <
               1.0e-6f);
    }

    {
        WriteV9MetriclessFixture(path);
        const pmg::BuiltPmgArtifact v9 =
            pmg::LoadPmgArtifactText(path.string());
        assert(v9.metadata.edge_builds[0].config.transition_metric_type ==
               pmg::TransitionMetricType::kKovarDirectionalPointCloud);
        assert(v9.metadata.edge_builds[0]
                   .config.transition_metric.contact_joint_indices.empty());
        assert(std::abs(v9.graph.Edge(0).samples[0].transition_distance -
                        12.5f) < 1.0e-6f);
    }

    {
        WriteV6CalibrationFixture(path);
        const pmg::BuiltPmgArtifact v6 =
            pmg::LoadPmgArtifactText(path.string());
        const pmg::ParametricMotionSpace& space =
            v6.graph.Node(0).motion_space;
        assert(space.HasParameterCalibration());
        const pmg::ParameterCalibration& calibration =
            space.ParameterCalibrationData();
        assert(calibration.metrics ==
               (std::vector<pmg::ParameterMetric>{
                   pmg::ParameterMetric::kTurnRate}));
        assert(calibration.samples.size() == 5);
        const std::vector<float> weights =
            space.ComputeLocalBlendWeights({0.5f});
        assert(std::abs(weights[1] - (0.5f * (0.5f / 0.7f))) <
               1.0e-5f);
    }

    std::filesystem::remove(path);
    return 0;
}
