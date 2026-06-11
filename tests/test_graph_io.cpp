#include "pmg/GraphIo.h"
#include "pmg/RuntimeController.h"
#include "pmg/AlignmentStrategy.h"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>

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
    pmg::ParametricMotionSpace space("walk", 1);
    space.AddExample({0.0f}, MakeClip(0.0f));
    space.AddExample({1.0f}, MakeClip(10.0f));
    if (registered) {
        space.SetExampleTimeWarps({
            pmg::TimeWarp::FromAnchors({0.5f}, {0.35f}),
            pmg::TimeWarp::FromAnchors({0.5f}, {0.65f}),
        });
        pmg::ParameterCalibration calibration;
        calibration.metric = pmg::ParameterMetric::kTurnRate;
        calibration.example_order = {0, 1};
        calibration.example_measured = {-0.5f, 0.5f};
        pmg::CalibrationSegment segment;
        segment.left_example = 0;
        segment.right_example = 1;
        segment.samples = {{0.0f, -0.5f}, {0.5f, 0.2f}, {1.0f, 0.5f}};
        calibration.segments.push_back(segment);
        space.SetParameterCalibration(calibration);
    }

    pmg::ParametricMotionGraph graph;
    const int node = graph.AddNode("walk", space);
    pmg::ParameterAabb box;
    box.min_corner = {0.0f};
    box.max_corner = {1.0f};
    pmg::PmgEdge edge;
    edge.source_node = node;
    edge.target_node = node;
    edge.samples.push_back({{0.0f}, box, 0.8f, 0.1f});
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
    edge.report.edge_created = true;
    pmg::SourceSampleBuildReport report;
    report.source_parameter = {0.25f};
    report.good_count = 8;
    report.neutral_count = 2;
    report.bad_count = 1;
    report.min_distance = 0.2f;
    report.p25_distance = 0.3f;
    report.median_distance = 0.4f;
    report.max_distance = 0.9f;
    report.target_box_before_shrink.min_corner = {0.0f};
    report.target_box_before_shrink.max_corner = {1.0f};
    report.target_box_after_shrink.min_corner = {0.0f};
    report.target_box_after_shrink.max_corner = {0.8f};
    report.accepted = true;
    edge.report.source_reports.push_back(report);
    artifact.metadata.edge_builds.push_back(edge);
    return artifact;
}

void CheckGraph(const pmg::ParametricMotionGraph& graph) {
    assert(graph.NumNodes() == 1);
    assert(graph.NumEdges() == 1);
    assert(graph.Node(0).name == "walk");
    assert(graph.Node(0).motion_space.NumExamples() == 2);
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

}  // namespace

int main() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "pmg_graph_io_test.pmg";

    const pmg::BuiltPmgArtifact original = MakeArtifact();
    pmg::SavePmgArtifactText(original, path.string());
    {
        std::ifstream input(path);
        std::string header;
        input >> header;
        assert(header == "PMG_GRAPH_V5");
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
    assert(loaded.metadata.edge_builds[0].report.source_reports[0].bad_count == 1);
    assert(loaded.graph.Node(0).motion_space.HasExampleTimeWarps());

    // Parameter calibration round-trips with the space.
    {
        const pmg::ParameterCalibration& calibration =
            loaded.graph.Node(0).motion_space.ParameterCalibrationData();
        assert(calibration.metric == pmg::ParameterMetric::kTurnRate);
        assert(calibration.example_order == (std::vector<int>{0, 1}));
        assert(calibration.segments.size() == 1);
        assert(calibration.segments[0].samples.size() == 3);
        assert(std::abs(calibration.segments[0].samples[1].blend_t - 0.5f) <
               1.0e-6f);
        assert(std::abs(calibration.segments[0].samples[1].measured - 0.2f) <
               1.0e-6f);
    }

    const pmg::ParametricMotionSpace& original_space =
        original.graph.Node(0).motion_space;
    const pmg::ParametricMotionSpace& loaded_space =
        loaded.graph.Node(0).motion_space;
    for (const float phase : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) {
        const pmg::Pose expected = original_space.EvaluatePose({0.5f}, phase);
        const pmg::Pose actual = loaded_space.EvaluatePose({0.5f}, phase);
        assert(std::abs(expected.root_position.x - actual.root_position.x) <
               1.0e-5f);
    }

    pmg::PointCloudAlignment original_alignment(original.skeleton);
    pmg::PointCloudAlignment loaded_alignment(loaded.skeleton);
    pmg::RuntimeController original_runtime(
        original.graph, original_alignment);
    pmg::RuntimeController loaded_runtime(loaded.graph, loaded_alignment);
    original_runtime.Start(0, {0.5f}, 30.0f);
    loaded_runtime.Start(0, {0.5f}, 30.0f);
    pmg::RuntimeControlRequest runtime_request;
    runtime_request.desired_node = 0;
    runtime_request.desired_parameter = {0.5f};
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

    std::filesystem::remove(path);
    return 0;
}
