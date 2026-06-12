#include "pmg/GraphSpec.h"

#include <cassert>
#include <filesystem>
#include <fstream>

namespace {

void WriteMinimalBvh(const std::filesystem::path& path, float end_x) {
    std::ofstream output(path);
    output
        << "HIERARCHY\n"
        << "ROOT Hips\n"
        << "{\n"
        << "  OFFSET 0 0 0\n"
        << "  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n"
        << "  JOINT Chest\n"
        << "  {\n"
        << "    OFFSET 0 1 0\n"
        << "    CHANNELS 3 Zrotation Xrotation Yrotation\n"
        << "    End Site ChestTip\n"
        << "    {\n"
        << "      OFFSET 0 1 0\n"
        << "    }\n"
        << "  }\n"
        << "}\n"
        << "MOTION\n"
        << "Frames: 4\n"
        << "Frame Time: 0.0333333\n"
        << "0 0 0 0 0 0 0 0 0\n"
        << end_x * 0.333333f << " 0 0 0 0 0 0 0 0\n"
        << end_x * 0.666667f << " 0 0 0 0 0 0 0 0\n"
        << end_x << " 0 0 0 0 0 0 0 0\n";
}

}  // namespace

int main() {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "pmg_graph_spec_test";
    std::filesystem::create_directories(directory);
    const std::filesystem::path bvh_a = directory / "walk_a.bvh";
    const std::filesystem::path bvh_b = directory / "walk_b.bvh";
    const std::filesystem::path spec_path = directory / "graph.txt";
    WriteMinimalBvh(bvh_a, 1.0f);
    WriteMinimalBvh(bvh_b, 2.0f);

    std::ofstream spec(spec_path);
    spec << "# one-node self graph\n"
         << "node walk 1\n"
         << "registration walk - - 3 0\n"
         << "example walk 0 walk_a.bvh\n"
         << "example walk 1 walk_b.bvh\n"
         << "edge walk walk\n"
         << "edge_config walk walk 10 20 2 3 99\n";
    spec.close();

    const pmg::GraphSpec parsed = pmg::LoadGraphSpec(spec_path.string());
    assert(parsed.nodes.size() == 1);
    assert(parsed.examples.size() == 2);
    assert(parsed.edges.size() == 1);
    assert(parsed.nodes[0].has_registration_config);
    assert(parsed.nodes[0].contact_joints.empty());
    assert(parsed.edges[0].has_build_config);
    assert(parsed.edges[0].build_config.seed == 99);

    const std::filesystem::path multidimensional_spec_path =
        directory / "multidimensional.txt";
    {
        std::ofstream multidimensional_spec(multidimensional_spec_path);
        multidimensional_spec
            << "node locomotion 2\n"
            << "parameter_metrics locomotion turn_rate travel_speed\n"
            << "parameter_calibration locomotion 7\n";
    }
    const pmg::GraphSpec multidimensional =
        pmg::LoadGraphSpec(multidimensional_spec_path.string());
    assert(multidimensional.nodes[0].parameter_metrics ==
           (std::vector<pmg::ParameterMetric>{
               pmg::ParameterMetric::kTurnRate,
               pmg::ParameterMetric::kTravelSpeed}));
    assert(multidimensional.nodes[0].calibration_samples_per_axis == 7);

    const std::filesystem::path invalid_metrics_spec_path =
        directory / "invalid_metrics.txt";
    {
        std::ofstream invalid_metrics_spec(invalid_metrics_spec_path);
        invalid_metrics_spec
            << "node locomotion 2\n"
            << "parameter_metrics locomotion turn_rate\n";
    }
    bool invalid_metric_count_threw = false;
    try {
        (void)pmg::LoadGraphSpec(invalid_metrics_spec_path.string());
    } catch (const std::runtime_error&) {
        invalid_metric_count_threw = true;
    }
    assert(invalid_metric_count_threw);

    pmg::PmgBuilderConfig config;
    config.source_sample_count = 1;
    config.target_sample_count = 1;
    config.generated_frame_count = 8;
    config.good_transition_threshold = 1.0e9f;
    config.bad_transition_threshold = 2.0e9f;

    const pmg::ParametricMotionGraph graph = pmg::BuildGraphFromSpec(parsed, config);
    assert(graph.NumNodes() == 1);
    assert(graph.NumEdges() == 1);
    assert(graph.Node(0).motion_space.NumExamples() == 2);
    assert(!graph.Edge(0).samples.empty());

    pmg::ArtifactBuildConfig artifact_config;
    artifact_config.default_edge_config = config;
    artifact_config.default_contact_joints.clear();
    artifact_config.default_dtw_refine = false;
    const pmg::BuiltPmgArtifact artifact =
        pmg::BuildPmgArtifactFromSpec(parsed, artifact_config);
    assert(artifact.skeleton.NumJoints() > 0);
    assert(artifact.graph.NumEdges() == 1);
    assert(artifact.metadata.edge_builds[0].config.seed == 99);

    const std::filesystem::path invalid_spec_path =
        directory / "invalid_graph.txt";
    std::ofstream invalid_spec(invalid_spec_path);
    invalid_spec << "node walk 1\n"
                 << "registration walk MissingJoint MissingJoint 3 0\n"
                 << "example walk 0 walk_a.bvh\n"
                 << "edge walk walk\n";
    invalid_spec.close();
    bool invalid_joint_threw = false;
    try {
        const pmg::GraphSpec invalid =
            pmg::LoadGraphSpec(invalid_spec_path.string());
        (void)pmg::BuildPmgArtifactFromSpec(invalid, artifact_config);
    } catch (const std::runtime_error&) {
        invalid_joint_threw = true;
    }
    assert(invalid_joint_threw);

    std::filesystem::remove_all(directory);
    return 0;
}
