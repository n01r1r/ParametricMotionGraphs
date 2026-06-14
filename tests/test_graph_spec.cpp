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

    // edge_phase_range sets the distance-grid search sub-range and merges with
    // edge_config regardless of line order (phase_range first here).
    const std::filesystem::path phase_range_spec_path =
        directory / "phase_range.txt";
    {
        std::ofstream phase_range_spec(phase_range_spec_path);
        phase_range_spec
            << "node walk 1\n"
            << "registration walk - - 3 0\n"
            << "example walk 0 walk_a.bvh\n"
            << "example walk 1 walk_b.bvh\n"
            << "edge walk walk\n"
            << "edge_phase_range walk walk 0.6 0.9 0.1 0.4\n"
            << "edge_config walk walk 10 20 2 3 99\n";
    }
    const pmg::GraphSpec phase_range_parsed =
        pmg::LoadGraphSpec(phase_range_spec_path.string());
    assert(phase_range_parsed.edges[0].has_build_config);
    assert(phase_range_parsed.edges[0].has_phase_range);
    assert(phase_range_parsed.edges[0].has_threshold_config);
    const pmg::DistanceGridConfig& phase_range_grid =
        phase_range_parsed.edges[0].build_config.distance_grid;
    assert(phase_range_grid.source_phase_start == 0.6f);
    assert(phase_range_grid.source_phase_end == 0.9f);
    assert(phase_range_grid.target_phase_start == 0.1f);
    assert(phase_range_grid.target_phase_end == 0.4f);
    // The later edge_config merged its fields without clobbering the range.
    assert(phase_range_parsed.edges[0].build_config.seed == 99);

    // Inverted range (start >= end) is rejected.
    const std::filesystem::path bad_range_spec_path = directory / "bad_range.txt";
    {
        std::ofstream bad_range_spec(bad_range_spec_path);
        bad_range_spec << "node walk 1\n"
                       << "example walk 0 walk_a.bvh\n"
                       << "edge walk walk\n"
                       << "edge_phase_range walk walk 0.9 0.6 0.1 0.4\n";
    }
    bool bad_range_threw = false;
    try {
        (void)pmg::LoadGraphSpec(bad_range_spec_path.string());
    } catch (const std::runtime_error&) {
        bad_range_threw = true;
    }
    assert(bad_range_threw);

    // expect clauses are validated against the parsed examples at load time.
    // A 2-D node sampling all four corners satisfies corner_coverage full.
    const std::filesystem::path full_corner_spec_path =
        directory / "full_corner.txt";
    {
        std::ofstream full_corner_spec(full_corner_spec_path);
        full_corner_spec << "node g 2\n"
                         << "example g 0 0 walk_a.bvh\n"
                         << "example g 1 0 walk_a.bvh\n"
                         << "example g 0 1 walk_a.bvh\n"
                         << "example g 1 1 walk_a.bvh\n"
                         << "expect g spanned_axes 2\n"
                         << "expect g corner_coverage full\n";
    }
    const pmg::GraphSpec full_corner =
        pmg::LoadGraphSpec(full_corner_spec_path.string());
    assert(full_corner.nodes[0].expect_corner_coverage_full);
    assert(full_corner.nodes[0].has_expect_spanned_axes);
    assert(full_corner.nodes[0].expect_spanned_axes == 2);

    // The same node missing the (1,1) corner is a degenerate simplex and fails
    // the corner_coverage full claim.
    const std::filesystem::path missing_corner_spec_path =
        directory / "missing_corner.txt";
    {
        std::ofstream missing_corner_spec(missing_corner_spec_path);
        missing_corner_spec << "node g 2\n"
                            << "example g 0 0 walk_a.bvh\n"
                            << "example g 1 0 walk_a.bvh\n"
                            << "example g 0 1 walk_a.bvh\n"
                            << "expect g corner_coverage full\n";
    }
    bool missing_corner_threw = false;
    try {
        (void)pmg::LoadGraphSpec(missing_corner_spec_path.string());
    } catch (const std::runtime_error&) {
        missing_corner_threw = true;
    }
    assert(missing_corner_threw);

    // A node whose examples vary along only one axis cannot satisfy
    // spanned_axes 2.
    const std::filesystem::path collinear_spec_path =
        directory / "collinear.txt";
    {
        std::ofstream collinear_spec(collinear_spec_path);
        collinear_spec << "node g 2\n"
                       << "example g 0 0 walk_a.bvh\n"
                       << "example g 1 0 walk_a.bvh\n"
                       << "expect g spanned_axes 2\n";
    }
    bool collinear_threw = false;
    try {
        (void)pmg::LoadGraphSpec(collinear_spec_path.string());
    } catch (const std::runtime_error&) {
        collinear_threw = true;
    }
    assert(collinear_threw);

    pmg::PmgBuilderConfig config;
    config.source_sample_count = 1;
    config.target_sample_count = 1;
    config.generated_frame_count = 8;
    config.good_transition_threshold = 1.0e9f;
    config.bad_transition_threshold = 2.0e9f;

    // One build path defines what a spec means: BuildPmgArtifactFromSpec honors
    // each edge's parsed edge_config (seed 99 and the spec's sample counts) and
    // the graph is read back from the artifact. The former graph-only
    // BuildGraphFromSpec shim ignored per-edge config and built a divergent
    // graph from the same spec; it was removed so tests validate the graph
    // production actually ships.
    pmg::ArtifactBuildConfig artifact_config;
    artifact_config.default_edge_config = config;
    artifact_config.default_contact_joints.clear();
    artifact_config.default_dtw_refine = false;
    const pmg::BuiltPmgArtifact artifact =
        pmg::BuildPmgArtifactFromSpec(parsed, artifact_config);
    assert(artifact.skeleton.NumJoints() > 0);
    assert(artifact.graph.NumNodes() == 1);
    assert(artifact.graph.NumEdges() == 1);
    assert(artifact.graph.Node(0).motion_space.NumExamples() == 2);
    assert(!artifact.graph.Edge(0).samples.empty());
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
