#include "pmg/PmgOfflinePipeline.h"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {

void WriteMinimalBvh(const std::filesystem::path& path, float end_x) {
    std::ofstream output(path);
    output
        << "HIERARCHY\n"
        << "ROOT Hips\n"
        << "{\n"
        << "  OFFSET 0 0 0\n"
        << "  CHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation\n"
        << "  JOINT LeftAnkle\n"
        << "  {\n"
        << "    OFFSET 0 -1 0\n"
        << "    CHANNELS 3 Xrotation Yrotation Zrotation\n"
        << "    End Site LeftToe\n"
        << "    {\n"
        << "      OFFSET 0 -0.2 0\n"
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

pmg::GraphSpec MakeSpec(const std::filesystem::path& bvh_a,
                        const std::filesystem::path& bvh_b) {
    pmg::GraphSpec spec;
    pmg::GraphSpecNode node;
    node.name = "walk";
    node.parameter_dimension = 1;
    node.has_registration_config = true;
    node.contact_joints.clear();
    node.dtw_refine = false;
    spec.nodes.push_back(node);
    spec.examples.push_back(
        {"walk", {0.0f}, bvh_a.string(), pmg::FullClipSegment(bvh_a.string())});
    spec.examples.push_back(
        {"walk", {1.0f}, bvh_b.string(), pmg::FullClipSegment(bvh_b.string())});
    spec.edges.push_back({"walk", "walk"});
    return spec;
}

pmg::ArtifactBuildConfig MakeConfig(int seed) {
    pmg::ArtifactBuildConfig config;
    config.default_contact_joints.clear();
    config.default_dtw_refine = false;
    config.default_edge_config.source_sample_count = 1;
    config.default_edge_config.target_sample_count = 1;
    config.default_edge_config.generated_frame_count = 8;
    config.default_edge_config.good_transition_threshold = 1.0e9f;
    config.default_edge_config.bad_transition_threshold = 2.0e9f;
    config.default_edge_config.seed = seed;
    return config;
}

}  // namespace

int main() {
    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() / "pmg_offline_pipeline_test";
    std::filesystem::create_directories(directory);
    const std::filesystem::path bvh_a = directory / "walk_a.bvh";
    const std::filesystem::path bvh_b = directory / "walk_b.bvh";
    WriteMinimalBvh(bvh_a, 1.0f);
    WriteMinimalBvh(bvh_b, 2.0f);

    const pmg::GraphSpec spec = MakeSpec(bvh_a, bvh_b);
    const pmg::ArtifactBuildConfig config = MakeConfig(77);

    const pmg::BuiltPmgArtifact first =
        pmg::BuildPmgOfflinePipeline(spec, config);
    const pmg::BuiltPmgArtifact second =
        pmg::BuildPmgOfflinePipeline(spec, config);

    // PmgOfflinePipeline_SameSeedSameArtifactMetadata
    assert(first.metadata.source_bvh_paths == second.metadata.source_bvh_paths);
    assert(first.metadata.edge_builds.size() == second.metadata.edge_builds.size());
    assert(first.metadata.edge_builds[0].config.seed == 77);
    assert(second.metadata.edge_builds[0].config.seed == 77);

    // PmgOfflinePipeline_PreservesNodeExampleEdgeMetadata
    assert(first.graph.NumNodes() == 1);
    assert(first.graph.Node(0).name == "walk");
    assert(first.graph.Node(0).motion_space.NumExamples() == 2);
    assert(first.metadata.edge_builds[0].source_node == "walk");
    assert(first.metadata.edge_builds[0].target_node == "walk");

    // PmgOfflinePipeline_RegistrationContract
    assert(first.metadata.node_registrations.size() == 1);
    assert(first.metadata.node_registrations[0].node_name == "walk");
    assert(first.metadata.node_registrations[0].contact_joints.empty());
    assert(!first.metadata.node_registrations[0].dtw_refine);

    // PmgOfflinePipeline_SupportContract
    assert(first.graph.Node(0).motion_space.ParameterDimension() == 1);

    // PmgOfflinePipeline_AllEdgesRejectedProducesExpectedFailure
    pmg::ArtifactBuildConfig rejecting_config = MakeConfig(77);
    rejecting_config.default_edge_config.good_transition_threshold = -1.0f;
    rejecting_config.default_edge_config.bad_transition_threshold = 0.0f;
    bool rejected_threw = false;
    try {
        (void)pmg::BuildPmgOfflinePipeline(spec, rejecting_config);
    } catch (const std::runtime_error& error) {
        rejected_threw =
            std::string(error.what()).find("every declared edge was rejected") !=
            std::string::npos;
    }
    assert(rejected_threw);

    std::filesystem::remove_all(directory);
    return 0;
}
