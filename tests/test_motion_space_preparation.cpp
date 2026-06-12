#include "pmg/GraphSpec.h"
#include "pmg/MotionSpacePreparation.h"

#include <cassert>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace {

void WriteMinimalBvh(const std::filesystem::path& path, float end_x) {
    std::ofstream output(path);
    output << "HIERARCHY\n"
           << "ROOT Hips\n"
           << "{\n"
           << "  OFFSET 0 0 0\n"
           << "  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation "
              "Yrotation\n"
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
        std::filesystem::temp_directory_path() / "pmg_motion_space_preparation_test";
    std::filesystem::create_directories(directory);
    const std::filesystem::path bvh_a = directory / "walk_a.bvh";
    const std::filesystem::path bvh_b = directory / "walk_b.bvh";
    WriteMinimalBvh(bvh_a, 1.0f);
    WriteMinimalBvh(bvh_b, 2.0f);

    pmg::GraphSpec spec;
    pmg::GraphSpecNode node;
    node.name = "walk";
    node.parameter_dimension = 1;
    node.has_registration_config = true;
    node.min_contact_frames = 3;
    node.dtw_refine = false;
    spec.nodes.push_back(node);
    spec.examples.push_back({"walk", {0.0f}, bvh_a.string()});
    spec.examples.push_back({"walk", {1.0f}, bvh_b.string()});

    pmg::MotionSpacePreparationConfig config;
    config.default_contact_joints.clear();
    config.default_dtw_refine = false;
    const pmg::PreparedMotionSpaces prepared = pmg::PrepareMotionSpaces(spec, config);

    assert(prepared.skeleton.NumJoints() > 0);
    assert(prepared.source_bvh_paths.size() == 2);
    const pmg::PreparedMotionSpace& walk = prepared.Node("walk");
    assert(walk.authored.NumExamples() == 2);
    assert(walk.production.NumExamples() == 2);
    assert(!walk.contact_registered.has_value());
    assert(!walk.dtw_refined.has_value());

    bool unknown_node_threw = false;
    try {
        (void)prepared.Node("missing");
    } catch (const std::runtime_error&) {
        unknown_node_threw = true;
    }
    assert(unknown_node_threw);

    pmg::ArtifactBuildConfig artifact_config;
    artifact_config.default_edge_config.generated_frames_per_second = 30.0f;
    artifact_config.default_contact_joints.clear();
    artifact_config.default_dtw_refine = false;
    const pmg::BuiltPmgArtifact artifact = pmg::BuildPmgArtifactFromSpec(spec, artifact_config);
    assert(artifact.graph.NumNodes() == 1);
    assert(artifact.graph.Node(0).motion_space.NumExamples() == walk.production.NumExamples());
    assert(artifact.metadata.source_bvh_paths == prepared.source_bvh_paths);
    assert(artifact.metadata.node_registrations.size() == 1);
    assert(artifact.metadata.node_registrations[0].node_name == "walk");

    std::filesystem::remove_all(directory);
    return 0;
}
