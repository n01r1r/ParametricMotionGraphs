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
         << "example walk 0 walk_a.bvh\n"
         << "example walk 1 walk_b.bvh\n"
         << "edge walk walk\n";
    spec.close();

    const pmg::GraphSpec parsed = pmg::LoadGraphSpec(spec_path.string());
    assert(parsed.nodes.size() == 1);
    assert(parsed.examples.size() == 2);
    assert(parsed.edges.size() == 1);

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

    std::filesystem::remove_all(directory);
    return 0;
}
