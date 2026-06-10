#include "pmg/GraphIo.h"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>

namespace {

pmg::MotionClip MakeClip(float root_x) {
    pmg::MotionClip clip;
    clip.name = "clip";
    clip.frames_per_second = 30.0f;
    for (int frame = 0; frame < 2; ++frame) {
        pmg::Pose pose;
        pose.root_position = {root_x + static_cast<float>(frame), 0.0f, 0.0f};
        pose.local_rotations.push_back(pmg::Quaternion::Identity());
        clip.frames.push_back(pose);
    }
    return clip;
}

}  // namespace

int main() {
    pmg::ParametricMotionSpace space("walk", 1);
    space.AddExample({0.0f}, MakeClip(0.0f));
    space.AddExample({1.0f}, MakeClip(10.0f));

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

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "pmg_graph_io_test.pmg";
    pmg::SaveGraphText(graph, path.string());

    {
        std::ifstream input(path);
        std::string header;
        input >> header;
        assert(header == "PMG_GRAPH_V2");
    }

    const pmg::ParametricMotionGraph loaded = pmg::LoadGraphText(path.string());

    assert(loaded.NumNodes() == 1);
    assert(loaded.NumEdges() == 1);
    assert(loaded.Node(0).name == "walk");
    assert(loaded.Node(0).motion_space.NumExamples() == 2);
    assert(loaded.Edge(0).samples.size() == 1);
    assert(std::abs(loaded.Edge(0).samples[0].source_transition_phase - 0.8f) < 1.0e-5f);

    const pmg::Pose pose = loaded.Node(0).motion_space.EvaluatePose({1.0f}, 1.0f);
    assert(std::abs(pose.root_position.x - 11.0f) < 1.0e-5f);

    std::filesystem::remove(path);
    return 0;
}
