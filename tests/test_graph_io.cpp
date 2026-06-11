#include "pmg/GraphIo.h"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

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

pmg::ParametricMotionGraph MakeGraph(bool registered) {
    pmg::ParametricMotionSpace space("walk", 1);
    space.AddExample({0.0f}, MakeClip(0.0f));
    space.AddExample({1.0f}, MakeClip(10.0f));
    if (registered) {
        space.SetExampleTimeWarps({
            pmg::TimeWarp::FromAnchors({0.5f}, {0.35f}),
            pmg::TimeWarp::FromAnchors({0.5f}, {0.65f}),
        });
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

void CheckCommonStructure(const pmg::ParametricMotionGraph& loaded) {
    assert(loaded.NumNodes() == 1);
    assert(loaded.NumEdges() == 1);
    assert(loaded.Node(0).name == "walk");
    assert(loaded.Node(0).motion_space.NumExamples() == 2);
    assert(loaded.Edge(0).samples.size() == 1);
    assert(std::abs(loaded.Edge(0).samples[0].source_transition_phase - 0.8f) < 1.0e-5f);
}

void TestUnregisteredRoundTrip(const std::filesystem::path& path) {
    pmg::SaveGraphText(MakeGraph(false), path.string());

    {
        std::ifstream input(path);
        std::string header;
        input >> header;
        assert(header == "PMG_GRAPH_V3");
    }

    const pmg::ParametricMotionGraph loaded = pmg::LoadGraphText(path.string());
    CheckCommonStructure(loaded);
    assert(!loaded.Node(0).motion_space.HasExampleTimeWarps());

    const pmg::Pose pose = loaded.Node(0).motion_space.EvaluatePose({1.0f}, 1.0f);
    assert(std::abs(pose.root_position.x - 11.0f) < 1.0e-5f);
}

void TestRegisteredRoundTrip(const std::filesystem::path& path) {
    const pmg::ParametricMotionGraph original = MakeGraph(true);
    pmg::SaveGraphText(original, path.string());
    const pmg::ParametricMotionGraph loaded = pmg::LoadGraphText(path.string());
    CheckCommonStructure(loaded);

    const pmg::ParametricMotionSpace& space = loaded.Node(0).motion_space;
    assert(space.HasExampleTimeWarps());
    assert(space.ExampleTimeWarps().size() == 2);
    assert(std::abs(space.ExampleTimeWarps()[0].Evaluate(0.5f) - 0.35f) < 1.0e-5f);
    assert(std::abs(space.ExampleTimeWarps()[1].Evaluate(0.5f) - 0.65f) < 1.0e-5f);

    // Registered evaluation must survive the round trip bit-for-bit in
    // behavior: warped sampling, not raw-phase sampling.
    const pmg::ParametricMotionSpace& original_space = original.Node(0).motion_space;
    for (const float phase : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) {
        const pmg::Pose expected = original_space.EvaluatePose({0.5f}, phase);
        const pmg::Pose actual = space.EvaluatePose({0.5f}, phase);
        assert(std::abs(expected.root_position.x - actual.root_position.x) < 1.0e-5f);
    }
}

// Pre-warp files must still load (as unregistered spaces).
void TestV2BackwardCompat(const std::filesystem::path& path) {
    pmg::SaveGraphText(MakeGraph(false), path.string());
    std::stringstream contents;
    {
        std::ifstream input(path);
        contents << input.rdbuf();
    }
    std::string text = contents.str();
    text.replace(text.find("PMG_GRAPH_V3"), 12, "PMG_GRAPH_V2");
    const std::string warps_record = "warps 0\n";
    text.erase(text.find(warps_record), warps_record.size());
    {
        std::ofstream output(path);
        output << text;
    }

    const pmg::ParametricMotionGraph loaded = pmg::LoadGraphText(path.string());
    CheckCommonStructure(loaded);
    assert(!loaded.Node(0).motion_space.HasExampleTimeWarps());
}

}  // namespace

int main() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "pmg_graph_io_test.pmg";

    TestUnregisteredRoundTrip(path);
    TestRegisteredRoundTrip(path);
    TestV2BackwardCompat(path);

    std::filesystem::remove(path);
    return 0;
}
