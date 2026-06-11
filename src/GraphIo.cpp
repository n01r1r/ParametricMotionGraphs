#include "pmg/GraphIo.h"

#include <fstream>
#include <stdexcept>
#include <utility>

namespace pmg {

namespace {

void WriteParameter(std::ostream& output, const ParameterVector& parameter) {
    output << parameter.size();
    for (const float value : parameter) {
        output << ' ' << value;
    }
}

ParameterVector ReadParameter(std::istream& input) {
    std::size_t size = 0;
    input >> size;
    ParameterVector parameter(size);
    for (float& value : parameter) {
        input >> value;
    }
    if (!input) {
        throw std::runtime_error("LoadGraphText: failed to read parameter vector");
    }
    return parameter;
}

void WriteClip(std::ostream& output, const MotionClip& clip) {
    output << "clip " << clip.name << ' ' << clip.frames_per_second << ' '
           << clip.frames.size() << '\n';
    for (const Pose& pose : clip.frames) {
        output << "frame " << pose.root_position.x << ' ' << pose.root_position.y << ' '
               << pose.root_position.z << ' ' << pose.local_rotations.size();
        for (const Quaternion& rotation : pose.local_rotations) {
            output << ' ' << rotation.w << ' ' << rotation.x << ' ' << rotation.y << ' '
                   << rotation.z;
        }
        output << '\n';
    }
}

MotionClip ReadClip(std::istream& input) {
    std::string keyword;
    MotionClip clip;
    std::size_t frame_count = 0;
    input >> keyword >> clip.name >> clip.frames_per_second >> frame_count;
    if (keyword != "clip" || !input) {
        throw std::runtime_error("LoadGraphText: expected clip record");
    }
    clip.frames.reserve(frame_count);
    for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
        Pose pose;
        std::size_t rotation_count = 0;
        input >> keyword >> pose.root_position.x >> pose.root_position.y >> pose.root_position.z
              >> rotation_count;
        if (keyword != "frame" || !input) {
            throw std::runtime_error("LoadGraphText: expected frame record");
        }
        pose.local_rotations.resize(rotation_count);
        for (Quaternion& rotation : pose.local_rotations) {
            input >> rotation.w >> rotation.x >> rotation.y >> rotation.z;
        }
        if (!input) {
            throw std::runtime_error("LoadGraphText: failed to read frame rotations");
        }
        clip.frames.push_back(pose);
    }
    return clip;
}

void WriteAabb(std::ostream& output, const ParameterAabb& box) {
    WriteParameter(output, box.min_corner);
    output << ' ';
    WriteParameter(output, box.max_corner);
}

ParameterAabb ReadAabb(std::istream& input) {
    ParameterAabb box;
    box.min_corner = ReadParameter(input);
    box.max_corner = ReadParameter(input);
    return box;
}

void WritePhaseList(std::ostream& output, const std::vector<float>& phases) {
    output << phases.size();
    for (const float phase : phases) {
        output << ' ' << phase;
    }
}

std::vector<float> ReadPhaseList(std::istream& input) {
    std::size_t size = 0;
    input >> size;
    std::vector<float> phases(size);
    for (float& phase : phases) {
        input >> phase;
    }
    if (!input) {
        throw std::runtime_error("LoadGraphText: failed to read warp phase list");
    }
    return phases;
}

}  // namespace

void SaveGraphText(const ParametricMotionGraph& graph, const std::string& path) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("SaveGraphText: failed to open '" + path + "'");
    }
    output << "PMG_GRAPH_V3\n";
    output << "nodes " << graph.NumNodes() << '\n';
    for (int node_index = 0; node_index < graph.NumNodes(); ++node_index) {
        const PmgNode& node = graph.Node(node_index);
        const ParametricMotionSpace& space = node.motion_space;
        output << "node " << node.name << ' ' << space.ParameterDimension() << ' '
               << space.NumExamples() << '\n';
        for (const ExampleMotion& example : space.Examples()) {
            output << "example ";
            WriteParameter(output, example.parameter);
            output << '\n';
            WriteClip(output, example.clip);
        }
        // Registration warps: count is NumExamples for registered spaces,
        // 0 for unregistered ones. Dropping them on load would silently
        // revert EvaluatePose to unregistered (raw-phase) blending.
        const std::vector<TimeWarp>& warps = space.ExampleTimeWarps();
        output << "warps " << warps.size() << '\n';
        for (const TimeWarp& warp : warps) {
            output << "warp ";
            WritePhaseList(output, warp.InteriorFromPhases());
            output << ' ';
            WritePhaseList(output, warp.InteriorToPhases());
            output << '\n';
        }
    }

    output << "edges " << graph.NumEdges() << '\n';
    for (int edge_index = 0; edge_index < graph.NumEdges(); ++edge_index) {
        const PmgEdge& edge = graph.Edge(edge_index);
        output << "edge " << edge.source_node << ' ' << edge.target_node << ' '
               << edge.samples.size() << '\n';
        for (const TransitionSample& sample : edge.samples) {
            output << "sample ";
            WriteParameter(output, sample.source_parameter);
            output << ' ';
            WriteAabb(output, sample.target_parameter_box);
            output << ' ' << sample.source_transition_phase << ' '
                   << sample.target_transition_phase << '\n';
        }
    }
}

ParametricMotionGraph LoadGraphText(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("LoadGraphText: failed to open '" + path + "'");
    }
    std::string keyword;
    input >> keyword;
    // V2 files predate registration warps; everything else is identical.
    const bool has_warp_records = keyword == "PMG_GRAPH_V3";
    if (keyword != "PMG_GRAPH_V2" && !has_warp_records) {
        throw std::runtime_error(
            "LoadGraphText: invalid graph file header (expected PMG_GRAPH_V2 or PMG_GRAPH_V3)");
    }

    ParametricMotionGraph graph;
    int node_count = 0;
    input >> keyword >> node_count;
    if (keyword != "nodes" || node_count < 0) {
        throw std::runtime_error("LoadGraphText: expected nodes record");
    }
    for (int node_index = 0; node_index < node_count; ++node_index) {
        std::string node_name;
        int parameter_dimension = 0;
        int example_count = 0;
        input >> keyword >> node_name >> parameter_dimension >> example_count;
        if (keyword != "node" || parameter_dimension <= 0 || example_count < 0) {
            throw std::runtime_error("LoadGraphText: invalid node record");
        }
        ParametricMotionSpace space(node_name, parameter_dimension);
        for (int example_index = 0; example_index < example_count; ++example_index) {
            input >> keyword;
            if (keyword != "example") {
                throw std::runtime_error("LoadGraphText: expected example record");
            }
            const ParameterVector parameter = ReadParameter(input);
            MotionClip clip = ReadClip(input);
            space.AddExample(parameter, std::move(clip));
        }
        if (has_warp_records) {
            int warp_count = 0;
            input >> keyword >> warp_count;
            if (keyword != "warps" || warp_count < 0 ||
                (warp_count != 0 && warp_count != example_count)) {
                throw std::runtime_error("LoadGraphText: invalid warps record");
            }
            if (warp_count > 0) {
                std::vector<TimeWarp> warps;
                warps.reserve(warp_count);
                for (int warp_index = 0; warp_index < warp_count; ++warp_index) {
                    input >> keyword;
                    if (keyword != "warp") {
                        throw std::runtime_error("LoadGraphText: expected warp record");
                    }
                    const std::vector<float> from_phases = ReadPhaseList(input);
                    const std::vector<float> to_phases = ReadPhaseList(input);
                    warps.push_back(TimeWarp::FromAnchors(from_phases, to_phases));
                }
                space.SetExampleTimeWarps(std::move(warps));
            }
        }
        graph.AddNode(node_name, std::move(space));
    }

    int edge_count = 0;
    input >> keyword >> edge_count;
    if (keyword != "edges" || edge_count < 0) {
        throw std::runtime_error("LoadGraphText: expected edges record");
    }
    for (int edge_index = 0; edge_index < edge_count; ++edge_index) {
        PmgEdge edge;
        int sample_count = 0;
        input >> keyword >> edge.source_node >> edge.target_node >> sample_count;
        if (keyword != "edge" || sample_count < 0) {
            throw std::runtime_error("LoadGraphText: invalid edge record");
        }
        for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
            input >> keyword;
            if (keyword != "sample") {
                throw std::runtime_error("LoadGraphText: expected sample record");
            }
            TransitionSample sample;
            sample.source_parameter = ReadParameter(input);
            sample.target_parameter_box = ReadAabb(input);
            input >> sample.source_transition_phase >> sample.target_transition_phase;
            if (!input) {
                throw std::runtime_error("LoadGraphText: failed to read transition sample");
            }
            edge.samples.push_back(std::move(sample));
        }
        graph.AddEdge(std::move(edge));
    }

    return graph;
}

}  // namespace pmg
