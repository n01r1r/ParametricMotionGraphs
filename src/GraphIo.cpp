#include "pmg/GraphIo.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace pmg {

namespace {

// One artifact version's on-disk grammar. Versions differ only in which
// optional records are present and whether names are quoted, so every version
// rule lives in the single table below (FormatForHeader): adding a version is
// one row, not a new branch threaded through the reader.
struct GraphPayloadFormat {
    bool has_metadata_and_skeleton;  // V4+ precede the graph with metadata + skeleton
    bool has_warp_records;           // V3+ store per-example time warps
    bool has_calibration_records;    // V5+ store parameter calibration
    bool has_vector_calibration;     // V7+ store multidimensional calibration
    bool has_target_phase_samples;   // V6+ store per-target transition phases
    bool quoted_names;               // V4+ quote node/clip/joint names
    bool has_transition_convention;  // V8+ store the edge transition-window convention
};

std::optional<GraphPayloadFormat> FormatForHeader(const std::string& header) {
    //                                       meta+skel warp  calib vector target quoted conv
    if (header == "PMG_GRAPH_V8") return GraphPayloadFormat{true,  true,  true,  true,  true,  true,  true};
    if (header == "PMG_GRAPH_V7") return GraphPayloadFormat{true,  true,  true,  true,  true,  true,  false};
    if (header == "PMG_GRAPH_V6") return GraphPayloadFormat{true,  true,  true,  false, true,  true,  false};
    if (header == "PMG_GRAPH_V5") return GraphPayloadFormat{true,  true,  true,  false, false, true,  false};
    if (header == "PMG_GRAPH_V4") return GraphPayloadFormat{true,  true,  false, false, false, true,  false};
    if (header == "PMG_GRAPH_V3") return GraphPayloadFormat{false, true,  false, false, false, false, false};
    if (header == "PMG_GRAPH_V2") return GraphPayloadFormat{false, false, false, false, false, false, false};
    return std::nullopt;
}

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
        throw std::runtime_error("LoadPmgArtifactText: failed to read parameter vector");
    }
    return parameter;
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

void WriteClip(std::ostream& output, const MotionClip& clip) {
    output << "clip " << std::quoted(clip.name) << ' ' << clip.frames_per_second
           << ' ' << clip.frames.size() << '\n';
    for (const Pose& pose : clip.frames) {
        output << "frame " << pose.root_position.x << ' ' << pose.root_position.y
               << ' ' << pose.root_position.z << ' ' << pose.local_rotations.size();
        for (const Quaternion& rotation : pose.local_rotations) {
            output << ' ' << rotation.w << ' ' << rotation.x << ' ' << rotation.y
                   << ' ' << rotation.z;
        }
        output << '\n';
    }
}

MotionClip ReadClip(std::istream& input, bool quoted_names) {
    std::string keyword;
    MotionClip clip;
    std::size_t frame_count = 0;
    input >> keyword;
    if (quoted_names) {
        input >> std::quoted(clip.name);
    } else {
        input >> clip.name;
    }
    input >> clip.frames_per_second >> frame_count;
    if (keyword != "clip" || !input) {
        throw std::runtime_error("LoadPmgArtifactText: expected clip record");
    }
    clip.frames.reserve(frame_count);
    for (std::size_t frame_index = 0; frame_index < frame_count; ++frame_index) {
        Pose pose;
        std::size_t rotation_count = 0;
        input >> keyword >> pose.root_position.x >> pose.root_position.y
              >> pose.root_position.z >> rotation_count;
        if (keyword != "frame" || !input) {
            throw std::runtime_error("LoadPmgArtifactText: expected frame record");
        }
        pose.local_rotations.resize(rotation_count);
        for (Quaternion& rotation : pose.local_rotations) {
            input >> rotation.w >> rotation.x >> rotation.y >> rotation.z;
        }
        if (!input) {
            throw std::runtime_error(
                "LoadPmgArtifactText: failed to read frame rotations");
        }
        clip.frames.push_back(pose);
    }
    return clip;
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
        throw std::runtime_error("LoadPmgArtifactText: failed to read warp phases");
    }
    return phases;
}

void WriteBuilderConfig(std::ostream& output, const PmgBuilderConfig& config) {
    const DistanceGridConfig& grid = config.distance_grid;
    output << config.source_sample_count << ' ' << config.target_sample_count << ' '
           << config.generated_frame_count << ' '
           << config.generated_frames_per_second << ' '
           << config.good_transition_threshold << ' '
           << config.bad_transition_threshold << ' ' << config.seed << ' '
           << config.box_shrink_epsilon << ' '
           << static_cast<int>(config.include_example_parameters) << ' '
           << grid.window_size << ' ' << grid.source_frame_stride << ' '
           << grid.target_frame_stride << ' ' << grid.source_phase_start << ' '
           << grid.source_phase_end << ' ' << grid.target_phase_start << ' '
           << grid.target_phase_end << ' '
           << static_cast<int>(config.transition_convention);
}

PmgBuilderConfig ReadBuilderConfig(
    std::istream& input, const GraphPayloadFormat& format) {
    PmgBuilderConfig config;
    int include_examples = 0;
    DistanceGridConfig& grid = config.distance_grid;
    input >> config.source_sample_count >> config.target_sample_count
          >> config.generated_frame_count >> config.generated_frames_per_second
          >> config.good_transition_threshold >> config.bad_transition_threshold
          >> config.seed >> config.box_shrink_epsilon >> include_examples
          >> grid.window_size >> grid.source_frame_stride
          >> grid.target_frame_stride >> grid.source_phase_start
          >> grid.source_phase_end >> grid.target_phase_start
          >> grid.target_phase_end;
    if (format.has_transition_convention) {
        int convention_value = 0;
        input >> convention_value;
        config.transition_convention =
            static_cast<TransitionWindowConvention>(convention_value);
    } else {
        // Pre-V8 artifacts were built with the Kovar directional window; pin
        // legacy semantics explicitly so the centered default does not silently
        // reinterpret them.
        config.transition_convention =
            TransitionWindowConvention::kKovarDirectional;
    }
    if (!input) {
        throw std::runtime_error(
            "LoadPmgArtifactText: invalid edge builder configuration");
    }
    config.include_example_parameters = include_examples != 0;
    return config;
}

void WriteGraphPayload(std::ostream& output, const ParametricMotionGraph& graph) {
    output << "nodes " << graph.NumNodes() << '\n';
    for (int node_index = 0; node_index < graph.NumNodes(); ++node_index) {
        const PmgNode& node = graph.Node(node_index);
        const ParametricMotionSpace& space = node.motion_space;
        output << "node " << std::quoted(node.name) << ' '
               << space.ParameterDimension() << ' ' << space.NumExamples() << '\n';
        for (const ExampleMotion& example : space.Examples()) {
            output << "example ";
            WriteParameter(output, example.parameter);
            output << '\n';
            WriteClip(output, example.clip);
        }
        const std::vector<TimeWarp>& warps = space.ExampleTimeWarps();
        output << "warps " << warps.size() << '\n';
        for (const TimeWarp& warp : warps) {
            output << "warp ";
            WritePhaseList(output, warp.InteriorFromPhases());
            output << ' ';
            WritePhaseList(output, warp.InteriorToPhases());
            output << '\n';
        }
        const ParameterCalibration& calibration =
            space.ParameterCalibrationData();
        output << "calibration " << calibration.metrics.size();
        for (const ParameterMetric metric : calibration.metrics) {
            output << ' ' << static_cast<int>(metric);
        }
        output << ' ' << calibration.samples_per_axis;
        output << '\n';
        if (space.HasParameterCalibration()) {
            output << "metric_scales ";
            WriteParameter(output, calibration.metric_scales);
            output << '\n';
            output << "example_metrics "
                   << calibration.example_measured.size() << '\n';
            for (const ParameterVector& measured :
                 calibration.example_measured) {
                output << "example_metric ";
                WriteParameter(output, measured);
                output << '\n';
            }
            output << "calibration_samples "
                   << calibration.samples.size() << '\n';
            for (const CalibrationSample& sample : calibration.samples) {
                output << "calibration_sample ";
                WriteParameter(output, sample.measured_parameter);
                output << ' ';
                WriteParameter(output, sample.blend_weights);
                output << '\n';
            }
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
                   << sample.target_transition_phase << ' '
                   << sample.target_phase_samples.size() << '\n';
            for (const TargetTransitionPhaseSample& phase_sample :
                 sample.target_phase_samples) {
                output << "target_phase ";
                WriteParameter(output, phase_sample.target_parameter);
                output << ' ' << phase_sample.source_transition_phase
                       << ' ' << phase_sample.target_transition_phase
                       << '\n';
            }
        }
    }
}

ParametricMotionGraph ReadGraphPayload(
    std::istream& input, const GraphPayloadFormat& format) {
    std::string keyword;
    int node_count = 0;
    input >> keyword >> node_count;
    if (keyword != "nodes" || node_count < 0) {
        throw std::runtime_error("LoadPmgArtifactText: expected nodes record");
    }

    ParametricMotionGraph graph;
    for (int node_index = 0; node_index < node_count; ++node_index) {
        std::string node_name;
        int parameter_dimension = 0;
        int example_count = 0;
        input >> keyword;
        if (format.quoted_names) {
            input >> std::quoted(node_name);
        } else {
            input >> node_name;
        }
        input >> parameter_dimension >> example_count;
        if (keyword != "node" || parameter_dimension <= 0 || example_count < 0) {
            throw std::runtime_error("LoadPmgArtifactText: invalid node record");
        }
        ParametricMotionSpace space(node_name, parameter_dimension);
        for (int example_index = 0; example_index < example_count; ++example_index) {
            input >> keyword;
            if (keyword != "example") {
                throw std::runtime_error(
                    "LoadPmgArtifactText: expected example record");
            }
            const ParameterVector parameter = ReadParameter(input);
            space.AddExample(parameter, ReadClip(input, format.quoted_names));
        }
        if (format.has_warp_records) {
            int warp_count = 0;
            input >> keyword >> warp_count;
            if (keyword != "warps" || warp_count < 0 ||
                (warp_count != 0 && warp_count != example_count)) {
                throw std::runtime_error(
                    "LoadPmgArtifactText: invalid warps record");
            }
            if (warp_count > 0) {
                std::vector<TimeWarp> warps;
                warps.reserve(warp_count);
                for (int warp_index = 0; warp_index < warp_count; ++warp_index) {
                    input >> keyword;
                    if (keyword != "warp") {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: expected warp record");
                    }
                    const std::vector<float> from_phases = ReadPhaseList(input);
                    const std::vector<float> to_phases = ReadPhaseList(input);
                    warps.push_back(
                        TimeWarp::FromAnchors(from_phases, to_phases));
                }
                space.SetExampleTimeWarps(std::move(warps));
            }
        }
        if (format.has_calibration_records) {
            if (format.has_vector_calibration) {
                std::size_t metric_count = 0;
                input >> keyword >> metric_count;
                if (keyword != "calibration" ||
                    metric_count > static_cast<std::size_t>(parameter_dimension) ||
                    !input) {
                    throw std::runtime_error(
                        "LoadPmgArtifactText: invalid calibration record");
                }
                if (metric_count > 0) {
                    ParameterCalibration calibration;
                    calibration.metrics.reserve(metric_count);
                    for (std::size_t metric_index = 0;
                         metric_index < metric_count; ++metric_index) {
                        int metric_value = 0;
                        input >> metric_value;
                        if (metric_value < 1 || metric_value > 2) {
                            throw std::runtime_error(
                                "LoadPmgArtifactText: invalid parameter metric");
                        }
                        calibration.metrics.push_back(
                            static_cast<ParameterMetric>(metric_value));
                    }
                    input >> calibration.samples_per_axis;
                    if (!input || calibration.samples_per_axis < 0) {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: invalid calibration sampling "
                            "configuration");
                    }
                    input >> keyword;
                    if (keyword != "metric_scales") {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: expected metric_scales record");
                    }
                    calibration.metric_scales = ReadParameter(input);

                    std::size_t example_metric_count = 0;
                    input >> keyword >> example_metric_count;
                    if (keyword != "example_metrics" ||
                        example_metric_count !=
                            static_cast<std::size_t>(example_count)) {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: invalid example_metrics record");
                    }
                    for (std::size_t example_index = 0;
                         example_index < example_metric_count; ++example_index) {
                        input >> keyword;
                        if (keyword != "example_metric") {
                            throw std::runtime_error(
                                "LoadPmgArtifactText: expected example_metric record");
                        }
                        calibration.example_measured.push_back(
                            ReadParameter(input));
                    }

                    std::size_t calibration_sample_count = 0;
                    input >> keyword >> calibration_sample_count;
                    if (keyword != "calibration_samples") {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: invalid calibration_samples record");
                    }
                    for (std::size_t sample_index = 0;
                         sample_index < calibration_sample_count; ++sample_index) {
                        input >> keyword;
                        if (keyword != "calibration_sample") {
                            throw std::runtime_error(
                                "LoadPmgArtifactText: expected calibration_sample record");
                        }
                        CalibrationSample sample;
                        sample.measured_parameter = ReadParameter(input);
                        sample.blend_weights = ReadParameter(input);
                        calibration.samples.push_back(std::move(sample));
                    }
                    space.SetParameterCalibration(std::move(calibration));
                } else {
                    int samples_per_axis = 0;
                    input >> samples_per_axis;
                    if (!input || samples_per_axis != 0) {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: invalid empty calibration record");
                    }
                }
                graph.AddNode(node_name, std::move(space));
                continue;
            }

            int metric_value = -1;
            input >> keyword >> metric_value;
            if (keyword != "calibration" || metric_value < 0 || metric_value > 1 ||
                !input) {
                throw std::runtime_error(
                    "LoadPmgArtifactText: invalid calibration record");
            }
            if (metric_value != 0) {
                ParameterCalibration calibration;
                calibration.metrics.push_back(
                    static_cast<ParameterMetric>(metric_value));
                std::size_t ordered_count = 0;
                input >> ordered_count;
                if (ordered_count !=
                    static_cast<std::size_t>(example_count)) {
                    throw std::runtime_error(
                        "LoadPmgArtifactText: calibration example count mismatch");
                }
                std::vector<int> example_order(ordered_count);
                for (int& example_index : example_order) {
                    input >> example_index;
                }
                std::vector<float> ordered_measured(ordered_count);
                for (float& measured : ordered_measured) {
                    input >> measured;
                }
                calibration.example_measured.assign(
                    static_cast<std::size_t>(example_count),
                    ParameterVector(1, 0.0f));
                for (std::size_t ordered_index = 0;
                     ordered_index < ordered_count; ++ordered_index) {
                    const int example_index = example_order[ordered_index];
                    if (example_index < 0 || example_index >= example_count) {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: invalid calibration example index");
                    }
                    calibration.example_measured[example_index][0] =
                        ordered_measured[ordered_index];
                    std::vector<float> weights(
                        static_cast<std::size_t>(example_count), 0.0f);
                    weights[example_index] = 1.0f;
                    calibration.samples.push_back(
                        {{ordered_measured[ordered_index]},
                         std::move(weights)});
                }
                std::size_t segment_count = 0;
                input >> segment_count;
                if (!input) {
                    throw std::runtime_error(
                        "LoadPmgArtifactText: invalid calibration table");
                }
                for (std::size_t segment_index = 0; segment_index < segment_count;
                     ++segment_index) {
                    int left_example = -1;
                    int right_example = -1;
                    std::size_t sample_count = 0;
                    input >> keyword >> left_example
                          >> right_example >> sample_count;
                    if (keyword != "segment" || !input) {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: invalid calibration segment");
                    }
                    if (left_example < 0 || left_example >= example_count ||
                        right_example < 0 || right_example >= example_count) {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: invalid calibration segment "
                            "example index");
                    }
                    for (std::size_t sample_index = 0;
                         sample_index < sample_count; ++sample_index) {
                        float blend_t = 0.0f;
                        float measured = 0.0f;
                        input >> blend_t >> measured;
                        CalibrationSample sample;
                        sample.measured_parameter = {measured};
                        sample.blend_weights.assign(
                            static_cast<std::size_t>(example_count), 0.0f);
                        sample.blend_weights[left_example] = 1.0f - blend_t;
                        sample.blend_weights[right_example] = blend_t;
                        calibration.samples.push_back(std::move(sample));
                    }
                    if (!input) {
                        throw std::runtime_error(
                            "LoadPmgArtifactText: invalid calibration samples");
                    }
                }
                float minimum = std::numeric_limits<float>::infinity();
                float maximum = -std::numeric_limits<float>::infinity();
                for (const CalibrationSample& sample : calibration.samples) {
                    minimum = std::min(
                        minimum, sample.measured_parameter.front());
                    maximum = std::max(
                        maximum, sample.measured_parameter.front());
                }
                const float range = maximum - minimum;
                calibration.metric_scales = {
                    std::isfinite(range) && range > kSmallEpsilon
                        ? range
                        : 1.0f};
                space.SetParameterCalibration(std::move(calibration));
            }
        }
        graph.AddNode(node_name, std::move(space));
    }

    int edge_count = 0;
    input >> keyword >> edge_count;
    if (keyword != "edges" || edge_count < 0) {
        throw std::runtime_error("LoadPmgArtifactText: expected edges record");
    }
    for (int edge_index = 0; edge_index < edge_count; ++edge_index) {
        PmgEdge edge;
        int sample_count = 0;
        input >> keyword >> edge.source_node >> edge.target_node >> sample_count;
        if (keyword != "edge" || sample_count < 0) {
            throw std::runtime_error("LoadPmgArtifactText: invalid edge record");
        }
        for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
            input >> keyword;
            if (keyword != "sample") {
                throw std::runtime_error(
                    "LoadPmgArtifactText: expected sample record");
            }
            TransitionSample sample;
            sample.source_parameter = ReadParameter(input);
            sample.target_parameter_box = ReadAabb(input);
            input >> sample.source_transition_phase
                  >> sample.target_transition_phase;
            std::size_t target_phase_sample_count = 0;
            if (format.has_target_phase_samples) {
                input >> target_phase_sample_count;
            }
            if (!input) {
                throw std::runtime_error(
                    "LoadPmgArtifactText: invalid transition sample");
            }
            for (std::size_t phase_index = 0;
                 phase_index < target_phase_sample_count; ++phase_index) {
                input >> keyword;
                if (keyword != "target_phase") {
                    throw std::runtime_error(
                        "LoadPmgArtifactText: expected target_phase record");
                }
                TargetTransitionPhaseSample phase_sample;
                phase_sample.target_parameter = ReadParameter(input);
                input >> phase_sample.source_transition_phase
                      >> phase_sample.target_transition_phase;
                if (!input) {
                    throw std::runtime_error(
                        "LoadPmgArtifactText: invalid target phase sample");
                }
                sample.target_phase_samples.push_back(
                    std::move(phase_sample));
            }
            edge.samples.push_back(std::move(sample));
        }
        graph.AddEdge(std::move(edge));
    }
    return graph;
}

void WriteSkeleton(std::ostream& output, const Skeleton& skeleton) {
    output << "skeleton " << skeleton.joints.size() << '\n';
    for (const Joint& joint : skeleton.joints) {
        output << "joint " << std::quoted(joint.name) << ' ' << joint.parent_index
               << ' ' << joint.offset.x << ' ' << joint.offset.y << ' '
               << joint.offset.z << ' ' << joint.channels.size();
        for (const BvhChannelType channel : joint.channels) {
            output << ' ' << static_cast<int>(channel);
        }
        output << '\n';
    }
}

Skeleton ReadSkeleton(std::istream& input) {
    std::string keyword;
    int joint_count = 0;
    input >> keyword >> joint_count;
    if (keyword != "skeleton" || joint_count < 0) {
        throw std::runtime_error("LoadPmgArtifactText: invalid skeleton record");
    }
    Skeleton skeleton;
    skeleton.joints.reserve(joint_count);
    for (int joint_index = 0; joint_index < joint_count; ++joint_index) {
        Joint joint;
        int channel_count = 0;
        input >> keyword >> std::quoted(joint.name) >> joint.parent_index
              >> joint.offset.x >> joint.offset.y >> joint.offset.z
              >> channel_count;
        if (keyword != "joint" || channel_count < 0 || !input) {
            throw std::runtime_error("LoadPmgArtifactText: invalid joint record");
        }
        for (int channel_index = 0; channel_index < channel_count; ++channel_index) {
            int channel = 0;
            input >> channel;
            if (channel < static_cast<int>(BvhChannelType::XPosition) ||
                channel > static_cast<int>(BvhChannelType::ZRotation)) {
                throw std::runtime_error(
                    "LoadPmgArtifactText: invalid BVH channel");
            }
            joint.channels.push_back(static_cast<BvhChannelType>(channel));
        }
        skeleton.joints.push_back(std::move(joint));
    }
    return skeleton;
}

void WriteMetadata(std::ostream& output, const PmgArtifactMetadata& metadata) {
    output << "runtime " << metadata.generated_frame_count << ' '
           << metadata.frames_per_second << ' ' << std::quoted(metadata.units)
           << '\n';
    output << "sources " << metadata.source_bvh_paths.size() << '\n';
    for (const std::string& path : metadata.source_bvh_paths) {
        output << "source " << std::quoted(path) << '\n';
    }
    output << "registrations " << metadata.node_registrations.size() << '\n';
    for (const NodeRegistrationMetadata& registration :
         metadata.node_registrations) {
        output << "registration " << std::quoted(registration.node_name) << ' '
               << std::quoted(registration.cycle_joint) << ' '
               << registration.contact_joints.size();
        for (const std::string& joint : registration.contact_joints) {
            output << ' ' << std::quoted(joint);
        }
        output << ' ' << registration.min_contact_frames << ' '
               << static_cast<int>(registration.dtw_refine) << '\n';
    }
    output << "edge_builds " << metadata.edge_builds.size() << '\n';
    for (const EdgeBuildMetadata& edge : metadata.edge_builds) {
        output << "edge_build " << std::quoted(edge.source_node) << ' '
               << std::quoted(edge.target_node) << ' ';
        WriteBuilderConfig(output, edge.config);
        output << ' ' << static_cast<int>(edge.report.edge_created) << ' '
               << std::quoted(edge.report.reject_reason) << ' '
               << edge.report.source_reports.size() << '\n';
        for (const SourceSampleBuildReport& source :
             edge.report.source_reports) {
            output << "source_report ";
            WriteParameter(output, source.source_parameter);
            output << ' ' << source.good_count << ' ' << source.neutral_count
                   << ' ' << source.bad_count << ' ' << source.min_distance
                   << ' ' << source.p25_distance << ' ' << source.median_distance
                   << ' ' << source.max_distance << ' ';
            WriteAabb(output, source.target_box_before_shrink);
            output << ' ';
            WriteAabb(output, source.target_box_after_shrink);
            output << ' ' << static_cast<int>(source.accepted) << ' '
                   << std::quoted(source.reject_reason) << '\n';
        }
    }
}

PmgArtifactMetadata ReadMetadata(
    std::istream& input, const GraphPayloadFormat& format) {
    std::string keyword;
    PmgArtifactMetadata metadata;
    input >> keyword >> metadata.generated_frame_count
          >> metadata.frames_per_second >> std::quoted(metadata.units);
    if (keyword != "runtime" || metadata.generated_frame_count < 0 ||
        metadata.frames_per_second < 0.0f) {
        throw std::runtime_error("LoadPmgArtifactText: invalid runtime record");
    }

    std::size_t source_count = 0;
    input >> keyword >> source_count;
    if (keyword != "sources") {
        throw std::runtime_error("LoadPmgArtifactText: expected sources record");
    }
    for (std::size_t index = 0; index < source_count; ++index) {
        std::string path;
        input >> keyword >> std::quoted(path);
        if (keyword != "source") {
            throw std::runtime_error(
                "LoadPmgArtifactText: expected source record");
        }
        metadata.source_bvh_paths.push_back(std::move(path));
    }

    std::size_t registration_count = 0;
    input >> keyword >> registration_count;
    if (keyword != "registrations") {
        throw std::runtime_error(
            "LoadPmgArtifactText: expected registrations record");
    }
    for (std::size_t index = 0; index < registration_count; ++index) {
        NodeRegistrationMetadata registration;
        std::size_t contact_count = 0;
        int dtw_refine = 0;
        input >> keyword >> std::quoted(registration.node_name)
              >> std::quoted(registration.cycle_joint) >> contact_count;
        if (keyword != "registration") {
            throw std::runtime_error(
                "LoadPmgArtifactText: invalid registration record");
        }
        for (std::size_t joint = 0; joint < contact_count; ++joint) {
            std::string joint_name;
            input >> std::quoted(joint_name);
            registration.contact_joints.push_back(std::move(joint_name));
        }
        input >> registration.min_contact_frames >> dtw_refine;
        registration.dtw_refine = dtw_refine != 0;
        metadata.node_registrations.push_back(std::move(registration));
    }

    std::size_t edge_build_count = 0;
    input >> keyword >> edge_build_count;
    if (keyword != "edge_builds") {
        throw std::runtime_error(
            "LoadPmgArtifactText: expected edge_builds record");
    }
    for (std::size_t index = 0; index < edge_build_count; ++index) {
        EdgeBuildMetadata edge;
        int edge_created = 0;
        std::size_t source_report_count = 0;
        input >> keyword >> std::quoted(edge.source_node)
              >> std::quoted(edge.target_node);
        if (keyword != "edge_build") {
            throw std::runtime_error(
                "LoadPmgArtifactText: invalid edge_build record");
        }
        edge.config = ReadBuilderConfig(input, format);
        input >> edge_created >> std::quoted(edge.report.reject_reason)
              >> source_report_count;
        edge.report.edge_created = edge_created != 0;
        for (std::size_t report_index = 0;
             report_index < source_report_count; ++report_index) {
            SourceSampleBuildReport source;
            int accepted = 0;
            input >> keyword;
            if (keyword != "source_report") {
                throw std::runtime_error(
                    "LoadPmgArtifactText: invalid source_report record");
            }
            source.source_parameter = ReadParameter(input);
            input >> source.good_count >> source.neutral_count >> source.bad_count
                  >> source.min_distance >> source.p25_distance
                  >> source.median_distance >> source.max_distance;
            source.target_box_before_shrink = ReadAabb(input);
            source.target_box_after_shrink = ReadAabb(input);
            input >> accepted >> std::quoted(source.reject_reason);
            source.accepted = accepted != 0;
            edge.report.source_reports.push_back(std::move(source));
        }
        metadata.edge_builds.push_back(std::move(edge));
    }
    return metadata;
}

}  // namespace

void SavePmgArtifactText(
    const BuiltPmgArtifact& artifact, const std::string& path) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error(
            "SavePmgArtifactText: failed to open '" + path + "'");
    }
    output << std::setprecision(9);
    output << "PMG_GRAPH_V8\n";
    WriteMetadata(output, artifact.metadata);
    WriteSkeleton(output, artifact.skeleton);
    WriteGraphPayload(output, artifact.graph);
}

BuiltPmgArtifact LoadPmgArtifactText(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error(
            "LoadPmgArtifactText: failed to open '" + path + "'");
    }
    std::string header;
    input >> header;

    const std::optional<GraphPayloadFormat> format = FormatForHeader(header);
    if (!format) {
        throw std::runtime_error(
            "LoadPmgArtifactText: expected PMG_GRAPH_V2 through V8");
    }

    BuiltPmgArtifact artifact;
    if (format->has_metadata_and_skeleton) {
        artifact.metadata = ReadMetadata(input, *format);
        artifact.skeleton = ReadSkeleton(input);
    }
    artifact.graph = ReadGraphPayload(input, *format);
    return artifact;
}

void SaveGraphText(
    const ParametricMotionGraph& graph, const std::string& path) {
    BuiltPmgArtifact artifact;
    artifact.graph = graph;
    SavePmgArtifactText(artifact, path);
}

ParametricMotionGraph LoadGraphText(const std::string& path) {
    return LoadPmgArtifactText(path).graph;
}

}  // namespace pmg
