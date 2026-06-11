#include "PmgCommands.h"

#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/FootLocking.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/GraphIo.h"
#include "pmg/GraphSpec.h"
#include "pmg/MotionDistance.h"
#include "pmg/MotionRegistration.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/RuntimeController.h"
#include "pmg/SkeletonCompatibility.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::string LowercaseCopy(std::string text) {
    std::transform(
        text.begin(), text.end(), text.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return text;
}

const char* ChannelName(pmg::BvhChannelType channel) {
    switch (channel) {
        case pmg::BvhChannelType::XPosition: return "Xposition";
        case pmg::BvhChannelType::YPosition: return "Yposition";
        case pmg::BvhChannelType::ZPosition: return "Zposition";
        case pmg::BvhChannelType::XRotation: return "Xrotation";
        case pmg::BvhChannelType::YRotation: return "Yrotation";
        case pmg::BvhChannelType::ZRotation: return "Zrotation";
    }
    return "Unknown";
}

std::optional<int> FindJointExact(
    const pmg::Skeleton& skeleton,
    const std::string& joint_name) {
    const std::string target = LowercaseCopy(joint_name);

    for (int joint_index = 0; joint_index < skeleton.NumJoints(); ++joint_index) {
        if (LowercaseCopy(skeleton.joints[joint_index].name) == target) {
            return joint_index;
        }
    }

    return std::nullopt;
}

float HorizontalLength(const pmg::Vec3& value) {
    return std::sqrt(value.x * value.x + value.z * value.z);
}

int PrintBvhSummary(const std::string& path) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);
    std::cout << "BVH: " << path << "\n";
    std::cout << "joints: " << data.skeleton.NumJoints() << "\n";
    std::cout << "frames: " << data.clip.NumFrames() << "\n";
    std::cout << "fps: " << data.clip.frames_per_second << "\n";
    return 0;
}

int ListBvhJoints(const std::string& path) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);

    for (int joint_index = 0; joint_index < data.skeleton.NumJoints(); ++joint_index) {
        const pmg::Joint& joint = data.skeleton.joints[joint_index];
        std::cout << joint_index
                  << " name=" << joint.name
                  << " parent=" << joint.parent_index
                  << " offset=("
                  << joint.offset.x << ", "
                  << joint.offset.y << ", "
                  << joint.offset.z << ")"
                  << " channels=";

        for (int channel_index = 0;
             channel_index < static_cast<int>(joint.channels.size());
             ++channel_index) {
            if (channel_index > 0) {
                std::cout << ",";
            }
            std::cout << ChannelName(joint.channels[channel_index]);
        }

        std::cout << "\n";
    }

    return 0;
}

// Heuristic action label from a filename: drop the extension, strip a trailing
// variant letter (WalkLoop[A]) and a trailing "Loop", then lowercase. Used only
// to group same-action vs different-action pairs for threshold calibration.
std::string ActionKey(const std::filesystem::path& path) {
    std::string stem = path.stem().string();
    while (!stem.empty() && std::isupper(static_cast<unsigned char>(stem.back()))) {
        stem.pop_back();
    }
    const std::string suffix = "Loop";
    if (stem.size() >= suffix.size()) {
        bool ends_with_loop = true;
        for (std::size_t i = 0; i < suffix.size(); ++i) {
            if (std::tolower(static_cast<unsigned char>(stem[stem.size() - suffix.size() + i])) !=
                std::tolower(static_cast<unsigned char>(suffix[i]))) {
                ends_with_loop = false;
                break;
            }
        }
        if (ends_with_loop) {
            stem.erase(stem.size() - suffix.size());
        }
    }
    return LowercaseCopy(stem);
}

// Linear-interpolated percentile of a sample set; p in [0,1].
float Percentile(std::vector<float> values, float p) {
    if (values.empty()) {
        return 0.0f;
    }
    std::sort(values.begin(), values.end());
    const float position = p * static_cast<float>(values.size() - 1);
    const std::size_t low = static_cast<std::size_t>(std::floor(position));
    const std::size_t high = static_cast<std::size_t>(std::ceil(position));
    const float fraction = position - static_cast<float>(low);
    return values[low] * (1.0f - fraction) + values[high] * fraction;
}

// Transition-region grid: late source phase -> early target phase, matching the
// PmgBuilderConfig defaults used by the edge builder (paper §3.1-3.2).
pmg::DistanceGridConfig TransitionGridConfig() {
    pmg::DistanceGridConfig config;
    config.window_size = 5;
    config.source_frame_stride = 2;
    config.target_frame_stride = 2;
    config.source_phase_start = 0.70f;
    config.source_phase_end = 0.95f;
    config.target_phase_start = 0.05f;
    config.target_phase_end = 0.30f;
    return config;
}

// Print the optimal transition between two clips, mirroring the viewer's
// Distance Grid readout so CLI and GUI report identical numbers.
int InspectTransition(const std::string& source_path, const std::string& target_path) {
    const pmg::BvhData source = pmg::BvhLoader::Load(source_path);
    const pmg::BvhData target = pmg::BvhLoader::Load(target_path);
    if (source.skeleton.NumJoints() != target.skeleton.NumJoints()) {
        throw std::runtime_error("InspectTransition: skeletons have different joint counts");
    }

    const pmg::DistanceGridConfig config = TransitionGridConfig();
    const pmg::OptimalTransition transition = pmg::MotionDistance::FindOptimalTransition(
        source.skeleton, source.clip, target.clip, config);

    // Paper's starting thresholds. BVH now loads in native units (no loader x10),
    // so the corpus distance scale matches the paper. See --calibrate-thresholds
    // for a tighter corpus-specific fit.
    constexpr float kGoodThreshold = 0.5f;
    constexpr float kBadThreshold = 0.7f;

    std::cout << "source_bvh=" << source_path << "\n";
    std::cout << "target_bvh=" << target_path << "\n";
    std::cout << "joints=" << source.skeleton.NumJoints() << "\n";
    std::cout << "source_frames=" << source.clip.NumFrames() << "\n";
    std::cout << "target_frames=" << target.clip.NumFrames() << "\n";

    if (transition.source_frame < 0) {
        std::cout << "transition=NONE (empty grid)\n";
        return 0;
    }

    const char* classification =
        transition.distance <= kGoodThreshold   ? "GOOD"
        : transition.distance >= kBadThreshold  ? "BAD"
                                                : "NEUTRAL";

    std::cout << "source_frame=" << transition.source_frame << "\n";
    std::cout << "target_frame=" << transition.target_frame << "\n";
    std::cout << "source_phase=" << transition.source_phase << "\n";
    std::cout << "target_phase=" << transition.target_phase << "\n";
    std::cout << "distance=" << transition.distance << "\n";
    std::cout << "TGOOD=" << kGoodThreshold << " TBAD=" << kBadThreshold << "\n";
    std::cout << "classification=" << classification << "\n";
    std::cout << "alignment_yaw=" << transition.alignment.yaw << "\n";
    std::cout << "alignment_dx=" << transition.alignment.dx << "\n";
    std::cout << "alignment_dz=" << transition.alignment.dz << "\n";
    std::cout << "NOTE: distance is unnormalized mean-squared per point in the BVH's "
                 "native units; still corpus-dependent, so calibrate per corpus.\n";
    return 0;
}

// Dump the full distance grid as CSV (source_frame,target_frame,distance) for
// offline heatmap plotting. Saving is optional; live viewing lives in the viewer.
int DumpDistanceGrid(const std::string& source_path, const std::string& target_path,
                     const std::string& out_csv) {
    const pmg::BvhData source = pmg::BvhLoader::Load(source_path);
    const pmg::BvhData target = pmg::BvhLoader::Load(target_path);
    if (source.skeleton.NumJoints() != target.skeleton.NumJoints()) {
        throw std::runtime_error("DumpDistanceGrid: skeletons have different joint counts");
    }

    const pmg::DistanceGridConfig config = TransitionGridConfig();
    const pmg::DistanceGrid grid = pmg::MotionDistance::BuildDistanceGrid(
        source.skeleton, source.clip, target.clip, config);

    std::ofstream out(out_csv);
    if (!out) {
        throw std::runtime_error("DumpDistanceGrid: cannot open output file: " + out_csv);
    }
    out << "source_frame,target_frame,distance\n";
    for (int source_index = 0; source_index < grid.SourceCount(); ++source_index) {
        for (int target_index = 0; target_index < grid.TargetCount(); ++target_index) {
            out << grid.source_frames[source_index] << ","
                << grid.target_frames[target_index] << ","
                << grid.At(source_index, target_index) << "\n";
        }
    }
    std::cout << "wrote " << grid.SourceCount() * grid.TargetCount()
              << " cells (" << grid.SourceCount() << " x " << grid.TargetCount()
              << ") to " << out_csv << "\n";
    return 0;
}

// Loop-ability of one clip: aligned point-cloud distance between its first and
// last frame windows. Small => clip loops (start pose ~= end pose).
float LoopGap(const pmg::Skeleton& skeleton, const pmg::MotionClip& clip, int window_size) {
    if (clip.NumFrames() < 2) {
        return 0.0f;
    }
    const pmg::PointCloud start_cloud =
        pmg::MotionDistance::BuildPointCloud(skeleton, clip, 0, window_size);
    const pmg::PointCloud end_cloud =
        pmg::MotionDistance::BuildPointCloud(skeleton, clip, clip.NumFrames() - 1, window_size);
    return pmg::MotionDistance::AlignedPointCloudDistance(start_cloud, end_cloud).distance;
}

// One manifest entry: filename (relative to the BVH dir) plus an explicit
// action label. Lines are "<filename> [action]"; blank lines and '#' comments
// are skipped. If action is omitted the filename heuristic (ActionKey) is used.
struct ManifestEntry {
    std::string filename;
    std::string action;  // empty => use ActionKey heuristic
};

std::vector<ManifestEntry> ReadManifest(const std::string& manifest_path) {
    std::ifstream in(manifest_path);
    if (!in) {
        throw std::runtime_error("ReadManifest: cannot open " + manifest_path);
    }
    std::vector<ManifestEntry> entries;
    std::string line;
    while (std::getline(in, line)) {
        // strip comment
        const std::size_t hash = line.find('#');
        if (hash != std::string::npos) {
            line.erase(hash);
        }
        std::istringstream tokens(line);
        ManifestEntry entry;
        if (!(tokens >> entry.filename)) {
            continue;  // blank / comment-only line
        }
        tokens >> entry.action;  // optional
        entries.push_back(entry);
    }
    return entries;
}

// Sweep BVH clips, measure optimal transition distances for every clip pair
// (self / same-action / different-action), report the distribution, and suggest
// TGOOD/TBAD. Pure analysis; calibrates the paper's thresholds to this corpus's
// distance scale. BVH values are loaded in native units, but absolute distances
// remain skeleton/window/corpus dependent. Additive: no core/runtime change.
//
// directory_path: folder holding the .bvh files.
// manifest_path:  optional. When non-empty, only the listed clips are used and
//   their explicit action labels group same-action pairs (so a locomotion-only
//   subset can be calibrated). When empty, every .bvh in the folder is swept and
//   actions come from the filename heuristic.
int CalibrateThresholds(const std::string& directory_path,
                        const std::string& manifest_path = "") {
    namespace fs = std::filesystem;

    std::error_code error;
    if (!fs::exists(directory_path, error)) {
        std::cout << "directory not found: " << directory_path << "\n";
        return 1;
    }

    struct Loaded {
        fs::path path;
        pmg::BvhData data;
        std::string action;
    };
    std::vector<Loaded> clips;
    const pmg::DistanceGridConfig config = TransitionGridConfig();

    if (manifest_path.empty()) {
        std::vector<fs::path> files;
        for (const auto& entry : fs::directory_iterator(directory_path, error)) {
            if (entry.is_regular_file() && entry.path().extension() == ".bvh") {
                files.push_back(entry.path());
            }
        }
        std::sort(files.begin(), files.end());
        if (files.empty()) {
            std::cout << "no .bvh files in " << directory_path << "\n";
            return 1;
        }
        for (const fs::path& file : files) {
            try {
                pmg::BvhData data = pmg::BvhLoader::Load(file.string());
                clips.push_back({file, std::move(data), ActionKey(file)});
            } catch (const std::exception& load_error) {
                std::cout << "skip " << file.filename().string() << ": "
                          << load_error.what() << "\n";
            }
        }
    } else {
        const std::vector<ManifestEntry> entries = ReadManifest(manifest_path);
        std::cout << "manifest: " << manifest_path << " (" << entries.size() << " entries)\n";
        for (const ManifestEntry& entry : entries) {
            const fs::path file = fs::path(directory_path) / entry.filename;
            try {
                pmg::BvhData data = pmg::BvhLoader::Load(file.string());
                const std::string action =
                    entry.action.empty() ? ActionKey(file) : entry.action;
                clips.push_back({file, std::move(data), action});
            } catch (const std::exception& load_error) {
                std::cout << "skip " << entry.filename << ": " << load_error.what() << "\n";
            }
        }
        if (clips.empty()) {
            std::cout << "no clips loaded from manifest.\n";
            return 1;
        }
    }

    std::cout << "=== per-clip loop gap (start<->end aligned cloud distance; small => loops) ===\n";
    for (const Loaded& clip : clips) {
        const float gap = LoopGap(clip.data.skeleton, clip.data.clip, config.window_size);
        std::cout << clip.path.filename().string()
                  << "  action=" << clip.action
                  << "  joints=" << clip.data.skeleton.NumJoints()
                  << "  frames=" << clip.data.clip.NumFrames()
                  << "  loop_gap=" << gap << "\n";
    }

    std::vector<float> self_distances;
    std::vector<float> same_distances;
    std::vector<float> diff_distances;

    std::cout << "\n=== pairwise optimal transition distance ===\n";
    std::cout << "source,target,type,distance\n";
    for (std::size_t i = 0; i < clips.size(); ++i) {
        for (std::size_t j = 0; j < clips.size(); ++j) {
            if (clips[i].data.skeleton.NumJoints() != clips[j].data.skeleton.NumJoints()) {
                continue;
            }
            const pmg::OptimalTransition transition = pmg::MotionDistance::FindOptimalTransition(
                clips[i].data.skeleton, clips[i].data.clip, clips[j].data.clip, config);
            if (transition.source_frame < 0) {
                continue;
            }
            const bool is_self = (i == j);
            const bool is_same = (clips[i].action == clips[j].action);
            const char* type = is_self ? "self" : is_same ? "same" : "diff";
            if (is_self) {
                self_distances.push_back(transition.distance);
            } else if (is_same) {
                same_distances.push_back(transition.distance);
            } else {
                diff_distances.push_back(transition.distance);
            }
            std::cout << clips[i].path.filename().string() << ","
                      << clips[j].path.filename().string() << ","
                      << type << "," << transition.distance << "\n";
        }
    }

    auto print_summary = [](const char* name, std::vector<float> values) {
        if (values.empty()) {
            std::cout << name << ": (none)\n";
            return;
        }
        std::sort(values.begin(), values.end());
        std::cout << name << ": n=" << values.size()
                  << " min=" << values.front()
                  << " p10=" << Percentile(values, 0.10f)
                  << " p25=" << Percentile(values, 0.25f)
                  << " median=" << Percentile(values, 0.50f)
                  << " p75=" << Percentile(values, 0.75f)
                  << " max=" << values.back() << "\n";
    };

    std::cout << "\n=== distance distribution by pair type ===\n";
    print_summary("self       ", self_distances);
    print_summary("same-action", same_distances);
    print_summary("diff-action", diff_distances);

    std::vector<float> accept = self_distances;
    accept.insert(accept.end(), same_distances.begin(), same_distances.end());

    std::cout << "\n=== suggested thresholds (mean-squared world-unit space) ===\n";
    if (accept.empty()) {
        std::cout << "insufficient accept-class (self+same) samples; inspect histogram manually.\n";
    } else {
        const float tgood = Percentile(accept, 0.25f);
        float tbad = tgood;
        if (!diff_distances.empty()) {
            tbad = std::max(tgood, Percentile(diff_distances, 0.25f));
        } else {
            tbad = Percentile(accept, 0.75f);
        }
        std::cout << "TGOOD ~= p25(self+same) = " << tgood << "\n";
        std::cout << "TBAD  ~= max(TGOOD, p25(diff)) = " << tbad << "\n";
    }

    std::cout << "NOTE: action grouping is a filename heuristic (stem minus trailing variant letter and 'Loop').\n";
    std::cout << "NOTE: distances are mean-squared per point in the BVH's native units (no loader scaling).\n";
    std::cout << "NOTE: non-looping clips show large self/same distance over the late->early window; expected.\n";
    return 0;
}


struct GraphSpecBuildInputs {
    pmg::Skeleton skeleton;
    std::map<std::string, pmg::ParametricMotionSpace> spaces;
};

void PrintParameterVector(const pmg::ParameterVector& parameter) {
    std::cout << "[";
    for (std::size_t i = 0; i < parameter.size(); ++i) {
        if (i > 0) {
            std::cout << ", ";
        }
        std::cout << parameter[i];
    }
    std::cout << "]";
}

void PrintAabb(const pmg::ParameterAabb& box) {
    if (box.IsEmpty()) {
        std::cout << "<empty>";
        return;
    }
    std::cout << "min=";
    PrintParameterVector(box.min_corner);
    std::cout << " max=";
    PrintParameterVector(box.max_corner);
}

const pmg::GraphSpecNode& FindSpecNodeForCli(
    const pmg::GraphSpec& spec,
    const std::string& name) {
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        if (node.name == name) {
            return node;
        }
    }
    throw std::runtime_error("GraphSpec CLI: unknown node '" + name + "'");
}

GraphSpecBuildInputs LoadSpecInputsForCli(
    const pmg::GraphSpec& spec,
    float skeleton_offset_tolerance = 1.0e-4f) {
    if (spec.nodes.empty()) {
        throw std::runtime_error("GraphSpec CLI: spec contains no nodes");
    }

    GraphSpecBuildInputs inputs;
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        inputs.spaces.emplace(node.name,
                              pmg::ParametricMotionSpace(node.name, node.parameter_dimension));
    }

    std::optional<pmg::Skeleton> reference_skeleton;
    for (const pmg::GraphSpecExample& example : spec.examples) {
        const auto space_it = inputs.spaces.find(example.node_name);
        if (space_it == inputs.spaces.end()) {
            throw std::runtime_error("GraphSpec CLI: example references unknown node '" +
                                     example.node_name + "'");
        }

        const pmg::BvhData data = pmg::BvhLoader::Load(example.bvh_path);
        if (!reference_skeleton.has_value()) {
            reference_skeleton = data.skeleton;
        } else {
            pmg::RequireSkeletonCompatible(*reference_skeleton, data.skeleton,
                                           "GraphSpec CLI", skeleton_offset_tolerance);
        }
        space_it->second.AddExample(example.parameter, data.clip);
    }

    if (!reference_skeleton.has_value()) {
        throw std::runtime_error("GraphSpec CLI: spec contains no examples");
    }
    inputs.skeleton = *reference_skeleton;
    return inputs;
}

int ValidateGraphSpecCommand(const std::string& spec_path) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(spec_path);
    std::cout << "graph spec: " << spec_path << "\n";
    std::cout << "nodes: " << spec.nodes.size() << "\n";
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        int example_count = 0;
        for (const pmg::GraphSpecExample& example : spec.examples) {
            if (example.node_name == node.name) {
                ++example_count;
            }
        }
        std::cout << "  " << node.name << " dim=" << node.parameter_dimension
                  << " examples=" << example_count << "\n";
    }

    const GraphSpecBuildInputs inputs = LoadSpecInputsForCli(spec);
    std::cout << "skeleton: joints=" << inputs.skeleton.NumJoints()
              << " compatible=yes\n";
    for (const auto& item : inputs.spaces) {
        const pmg::ParametricMotionSpace& space = item.second;
        std::cout << "space " << item.first
                  << ": dim=" << space.ParameterDimension()
                  << " examples=" << space.NumExamples();
        if (space.NumExamples() > 0) {
            std::cout << " min=";
            PrintParameterVector(space.MinParameter());
            std::cout << " max=";
            PrintParameterVector(space.MaxParameter());
        }
        std::cout << "\n";
    }
    std::cout << "edges requested: " << spec.edges.size() << "\n";
    for (const pmg::GraphSpecEdge& edge : spec.edges) {
        (void)FindSpecNodeForCli(spec, edge.source_node);
        (void)FindSpecNodeForCli(spec, edge.target_node);
        std::cout << "  " << edge.source_node << " -> " << edge.target_node << "\n";
    }
    return 0;
}

void PrintEdgeBuildReport(const pmg::EdgeBuildReport& report) {
    std::cout << "edge_created=" << (report.edge_created ? "yes" : "no") << "\n";
    if (!report.edge_created) {
        std::cout << "reject_reason=" << report.reject_reason << "\n";
    }
    std::cout << "source sample reports: " << report.source_reports.size() << "\n";
    for (std::size_t i = 0; i < report.source_reports.size(); ++i) {
        const pmg::SourceSampleBuildReport& source = report.source_reports[i];
        std::cout << "  [" << i << "] source=";
        PrintParameterVector(source.source_parameter);
        std::cout << " accepted=" << (source.accepted ? "yes" : "no") << "\n";
        std::cout << "      D min=" << source.min_distance
                  << " p25=" << source.p25_distance
                  << " median=" << source.median_distance
                  << " max=" << source.max_distance << "\n";
        std::cout << "      counts GOOD=" << source.good_count
                  << " NEUTRAL=" << source.neutral_count
                  << " BAD=" << source.bad_count << "\n";
        std::cout << "      box before: ";
        PrintAabb(source.target_box_before_shrink);
        std::cout << "\n      box after : ";
        PrintAabb(source.target_box_after_shrink);
        std::cout << "\n";
        if (!source.accepted) {
            std::cout << "      reject_reason=" << source.reject_reason << "\n";
        }
    }
}

int DiagnoseGraphEdgeCommand(
    const std::string& spec_path,
    const std::string& source_node,
    const std::string& target_node,
    const pmg::PmgBuilderConfig& config) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(spec_path);
    const GraphSpecBuildInputs inputs = LoadSpecInputsForCli(spec);

    const auto source_it = inputs.spaces.find(source_node);
    const auto target_it = inputs.spaces.find(target_node);
    if (source_it == inputs.spaces.end()) {
        throw std::runtime_error("--diagnose-graph-edge: unknown source node '" + source_node + "'");
    }
    if (target_it == inputs.spaces.end()) {
        throw std::runtime_error("--diagnose-graph-edge: unknown target node '" + target_node + "'");
    }

    std::cout << "diagnose edge: " << source_node << " -> " << target_node << "\n";
    std::cout << "config: TGOOD=" << config.good_transition_threshold
              << " TBAD=" << config.bad_transition_threshold
              << " source_samples=" << config.source_sample_count
              << " target_samples=" << config.target_sample_count << "\n";

    const pmg::EdgeBuildResult result = pmg::PmgBuilder::BuildEdgeWithReport(
        inputs.skeleton, 0, 1, source_it->second, target_it->second, config);
    std::cout << "transition_samples=" << result.edge.samples.size() << "\n";
    PrintEdgeBuildReport(result.report);
    return result.edge.samples.empty() ? 1 : 0;
}

pmg::PmgBuilderConfig ParseBuilderConfigOptions(int argc, char** argv, int first_option_index) {
    pmg::PmgBuilderConfig config;
    for (int index = first_option_index; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--tgood") {
            config.good_transition_threshold = std::stof(require_value("--tgood"));
        } else if (option == "--tbad") {
            config.bad_transition_threshold = std::stof(require_value("--tbad"));
        } else if (option == "--source-samples") {
            config.source_sample_count = std::stoi(require_value("--source-samples"));
        } else if (option == "--target-samples") {
            config.target_sample_count = std::stoi(require_value("--target-samples"));
        } else if (option == "--seed") {
            config.seed = static_cast<unsigned int>(std::stoul(require_value("--seed")));
        } else {
            throw std::runtime_error("unknown builder option '" + option + "'");
        }
    }
    return config;
}

std::string JsonEscape(const std::string& text) {
    std::string escaped;
    for (const char character : text) {
        if (character == '\\' || character == '"') {
            escaped.push_back('\\');
        }
        escaped.push_back(character);
    }
    return escaped;
}

bool BoxesDiffer(const pmg::ParameterAabb& before,
                 const pmg::ParameterAabb& after) {
    return before.min_corner != after.min_corner ||
           before.max_corner != after.max_corner;
}

struct ArtifactRuntimeMetrics {
    int simulated_frames = 0;
    int transitions = 0;
    double runtime_frames_per_second = 0.0;
    float target_min_distance = std::numeric_limits<float>::quiet_NaN();
};

ArtifactRuntimeMetrics BenchmarkArtifact(
    const pmg::BuiltPmgArtifact& artifact) {
    ArtifactRuntimeMetrics metrics;
    pmg::PointCloudAlignment alignment(artifact.skeleton);
    pmg::RuntimeController controller(artifact.graph, alignment);
    const pmg::ParameterDomain domain =
        artifact.graph.Node(0).motion_space.Domain();
    pmg::ParameterVector start_parameter = domain.Bounds().min_corner;
    for (std::size_t dimension = 0; dimension < start_parameter.size(); ++dimension) {
        start_parameter[dimension] =
            0.5f * (domain.Bounds().min_corner[dimension] +
                    domain.Bounds().max_corner[dimension]);
    }
    controller.Start(0, start_parameter, artifact.metadata.frames_per_second);

    std::mt19937 random(12345u);
    pmg::RuntimeControlRequest request =
        pmg::ChooseRandomOutgoingTransition(
            artifact.graph, controller.CurrentNode(), random)
            .request;
    const float delta_seconds = 1.0f / artifact.metadata.frames_per_second;
    metrics.simulated_frames =
        static_cast<int>(10.0f * artifact.metadata.frames_per_second);
    int previous_transitions = 0;
    const auto runtime_start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < metrics.simulated_frames; ++frame) {
        controller.Update(delta_seconds, request);
        if (controller.CompletedTransitions() != previous_transitions) {
            previous_transitions = controller.CompletedTransitions();
            if (artifact.graph.OutgoingEdgeIndices(
                    controller.CurrentNode()).empty()) {
                metrics.simulated_frames = frame + 1;
                break;
            }
            request =
                pmg::ChooseRandomOutgoingTransition(
                    artifact.graph, controller.CurrentNode(), random)
                    .request;
        }
    }
    const double runtime_seconds =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - runtime_start)
            .count();
    metrics.transitions = controller.CompletedTransitions();
    metrics.runtime_frames_per_second =
        runtime_seconds > 0.0
            ? static_cast<double>(metrics.simulated_frames) / runtime_seconds
            : 0.0;

    try {
        pmg::GoalDirectedLocomotion steering(
            artifact.graph, artifact.skeleton, 0,
            artifact.metadata.frames_per_second);
        pmg::PointCloudAlignment goal_alignment(artifact.skeleton);
        pmg::RuntimeController goal_controller(
            artifact.graph, goal_alignment);
        goal_controller.Start(
            0, start_parameter, artifact.metadata.frames_per_second);
        pmg::GoalRequest goal;
        goal.target_position = {10.0f, 0.0f, 10.0f};
        metrics.target_min_distance =
            std::numeric_limits<float>::infinity();
        const int goal_frames =
            static_cast<int>(60.0f * artifact.metadata.frames_per_second);
        for (int frame = 0; frame < goal_frames; ++frame) {
            const pmg::Pose pose = goal_controller.CurrentPose();
            const float delta_x = goal.target_position.x - pose.root_position.x;
            const float delta_z = goal.target_position.z - pose.root_position.z;
            metrics.target_min_distance = std::min(
                metrics.target_min_distance,
                std::sqrt(delta_x * delta_x + delta_z * delta_z));
            goal_controller.Update(
                delta_seconds, steering.RequestForPose(pose, goal));
        }
    } catch (const std::exception&) {
        // Goal-directed control is only defined for a one-dimensional
        // steerable node with a usable self-transition.
    }
    return metrics;
}

void WriteArtifactReports(
    const pmg::BuiltPmgArtifact& artifact,
    const std::filesystem::path& artifact_path,
    double build_seconds) {
    const std::filesystem::path output_directory =
        artifact_path.has_parent_path() ? artifact_path.parent_path()
                                        : std::filesystem::current_path();
    const std::filesystem::path tables_directory =
        output_directory / "tables";
    std::filesystem::create_directories(tables_directory);
    const std::uintmax_t artifact_bytes =
        std::filesystem::file_size(artifact_path);

    int source_report_count = 0;
    int good_count = 0;
    int neutral_count = 0;
    int bad_count = 0;
    int shrunken_box_count = 0;
    for (const pmg::EdgeBuildMetadata& edge : artifact.metadata.edge_builds) {
        for (const pmg::SourceSampleBuildReport& source :
             edge.report.source_reports) {
            ++source_report_count;
            good_count += source.good_count;
            neutral_count += source.neutral_count;
            bad_count += source.bad_count;
            if (BoxesDiffer(
                    source.target_box_before_shrink,
                    source.target_box_after_shrink)) {
                ++shrunken_box_count;
            }
        }
    }
    const ArtifactRuntimeMetrics runtime = BenchmarkArtifact(artifact);

    {
        std::ofstream config(output_directory / "config.json");
        config << "{\n"
               << "  \"format\": \"PMG_GRAPH_V5\",\n"
               << "  \"units\": \"" << JsonEscape(artifact.metadata.units) << "\",\n"
               << "  \"generated_frame_count\": "
               << artifact.metadata.generated_frame_count << ",\n"
               << "  \"frames_per_second\": "
               << artifact.metadata.frames_per_second << ",\n"
               << "  \"source_bvh_paths\": [";
        for (std::size_t index = 0;
             index < artifact.metadata.source_bvh_paths.size(); ++index) {
            if (index > 0) {
                config << ", ";
            }
            config << '"' << JsonEscape(
                artifact.metadata.source_bvh_paths[index]) << '"';
        }
        config << "],\n  \"registrations\": [\n";
        for (std::size_t index = 0;
             index < artifact.metadata.node_registrations.size(); ++index) {
            const pmg::NodeRegistrationMetadata& registration =
                artifact.metadata.node_registrations[index];
            config << "    {\"node\": \"" << JsonEscape(registration.node_name)
                   << "\", \"cycle_joint\": \""
                   << JsonEscape(registration.cycle_joint)
                   << "\", \"contact_joints\": [";
            for (std::size_t joint = 0;
                 joint < registration.contact_joints.size(); ++joint) {
                if (joint > 0) {
                    config << ", ";
                }
                config << '"' << JsonEscape(
                    registration.contact_joints[joint]) << '"';
            }
            config << "], \"min_contact_frames\": "
                   << registration.min_contact_frames
                   << ", \"dtw_refine\": "
                   << (registration.dtw_refine ? "true" : "false") << "}";
            config << (index + 1 ==
                               artifact.metadata.node_registrations.size()
                           ? "\n"
                           : ",\n");
        }
        config << "  ],\n  \"edges\": [\n";
        for (std::size_t index = 0;
             index < artifact.metadata.edge_builds.size(); ++index) {
            const pmg::EdgeBuildMetadata& edge =
                artifact.metadata.edge_builds[index];
            config << "    {\"source\": \"" << JsonEscape(edge.source_node)
                   << "\", \"target\": \"" << JsonEscape(edge.target_node)
                   << "\", \"tgood\": "
                   << edge.config.good_transition_threshold
                   << ", \"tbad\": " << edge.config.bad_transition_threshold
                   << ", \"source_samples\": "
                   << edge.config.source_sample_count
                   << ", \"target_samples\": "
                   << edge.config.target_sample_count
                   << ", \"seed\": " << edge.config.seed << "}";
            config << (index + 1 == artifact.metadata.edge_builds.size()
                           ? "\n"
                           : ",\n");
        }
        config << "  ]\n}\n";
    }
    {
        std::ofstream metrics_file(output_directory / "metrics.json");
        metrics_file << "{\n"
                     << "  \"build_seconds\": " << build_seconds << ",\n"
                     << "  \"artifact_bytes\": " << artifact_bytes << ",\n"
                     << "  \"nodes\": " << artifact.graph.NumNodes() << ",\n"
                     << "  \"edges\": " << artifact.graph.NumEdges() << ",\n"
                     << "  \"source_reports\": " << source_report_count << ",\n"
                     << "  \"good_samples\": " << good_count << ",\n"
                     << "  \"neutral_samples\": " << neutral_count << ",\n"
                     << "  \"bad_samples\": " << bad_count << ",\n"
                     << "  \"shrunken_boxes\": " << shrunken_box_count << ",\n"
                     << "  \"runtime_simulated_frames\": "
                     << runtime.simulated_frames << ",\n"
                     << "  \"runtime_transitions\": " << runtime.transitions << ",\n"
                     << "  \"runtime_frames_per_second\": "
                     << runtime.runtime_frames_per_second << ",\n"
                     << "  \"target_10_10_min_distance\": ";
        if (std::isfinite(runtime.target_min_distance)) {
            metrics_file << runtime.target_min_distance;
        } else {
            metrics_file << "null";
        }
        metrics_file << "\n}\n";
    }
    {
        std::ofstream table(tables_directory / "edge_samples.csv");
        table << "edge,source_parameter,good,neutral,bad,min_distance,"
                 "median_distance,box_shrunk,accepted,reject_reason\n";
        for (const pmg::EdgeBuildMetadata& edge :
             artifact.metadata.edge_builds) {
            for (const pmg::SourceSampleBuildReport& source :
                 edge.report.source_reports) {
                table << edge.source_node << "->" << edge.target_node << ",\"";
                for (std::size_t dimension = 0;
                     dimension < source.source_parameter.size(); ++dimension) {
                    if (dimension > 0) {
                        table << ' ';
                    }
                    table << source.source_parameter[dimension];
                }
                table << "\"," << source.good_count << ','
                      << source.neutral_count << ',' << source.bad_count << ','
                      << source.min_distance << ',' << source.median_distance
                      << ',' << static_cast<int>(BoxesDiffer(
                                     source.target_box_before_shrink,
                                     source.target_box_after_shrink))
                      << ',' << static_cast<int>(source.accepted) << ",\""
                      << source.reject_reason << "\"\n";
            }
        }
    }
    {
        std::ofstream report(output_directory / "report.md");
        report << "# PMG Paper-Core Build Report\n\n"
               << "## Purpose\n\n"
               << "Offline-built PMG artifact validated through the same "
                  "complete artifact used by online runtime playback.\n\n"
               << "## Inputs\n\n"
               << "- Source BVHs: " << artifact.metadata.source_bvh_paths.size()
               << "\n- Units: " << artifact.metadata.units
               << "\n- Seed/config: `config.json`\n\n"
               << "## Outputs\n\n"
               << "- Artifact: `" << artifact_path.filename().string()
               << "` (" << artifact_bytes << " bytes)\n"
               << "- Edge table: `tables/edge_samples.csv`\n\n"
               << "## Metrics\n\n"
               << "- Build time: " << build_seconds << " s\n"
               << "- Nodes / edges: " << artifact.graph.NumNodes() << " / "
               << artifact.graph.NumEdges() << "\n"
               << "- GOOD / NEUTRAL / BAD: " << good_count << " / "
               << neutral_count << " / " << bad_count << "\n"
               << "- Shrunken target boxes: " << shrunken_box_count << "\n"
               << "- Runtime transitions (10 s simulated): "
               << runtime.transitions << "\n"
               << "- Runtime throughput: "
               << runtime.runtime_frames_per_second << " frames/s\n"
               << "- Goal (10,10) minimum distance: "
               << (std::isfinite(runtime.target_min_distance)
                       ? std::to_string(runtime.target_min_distance)
                       : std::string("not applicable"))
               << "\n\n"
               << "## Limitations\n\n"
               << "- This validates renderer-conditioned PMG behavior on the "
                  "included BVH corpus; it does not reproduce the paper's "
                  "boxing/platform datasets.\n"
               << "- Foot locking is an optional post-process and is not part "
                  "of the PMG artifact runtime contract.\n";
    }
}


int BuildGraphCommand(const std::string& spec_path,
                      const std::string& output_path,
                      const pmg::PmgBuilderConfig& config) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(spec_path);
    pmg::ArtifactBuildConfig artifact_config;
    artifact_config.default_edge_config = config;
    const auto build_start = std::chrono::steady_clock::now();
    const pmg::BuiltPmgArtifact artifact =
        pmg::BuildPmgArtifactFromSpec(spec, artifact_config);
    const double build_seconds =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - build_start)
            .count();
    const std::filesystem::path artifact_path(output_path);
    if (artifact_path.has_parent_path()) {
        std::filesystem::create_directories(artifact_path.parent_path());
    }
    pmg::SavePmgArtifactText(artifact, output_path);
    WriteArtifactReports(artifact, artifact_path, build_seconds);
    std::cout << "wrote graph: " << output_path << "\n";
    std::cout << "format=PMG_GRAPH_V5\n";
    std::cout << "nodes=" << artifact.graph.NumNodes()
              << " edges=" << artifact.graph.NumEdges() << "\n";
    std::cout << "skeleton_joints=" << artifact.skeleton.NumJoints() << "\n";
    std::cout << "generated_frame_count="
              << artifact.metadata.generated_frame_count << "\n";
    std::cout << "frames_per_second="
              << artifact.metadata.frames_per_second << "\n";
    return 0;
}

int InspectGraphCommand(const std::string& graph_path) {
    const pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(graph_path);
    const pmg::ParametricMotionGraph& graph = artifact.graph;
    std::cout << "graph: " << graph_path << "\n";
    std::cout << "skeleton_joints: " << artifact.skeleton.NumJoints() << "\n";
    std::cout << "runtime: frames=" << artifact.metadata.generated_frame_count
              << " fps=" << artifact.metadata.frames_per_second << "\n";
    std::cout << "nodes: " << graph.NumNodes() << "\n";
    for (int node_index = 0; node_index < graph.NumNodes(); ++node_index) {
        const pmg::PmgNode& node = graph.Node(node_index);
        std::cout << "  [" << node_index << "] " << node.name
                  << " dim=" << node.motion_space.ParameterDimension()
                  << " examples=" << node.motion_space.NumExamples() << "\n";
    }
    std::cout << "edges: " << graph.NumEdges() << "\n";
    for (int edge_index = 0; edge_index < graph.NumEdges(); ++edge_index) {
        const pmg::PmgEdge& edge = graph.Edge(edge_index);
        std::cout << "  [" << edge_index << "] "
                  << edge.source_node << " -> " << edge.target_node
                  << " samples=" << edge.samples.size() << "\n";
    }
    return 0;
}

std::vector<std::string> SplitCommaList(const std::string& text) {
    std::vector<std::string> items;
    std::istringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
        if (!item.empty()) {
            items.push_back(item);
        }
    }
    return items;
}

std::vector<int> ResolveJointList(
    const pmg::Skeleton& skeleton,
    const std::string& comma_names) {
    std::vector<int> indices;
    for (const std::string& name : SplitCommaList(comma_names)) {
        const std::optional<int> index = FindJointExact(skeleton, name);
        if (!index) {
            throw std::runtime_error("unknown joint '" + name + "'");
        }
        indices.push_back(*index);
    }
    if (indices.empty()) {
        throw std::runtime_error("no contact joints given");
    }
    return indices;
}

// Print detected contact intervals and the anchor phases registration would
// use. Diagnostic for picking contact joints and verifying that clips meant
// to share a motion space expose the same contact structure.
int InspectContacts(const std::string& path, const std::string& joints_csv) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);
    const std::vector<int> joints = ResolveJointList(data.skeleton, joints_csv);

    const pmg::ContactDetectionSettings settings =
        pmg::EstimateContactSettings(data.skeleton, data.clip, joints);
    const std::vector<pmg::ContactInterval> intervals =
        pmg::DetectContacts(data.skeleton, data.clip, joints, settings);

    std::cout << "bvh=" << path << "\n";
    std::cout << "frames=" << data.clip.NumFrames()
              << " fps=" << data.clip.frames_per_second << "\n";
    std::cout << "height_threshold=" << settings.height_threshold
              << " speed_threshold=" << settings.speed_threshold << "\n";
    std::cout << "contacts=" << intervals.size() << "\n";
    for (const pmg::ContactInterval& interval : intervals) {
        std::cout << "  joint=" << data.skeleton.joints[interval.joint_index].name
                  << " frames=[" << interval.first_frame << ", " << interval.last_frame << "]"
                  << " strike_phase=" << interval.StrikePhase(data.clip.NumFrames())
                  << " lift_phase=" << interval.LiftPhase(data.clip.NumFrames())
                  << "\n";
    }

    const std::vector<float> anchors =
        pmg::ContactAnchorPhases(intervals, data.clip.NumFrames());
    std::cout << "anchor_count=" << anchors.size() << "\n";
    std::cout << "anchors=";
    for (std::size_t i = 0; i < anchors.size(); ++i) {
        if (i > 0) {
            std::cout << ",";
        }
        std::cout << anchors[i];
    }
    std::cout << "\n";
    return 0;
}

struct SpaceSweepOptions {
    std::string spec_path;
    std::string node_name;
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    std::string cycle_joint;  // empty = use example clips as authored
    int sweep_steps = 11;
    // Floor for contact run length; real mocap shows spurious 1-2 frame
    // grazes that would break anchor-structure matching across examples.
    int min_contact_frames = 3;
    // Assert thresholds; negative = report only.
    int min_contacts = -1;
    float max_foot_slide = -1.0f;
    float max_adjacent_step = -1.0f;
    bool assert_no_regression = false;  // registered must not be worse than naive
    // Also measure a DTW-refined registration and gate it against the
    // contact-anchor registration the same way.
    bool dtw_refine = false;
    // Also measure IK foot locking applied to the best registered variant's
    // generated clips and require it to cut the residual slide.
    bool foot_lock = false;
};

struct SweepMetrics {
    int min_contacts = 0;
    // Total planted frames in the worst sweep step. Degraded blends lift the
    // foot during what should be stance, so low coverage = lost contacts.
    int min_contact_coverage = 0;
    float max_foot_slide = 0.0f;
    // Slide per second of contact. Comparable across blends whose detected
    // stance lengths differ (longer honest stance accumulates more absolute
    // drift than a short smeared one).
    float max_slide_rate = 0.0f;
    float max_adjacent_step = 0.0f;
};

// Horizontal drift of a joint while "planted": max distance from its position
// at the strike frame, over the contact interval. The numeric form of the
// foot-slide artifact.
float ContactSlide(
    const pmg::Skeleton& skeleton,
    const pmg::MotionClip& clip,
    const pmg::ContactInterval& interval) {
    const pmg::Vec3 strike_position =
        pmg::ComputeJointWorldPositions(skeleton, clip.frames[interval.first_frame])
            [interval.joint_index];
    float max_drift = 0.0f;
    for (int frame = interval.first_frame; frame <= interval.last_frame; ++frame) {
        const pmg::Vec3 position =
            pmg::ComputeJointWorldPositions(skeleton, clip.frames[frame])[interval.joint_index];
        const pmg::Vec3 delta = position - strike_position;
        max_drift = std::max(max_drift, HorizontalLength(delta));
    }
    return max_drift;
}

// Mean joint world distance between two equally-sized poses.
float MeanJointDistance(
    const pmg::Skeleton& skeleton,
    const pmg::Pose& first,
    const pmg::Pose& second) {
    const std::vector<pmg::Vec3> first_positions =
        pmg::ComputeJointWorldPositions(skeleton, first);
    const std::vector<pmg::Vec3> second_positions =
        pmg::ComputeJointWorldPositions(skeleton, second);
    float sum = 0.0f;
    for (std::size_t i = 0; i < first_positions.size(); ++i) {
        sum += (first_positions[i] - second_positions[i]).Norm();
    }
    return sum / static_cast<float>(first_positions.size());
}

// Sweep the first parameter dimension across the space's domain (other
// dimensions pinned at the domain midpoint), generate a clip per step, and
// measure: contacts found per step, worst foot slide inside any contact, and
// the worst frame-wise mean joint distance between adjacent steps.
SweepMetrics MeasureSpaceSweep(
    const pmg::ParametricMotionSpace& space,
    const pmg::Skeleton& skeleton,
    const std::vector<int>& contact_joints,
    const pmg::ContactDetectionSettings& settings,
    int sweep_steps,
    int generated_frame_count,
    float frames_per_second,
    const char* label,
    bool foot_lock = false) {
    const std::vector<float> min_parameter = space.MinParameter();
    const std::vector<float> max_parameter = space.MaxParameter();

    SweepMetrics metrics;
    metrics.min_contacts = std::numeric_limits<int>::max();
    metrics.min_contact_coverage = std::numeric_limits<int>::max();

    std::vector<pmg::MotionClip> generated;
    generated.reserve(static_cast<std::size_t>(sweep_steps));

    std::cout << "--- sweep " << label << " ---\n";
    std::cout << "step,parameter,contacts,contact_frames,foot_slide,slide_rate\n";
    for (int step = 0; step < sweep_steps; ++step) {
        const float alpha =
            sweep_steps == 1 ? 0.0f
                             : static_cast<float>(step) / static_cast<float>(sweep_steps - 1);
        pmg::ParameterVector parameter(min_parameter.size());
        for (std::size_t dim = 0; dim < parameter.size(); ++dim) {
            const float mid = 0.5f * (min_parameter[dim] + max_parameter[dim]);
            parameter[dim] =
                dim == 0 ? min_parameter[0] + alpha * (max_parameter[0] - min_parameter[0])
                         : mid;
        }

        pmg::MotionClip clip =
            space.GenerateClip(parameter, generated_frame_count, frames_per_second);
        if (foot_lock) {
            pmg::FootLockSettings lock_settings;
            lock_settings.contacts = settings;
            pmg::LockFootContacts(skeleton, clip, contact_joints, lock_settings);
        }

        const std::vector<pmg::ContactInterval> intervals =
            pmg::DetectContacts(skeleton, clip, contact_joints, settings);
        float step_slide = 0.0f;
        float step_slide_rate = 0.0f;
        int step_coverage = 0;
        for (const pmg::ContactInterval& interval : intervals) {
            const float slide = ContactSlide(skeleton, clip, interval);
            const int contact_frames = interval.last_frame - interval.first_frame + 1;
            const float contact_seconds =
                static_cast<float>(contact_frames) / frames_per_second;
            step_slide = std::max(step_slide, slide);
            step_slide_rate = std::max(step_slide_rate, slide / contact_seconds);
            step_coverage += contact_frames;
        }

        metrics.min_contacts = std::min(metrics.min_contacts, static_cast<int>(intervals.size()));
        metrics.min_contact_coverage = std::min(metrics.min_contact_coverage, step_coverage);
        metrics.max_foot_slide = std::max(metrics.max_foot_slide, step_slide);
        metrics.max_slide_rate = std::max(metrics.max_slide_rate, step_slide_rate);

        std::cout << step << "," << parameter[0] << "," << intervals.size() << ","
                  << step_coverage << "," << step_slide << "," << step_slide_rate << "\n";
        generated.push_back(clip);
    }

    for (std::size_t step = 0; step + 1 < generated.size(); ++step) {
        for (int frame = 0; frame < generated_frame_count; ++frame) {
            const float distance = MeanJointDistance(
                skeleton, generated[step].frames[frame], generated[step + 1].frames[frame]);
            metrics.max_adjacent_step = std::max(metrics.max_adjacent_step, distance);
        }
    }

    std::cout << label << "_min_contacts=" << metrics.min_contacts << "\n";
    std::cout << label << "_min_contact_coverage=" << metrics.min_contact_coverage << "\n";
    std::cout << label << "_max_foot_slide=" << metrics.max_foot_slide << "\n";
    std::cout << label << "_max_slide_rate=" << metrics.max_slide_rate << "\n";
    std::cout << label << "_max_adjacent_step=" << metrics.max_adjacent_step << "\n";
    return metrics;
}

// Build one node's motion space from a graph spec, optionally normalizing
// every example to its first gait cycle.
pmg::ParametricMotionSpace BuildSpaceForSweep(
    const pmg::GraphSpec& spec,
    const std::string& node_name,
    const std::string& cycle_joint,
    pmg::Skeleton& skeleton_out) {
    const pmg::GraphSpecNode& node = FindSpecNodeForCli(spec, node_name);
    pmg::ParametricMotionSpace space(node.name, node.parameter_dimension);

    std::optional<pmg::Skeleton> reference_skeleton;
    for (const pmg::GraphSpecExample& example : spec.examples) {
        if (example.node_name != node_name) {
            continue;
        }
        const pmg::BvhData data = pmg::BvhLoader::Load(example.bvh_path);
        if (!reference_skeleton.has_value()) {
            reference_skeleton = data.skeleton;
        } else {
            pmg::RequireSkeletonCompatible(*reference_skeleton, data.skeleton,
                                           "--space-sweep", 1.0e-4f);
        }

        pmg::MotionClip clip = data.clip;
        if (!cycle_joint.empty()) {
            const std::optional<int> joint = FindJointExact(data.skeleton, cycle_joint);
            if (!joint) {
                throw std::runtime_error("--space-sweep: unknown cycle joint '" + cycle_joint + "'");
            }
            const pmg::ContactDetectionSettings cycle_settings =
                pmg::EstimateContactSettings(data.skeleton, data.clip, {*joint});
            clip = pmg::ExtractFirstCycle(data.skeleton, data.clip, *joint, cycle_settings);
            std::cout << "cycle " << example.bvh_path << ": frames " << data.clip.NumFrames()
                      << " -> " << clip.NumFrames() << "\n";
        }
        space.AddExample(example.parameter, std::move(clip));
    }

    if (space.NumExamples() == 0) {
        throw std::runtime_error("--space-sweep: node '" + node_name + "' has no examples");
    }
    skeleton_out = *reference_skeleton;
    return space;
}

// Phase 2 diagnostic: numerically verify that a parametric motion space
// produces smooth, contact-preserving motion across its parameter range, and
// that contact registration does not regress the naive blend. Exit code is
// nonzero when an assert threshold is violated.
int SpaceSweepCommand(const SpaceSweepOptions& options) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(options.spec_path);

    pmg::Skeleton skeleton;
    pmg::ParametricMotionSpace naive_space =
        BuildSpaceForSweep(spec, options.node_name, options.cycle_joint, skeleton);

    const std::vector<int> contact_joints =
        ResolveJointList(skeleton, options.contact_joints_csv);
    pmg::ContactDetectionSettings settings = pmg::EstimateContactSettings(
        skeleton, naive_space.Examples().front().clip, contact_joints);
    settings.min_contact_frames = options.min_contact_frames;
    std::cout << "contact settings: height_threshold=" << settings.height_threshold
              << " speed_threshold=" << settings.speed_threshold
              << " min_contact_frames=" << settings.min_contact_frames << "\n";

    for (const pmg::ExampleMotion& example : naive_space.Examples()) {
        const std::vector<pmg::ContactInterval> intervals =
            pmg::DetectContacts(skeleton, example.clip, contact_joints, settings);
        const std::vector<float> anchors =
            pmg::ContactAnchorPhases(intervals, example.clip.NumFrames());
        std::cout << "example " << example.clip.name
                  << ": frames=" << example.clip.NumFrames()
                  << " contacts=" << intervals.size()
                  << " anchors=" << anchors.size() << "\n";
    }

    const int generated_frame_count = naive_space.Examples().front().clip.NumFrames();
    const float frames_per_second = naive_space.Examples().front().clip.frames_per_second;

    pmg::ParametricMotionSpace registered_space = naive_space;
    pmg::RegisterSpaceByContacts(registered_space, skeleton, contact_joints, settings);
    std::cout << "registered=yes\n";

    const SweepMetrics naive = MeasureSpaceSweep(
        naive_space, skeleton, contact_joints, settings, options.sweep_steps,
        generated_frame_count, frames_per_second, "naive");
    const SweepMetrics registered = MeasureSpaceSweep(
        registered_space, skeleton, contact_joints, settings, options.sweep_steps,
        generated_frame_count, frames_per_second, "registered");

    std::optional<SweepMetrics> dtw;
    pmg::ParametricMotionSpace best_space = registered_space;
    if (options.dtw_refine) {
        pmg::RefineRegistrationByDtw(best_space, skeleton, {});
        std::cout << "dtw_refined=yes\n";
        dtw = MeasureSpaceSweep(
            best_space, skeleton, contact_joints, settings, options.sweep_steps,
            generated_frame_count, frames_per_second, "dtw");
    }

    // IK foot locking post-processes the best registered variant's clips.
    std::optional<SweepMetrics> locked;
    if (options.foot_lock) {
        std::cout << "foot_lock=yes\n";
        locked = MeasureSpaceSweep(
            best_space, skeleton, contact_joints, settings, options.sweep_steps,
            generated_frame_count, frames_per_second, "locked", /*foot_lock=*/true);
    }

    bool failed = false;
    auto fail_if = [&failed](bool condition, const std::string& message) {
        if (condition) {
            std::cout << "ASSERT FAIL: " << message << "\n";
            failed = true;
        }
    };

    if (options.min_contacts >= 0) {
        fail_if(registered.min_contacts < options.min_contacts,
                "registered_min_contacts=" + std::to_string(registered.min_contacts) +
                    " < " + std::to_string(options.min_contacts));
    }
    if (options.max_foot_slide >= 0.0f) {
        fail_if(registered.max_foot_slide > options.max_foot_slide,
                "registered_max_foot_slide=" + std::to_string(registered.max_foot_slide) +
                    " > " + std::to_string(options.max_foot_slide));
    }
    if (options.max_adjacent_step >= 0.0f) {
        fail_if(registered.max_adjacent_step > options.max_adjacent_step,
                "registered_max_adjacent_step=" + std::to_string(registered.max_adjacent_step) +
                    " > " + std::to_string(options.max_adjacent_step));
    }
    if (options.assert_no_regression) {
        fail_if(registered.min_contacts < naive.min_contacts,
                "registration lost contacts: " + std::to_string(registered.min_contacts) +
                    " < " + std::to_string(naive.min_contacts));
        fail_if(registered.min_contact_coverage < naive.min_contact_coverage,
                "registration lost contact coverage: " +
                    std::to_string(registered.min_contact_coverage) + " < " +
                    std::to_string(naive.min_contact_coverage));
        fail_if(registered.max_slide_rate > naive.max_slide_rate * 1.05f + 1.0e-4f,
                "registration increased slide rate: " +
                    std::to_string(registered.max_slide_rate) + " > " +
                    std::to_string(naive.max_slide_rate));
    }
    if (dtw.has_value()) {
        if (options.min_contacts >= 0) {
            fail_if(dtw->min_contacts < options.min_contacts,
                    "dtw_min_contacts=" + std::to_string(dtw->min_contacts) + " < " +
                        std::to_string(options.min_contacts));
        }
        if (options.max_foot_slide >= 0.0f) {
            fail_if(dtw->max_foot_slide > options.max_foot_slide,
                    "dtw_max_foot_slide=" + std::to_string(dtw->max_foot_slide) + " > " +
                        std::to_string(options.max_foot_slide));
        }
        if (options.max_adjacent_step >= 0.0f) {
            fail_if(dtw->max_adjacent_step > options.max_adjacent_step,
                    "dtw_max_adjacent_step=" + std::to_string(dtw->max_adjacent_step) +
                        " > " + std::to_string(options.max_adjacent_step));
        }
        if (options.assert_no_regression) {
            fail_if(dtw->min_contacts < registered.min_contacts,
                    "dtw refinement lost contacts: " + std::to_string(dtw->min_contacts) +
                        " < " + std::to_string(registered.min_contacts));
            fail_if(dtw->min_contact_coverage < registered.min_contact_coverage,
                    "dtw refinement lost contact coverage: " +
                        std::to_string(dtw->min_contact_coverage) + " < " +
                        std::to_string(registered.min_contact_coverage));
            fail_if(dtw->max_slide_rate > registered.max_slide_rate * 1.05f + 1.0e-4f,
                    "dtw refinement increased slide rate: " +
                        std::to_string(dtw->max_slide_rate) + " > " +
                        std::to_string(registered.max_slide_rate));
        }
    }
    if (locked.has_value()) {
        const SweepMetrics& baseline = dtw.has_value() ? *dtw : registered;
        if (options.min_contacts >= 0) {
            fail_if(locked->min_contacts < options.min_contacts,
                    "locked_min_contacts=" + std::to_string(locked->min_contacts) + " < " +
                        std::to_string(options.min_contacts));
        }
        if (options.max_adjacent_step >= 0.0f) {
            fail_if(locked->max_adjacent_step > options.max_adjacent_step,
                    "locked_max_adjacent_step=" + std::to_string(locked->max_adjacent_step) +
                        " > " + std::to_string(options.max_adjacent_step));
        }
        if (options.assert_no_regression) {
            fail_if(locked->min_contacts < baseline.min_contacts,
                    "foot lock lost contacts: " + std::to_string(locked->min_contacts) +
                        " < " + std::to_string(baseline.min_contacts));
            fail_if(locked->min_contact_coverage < baseline.min_contact_coverage,
                    "foot lock lost contact coverage: " +
                        std::to_string(locked->min_contact_coverage) + " < " +
                        std::to_string(baseline.min_contact_coverage));
            // Locking exists to cut residual slide; require a real reduction,
            // not parity (measured 0.41 -> 0.20 on the walk corpus).
            fail_if(locked->max_slide_rate > baseline.max_slide_rate * 0.6f,
                    "foot lock did not cut slide rate by 40%: " +
                        std::to_string(locked->max_slide_rate) + " > 0.6 * " +
                        std::to_string(baseline.max_slide_rate));
        }
    }

    std::cout << (failed ? "RESULT=FAIL" : "RESULT=PASS") << "\n";
    return failed ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Phase 3 diagnostic: rebuild PMG edges on registered motion spaces and check,
// numerically, that registration does not degrade edge quality (and report
// GOOD fractions / boxes / distances so threshold drift is visible).
// ---------------------------------------------------------------------------

struct ValidateGraphOptions {
    std::string spec_path;
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    std::string cycle_joint;
    int min_contact_frames = 3;
    pmg::PmgBuilderConfig builder;
    // Assert thresholds on the registered build; negative = report only.
    int min_edge_samples = -1;
    float min_good_fraction = -1.0f;
    bool assert_no_regression = false;
};

struct EdgeQuality {
    bool created = false;
    int samples = 0;
    float mean_good_fraction = 0.0f;
    float mean_min_distance = 0.0f;
    float mean_p25_distance = 0.0f;
    float mean_median_distance = 0.0f;
    float mean_box_volume_fraction = 0.0f;
};

EdgeQuality MeasureEdgeQuality(
    const pmg::EdgeBuildResult& result,
    const pmg::ParametricMotionSpace& target_space) {
    EdgeQuality quality;
    quality.created = result.report.edge_created;
    quality.samples = static_cast<int>(result.edge.samples.size());

    float good_fraction_sum = 0.0f;
    float min_distance_sum = 0.0f;
    float p25_distance_sum = 0.0f;
    float median_distance_sum = 0.0f;
    int report_count = 0;
    for (const pmg::SourceSampleBuildReport& report : result.report.source_reports) {
        const int total = report.good_count + report.neutral_count + report.bad_count;
        if (total > 0) {
            good_fraction_sum += static_cast<float>(report.good_count) / static_cast<float>(total);
        }
        min_distance_sum += report.min_distance;
        p25_distance_sum += report.p25_distance;
        median_distance_sum += report.median_distance;
        ++report_count;
    }
    if (report_count > 0) {
        quality.mean_good_fraction = good_fraction_sum / static_cast<float>(report_count);
        quality.mean_min_distance = min_distance_sum / static_cast<float>(report_count);
        quality.mean_p25_distance = p25_distance_sum / static_cast<float>(report_count);
        quality.mean_median_distance = median_distance_sum / static_cast<float>(report_count);
    }

    const std::vector<float> domain_min = target_space.MinParameter();
    const std::vector<float> domain_max = target_space.MaxParameter();
    float volume_sum = 0.0f;
    for (const pmg::TransitionSample& sample : result.edge.samples) {
        float volume = 1.0f;
        for (std::size_t dim = 0; dim < domain_min.size(); ++dim) {
            const float domain_extent = std::max(domain_max[dim] - domain_min[dim], 1.0e-6f);
            const float box_extent =
                sample.target_parameter_box.max_corner[dim] -
                sample.target_parameter_box.min_corner[dim];
            volume *= std::clamp(box_extent / domain_extent, 0.0f, 1.0f);
        }
        volume_sum += volume;
    }
    if (quality.samples > 0) {
        quality.mean_box_volume_fraction = volume_sum / static_cast<float>(quality.samples);
    }
    return quality;
}

void PrintEdgeQuality(const char* label, const EdgeQuality& quality) {
    std::cout << label << "_edge_created=" << (quality.created ? 1 : 0) << "\n";
    std::cout << label << "_samples=" << quality.samples << "\n";
    std::cout << label << "_mean_good_fraction=" << quality.mean_good_fraction << "\n";
    std::cout << label << "_mean_min_distance=" << quality.mean_min_distance << "\n";
    std::cout << label << "_mean_p25_distance=" << quality.mean_p25_distance << "\n";
    std::cout << label << "_mean_median_distance=" << quality.mean_median_distance << "\n";
    std::cout << label << "_mean_box_volume_fraction=" << quality.mean_box_volume_fraction << "\n";
}

int ValidateGraphCommand(const ValidateGraphOptions& options) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(options.spec_path);

    // Build every node's space twice: as-authored (naive) and registered.
    std::map<std::string, pmg::ParametricMotionSpace> naive_spaces;
    std::map<std::string, pmg::ParametricMotionSpace> registered_spaces;
    pmg::Skeleton skeleton;
    for (const pmg::GraphSpecNode& node : spec.nodes) {
        pmg::ParametricMotionSpace space =
            BuildSpaceForSweep(spec, node.name, options.cycle_joint, skeleton);

        const std::vector<int> contact_joints =
            ResolveJointList(skeleton, options.contact_joints_csv);
        pmg::ContactDetectionSettings settings = pmg::EstimateContactSettings(
            skeleton, space.Examples().front().clip, contact_joints);
        settings.min_contact_frames = options.min_contact_frames;

        pmg::ParametricMotionSpace registered = space;
        pmg::RegisterSpaceByContacts(registered, skeleton, contact_joints, settings);
        pmg::RefineRegistrationByDtw(registered, skeleton, {});

        naive_spaces.emplace(node.name, std::move(space));
        registered_spaces.emplace(node.name, std::move(registered));
    }

    std::cout << "builder: TGOOD=" << options.builder.good_transition_threshold
              << " TBAD=" << options.builder.bad_transition_threshold
              << " source_samples=" << options.builder.source_sample_count
              << " target_samples=" << options.builder.target_sample_count
              << " seed=" << options.builder.seed << "\n";

    bool failed = false;
    auto fail_if = [&failed](bool condition, const std::string& message) {
        if (condition) {
            std::cout << "ASSERT FAIL: " << message << "\n";
            failed = true;
        }
    };

    for (const pmg::GraphSpecEdge& edge : spec.edges) {
        std::cout << "=== edge " << edge.source_node << " -> " << edge.target_node << " ===\n";

        const pmg::EdgeBuildResult naive_result = pmg::PmgBuilder::BuildEdgeWithReport(
            skeleton, 0, 0, naive_spaces.at(edge.source_node),
            naive_spaces.at(edge.target_node), options.builder);
        const pmg::EdgeBuildResult registered_result = pmg::PmgBuilder::BuildEdgeWithReport(
            skeleton, 0, 0, registered_spaces.at(edge.source_node),
            registered_spaces.at(edge.target_node), options.builder);

        const EdgeQuality naive = MeasureEdgeQuality(
            naive_result, naive_spaces.at(edge.target_node));
        const EdgeQuality registered = MeasureEdgeQuality(
            registered_result, registered_spaces.at(edge.target_node));

        PrintEdgeQuality("naive", naive);
        PrintEdgeQuality("registered", registered);
        if (!registered_result.report.edge_created) {
            std::cout << "registered_reject_reason=" << registered_result.report.reject_reason
                      << "\n";
        }

        fail_if(!registered.created, "registered edge not created");
        if (options.min_edge_samples >= 0) {
            fail_if(registered.samples < options.min_edge_samples,
                    "registered_samples=" + std::to_string(registered.samples) + " < " +
                        std::to_string(options.min_edge_samples));
        }
        if (options.min_good_fraction >= 0.0f) {
            fail_if(registered.mean_good_fraction < options.min_good_fraction,
                    "registered_mean_good_fraction=" +
                        std::to_string(registered.mean_good_fraction) + " < " +
                        std::to_string(options.min_good_fraction));
        }
        // Distribution comparisons are only meaningful when both builds
        // covered the same source samples (a rejected build aborts early).
        if (options.assert_no_regression) {
            fail_if(!naive.created || !registered.created,
                    "no-regression comparison needs both builds to complete; "
                    "loosen --tgood/--tbad");
            if (naive.created && registered.created) {
                fail_if(registered.mean_good_fraction < naive.mean_good_fraction * 0.95f,
                        "registration shrank GOOD fraction: " +
                            std::to_string(registered.mean_good_fraction) + " < " +
                            std::to_string(naive.mean_good_fraction));
                // Gate only on the BEST transition per source sample: that is
                // what the runtime schedules. Median/p25 are reported but not
                // gated: registration de-smears the targets' timing, so far
                // parameters legitimately become more distant while the best
                // transition improves.
                fail_if(registered.mean_min_distance > naive.mean_min_distance * 1.05f,
                        "registration worsened best transition distance: " +
                            std::to_string(registered.mean_min_distance) + " > " +
                            std::to_string(naive.mean_min_distance));
            }
        }
    }

    std::cout << (failed ? "RESULT=FAIL" : "RESULT=PASS") << "\n";
    return failed ? 1 : 0;
}

ValidateGraphOptions ParseValidateGraphOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--validate-graph needs <spec>");
    }
    ValidateGraphOptions options;
    options.spec_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--contact-joints") {
            options.contact_joints_csv = require_value("--contact-joints");
        } else if (option == "--cycle-joint") {
            options.cycle_joint = require_value("--cycle-joint");
        } else if (option == "--min-contact-frames") {
            options.min_contact_frames = std::stoi(require_value("--min-contact-frames"));
        } else if (option == "--tgood") {
            options.builder.good_transition_threshold = std::stof(require_value("--tgood"));
        } else if (option == "--tbad") {
            options.builder.bad_transition_threshold = std::stof(require_value("--tbad"));
        } else if (option == "--source-samples") {
            options.builder.source_sample_count = std::stoi(require_value("--source-samples"));
        } else if (option == "--target-samples") {
            options.builder.target_sample_count = std::stoi(require_value("--target-samples"));
        } else if (option == "--seed") {
            options.builder.seed =
                static_cast<unsigned int>(std::stoul(require_value("--seed")));
        } else if (option == "--min-edge-samples") {
            options.min_edge_samples = std::stoi(require_value("--min-edge-samples"));
        } else if (option == "--min-good-fraction") {
            options.min_good_fraction = std::stof(require_value("--min-good-fraction"));
        } else if (option == "--assert-no-regression") {
            options.assert_no_regression = true;
        } else {
            throw std::runtime_error("unknown validate-graph option '" + option + "'");
        }
    }
    return options;
}

// ---------------------------------------------------------------------------
// Phase 4: paper applications, validated numerically.
//   --random-walk  random transitions through the PMG; popping metric
//   --goto         goal-directed locomotion via semantic control
//                  (target position -> desired heading -> curvature parameter)
// ---------------------------------------------------------------------------

// Load a complete artifact (V4+), or build the same artifact in-memory from a
// GraphSpec for compatibility with existing commands and tests.
pmg::BuiltPmgArtifact LoadOrBuildRuntimeArtifact(
    const std::string& input_path,
    const std::string& cycle_joint,
    const std::string& contact_joints_csv,
    int min_contact_frames,
    const pmg::PmgBuilderConfig& builder_config) {
    if (std::filesystem::path(input_path).extension() == ".pmg") {
        pmg::BuiltPmgArtifact artifact =
            pmg::LoadPmgArtifactText(input_path);
        if (artifact.skeleton.NumJoints() == 0) {
            throw std::runtime_error(
                "runtime requires a complete (V4+) artifact containing its Skeleton");
        }
        if (artifact.metadata.frames_per_second <= 0.0f) {
            throw std::runtime_error(
                "runtime artifact has invalid frame metadata");
        }
        return artifact;
    }

    pmg::ArtifactBuildConfig config;
    config.default_edge_config = builder_config;
    config.default_cycle_joint = cycle_joint;
    config.default_contact_joints = SplitCommaList(contact_joints_csv);
    config.default_min_contact_frames = min_contact_frames;
    config.default_dtw_refine = true;
    return pmg::BuildPmgArtifactFromSpec(
        pmg::LoadGraphSpec(input_path), config);
}

struct PopStats {
    float median_step = 0.0f;
    float max_step = 0.0f;
    float Ratio() const {
        return median_step > 1.0e-6f ? max_step / median_step : 0.0f;
    }
};

PopStats MeasurePopping(const pmg::Skeleton& skeleton, const std::vector<pmg::Pose>& poses) {
    PopStats stats;
    std::vector<float> deltas;
    deltas.reserve(poses.size());
    for (std::size_t frame = 1; frame < poses.size(); ++frame) {
        deltas.push_back(MeanJointDistance(skeleton, poses[frame - 1], poses[frame]));
    }
    if (deltas.empty()) {
        return stats;
    }
    stats.max_step = *std::max_element(deltas.begin(), deltas.end());
    stats.median_step = Percentile(deltas, 0.5f);
    return stats;
}

struct RandomWalkOptions {
    std::string spec_path;
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    std::string cycle_joint;
    int min_contact_frames = 3;
    pmg::PmgBuilderConfig builder;
    float seconds = 30.0f;
    unsigned int seed = 99u;
    // Hold one constant parameter instead of randomizing: calibration mode
    // that reports the achieved world turn rate at that parameter.
    float hold_parameter = std::numeric_limits<float>::quiet_NaN();
    // Assert thresholds; negative = report only.
    int min_transitions = -1;
    float max_pop_ratio = -1.0f;
};

// Paper application A: stream random transitions through the graph and verify
// no frame-to-frame popping (worst step vs the median walking step).
int RandomWalkCommand(const RandomWalkOptions& options) {
    const pmg::BuiltPmgArtifact artifact = LoadOrBuildRuntimeArtifact(
        options.spec_path, options.cycle_joint, options.contact_joints_csv,
        options.min_contact_frames, options.builder);

    pmg::PointCloudAlignment alignment(artifact.skeleton);
    pmg::RuntimeController controller(artifact.graph, alignment);

    std::mt19937 rng(options.seed);
    const bool hold = !std::isnan(options.hold_parameter);

    const pmg::ParameterDomain start_domain =
        artifact.graph.Node(0).motion_space.Domain();
    controller.Start(0,
                     hold ? pmg::ParameterVector{options.hold_parameter}
                          : start_domain.SampleUniform(rng),
                     artifact.metadata.frames_per_second);

    pmg::RuntimeControlRequest request;
    if (hold) {
        request.desired_node = 0;
        request.desired_parameter = {options.hold_parameter};
    } else {
        request = pmg::ChooseRandomOutgoingTransition(
                      artifact.graph, controller.CurrentNode(), rng)
                      .request;
    }

    const float dt = 1.0f / artifact.metadata.frames_per_second;
    const int total_frames = static_cast<int>(
        options.seconds * artifact.metadata.frames_per_second);
    int last_transition_count = 0;

    std::vector<pmg::Pose> poses;
    poses.reserve(static_cast<std::size_t>(total_frames));
    poses.push_back(controller.CurrentPose());

    for (int frame = 0; frame < total_frames; ++frame) {
        controller.Update(dt, request);
        poses.push_back(controller.CurrentPose());

        if (!hold && controller.CompletedTransitions() != last_transition_count) {
            last_transition_count = controller.CompletedTransitions();
            request = pmg::ChooseRandomOutgoingTransition(
                          artifact.graph, controller.CurrentNode(), rng)
                          .request;
        }
    }

    if (hold) {
        // Achieved world turn rate: unwrapped heading change over the run.
        float unwrapped_heading = 0.0f;
        float previous_heading = pmg::PoseFacingYaw(poses.front());
        for (std::size_t frame = 1; frame < poses.size(); ++frame) {
            const float heading = pmg::PoseFacingYaw(poses[frame]);
            unwrapped_heading +=
                pmg::WrapAngleRadians(heading - previous_heading);
            previous_heading = heading;
        }
        const float elapsed = static_cast<float>(poses.size() - 1) /
                              artifact.metadata.frames_per_second;
        std::cout << "hold_parameter=" << options.hold_parameter << "\n";
        std::cout << "achieved_turn_rate=" << unwrapped_heading / elapsed << " rad/s\n";
    }

    const PopStats stats = MeasurePopping(artifact.skeleton, poses);
    std::cout << "frames=" << poses.size() << "\n";
    std::cout << "transitions=" << controller.CompletedTransitions() << "\n";
    std::cout << "median_step=" << stats.median_step << "\n";
    std::cout << "max_step=" << stats.max_step << "\n";
    std::cout << "pop_ratio=" << stats.Ratio() << "\n";

    bool failed = false;
    if (options.min_transitions >= 0 &&
        controller.CompletedTransitions() < options.min_transitions) {
        std::cout << "ASSERT FAIL: transitions=" << controller.CompletedTransitions()
                  << " < " << options.min_transitions << "\n";
        failed = true;
    }
    if (options.max_pop_ratio >= 0.0f && stats.Ratio() > options.max_pop_ratio) {
        std::cout << "ASSERT FAIL: pop_ratio=" << stats.Ratio() << " > "
                  << options.max_pop_ratio << "\n";
        failed = true;
    }
    std::cout << (failed ? "RESULT=FAIL" : "RESULT=PASS") << "\n";
    return failed ? 1 : 0;
}

struct GotoOptions {
    std::string spec_path;
    float target_x = 0.0f;
    float target_z = 0.0f;
    std::string contact_joints_csv = "LeftAnkle,RightAnkle";
    std::string cycle_joint;
    int min_contact_frames = 3;
    pmg::PmgBuilderConfig builder;
    float seconds = 60.0f;
    bool trace = false;
    std::optional<float> final_facing_degrees;
    float facing_tolerance_degrees = 15.0f;
    // Assert thresholds; negative = report only.
    float tolerance = -1.0f;
    float max_pop_ratio = -1.0f;
};

// Paper application B/C: goal-directed locomotion through semantic control.
// The user-level intent is a target position; it is converted to a desired
// heading, the heading error to a turn rate, and the turn rate to the walk
// space's curvature parameter via the examples' measured turn rates. The
// raw parameter never appears in the interface.
int GotoCommand(const GotoOptions& options) {
    const pmg::BuiltPmgArtifact artifact = LoadOrBuildRuntimeArtifact(
        options.spec_path, options.cycle_joint, options.contact_joints_csv,
        options.min_contact_frames, options.builder);

    const pmg::ParametricMotionSpace& walk_space =
        artifact.graph.Node(0).motion_space;
    const float parameter_min = walk_space.MinParameter()[0];
    const float parameter_max = walk_space.MaxParameter()[0];
    pmg::GoalDirectedLocomotion steering(
        artifact.graph, artifact.skeleton, 0,
        artifact.metadata.frames_per_second);
    const pmg::SteeringCalibration& calibration = steering.Calibration();
    for (std::size_t sample = 0; sample < calibration.parameters.size(); ++sample) {
        std::cout << "achieved_turn_rate[param="
                  << calibration.parameters[sample] << "]="
                  << calibration.achieved_turn_rates[sample] << " rad/s\n";
    }
    std::cout << "travel_heading_offset_deg="
              << calibration.travel_heading_offset * 180.0f / pmg::kPi
              << "\n";

    pmg::PointCloudAlignment alignment(artifact.skeleton);
    pmg::RuntimeController controller(artifact.graph, alignment);
    controller.Start(0, {0.5f * (parameter_min + parameter_max)},
                     artifact.metadata.frames_per_second);

    const float dt = 1.0f / artifact.metadata.frames_per_second;
    const int total_frames = static_cast<int>(
        options.seconds * artifact.metadata.frames_per_second);

    std::vector<pmg::Pose> poses;
    poses.reserve(static_cast<std::size_t>(total_frames));
    float min_distance = std::numeric_limits<float>::max();
    float reached_at_seconds = -1.0f;
    pmg::GoalRequest goal;
    goal.target_position = {options.target_x, 0.0f, options.target_z};
    if (options.final_facing_degrees.has_value()) {
        goal.final_facing_yaw =
            *options.final_facing_degrees * pmg::kPi / 180.0f;
    }

    for (int frame = 0; frame < total_frames; ++frame) {
        const pmg::Pose pose = controller.CurrentPose();
        poses.push_back(pose);

        const float dx = options.target_x - pose.root_position.x;
        const float dz = options.target_z - pose.root_position.z;
        const float distance = std::sqrt(dx * dx + dz * dz);
        min_distance = std::min(min_distance, distance);
        if (options.tolerance >= 0.0f &&
            steering.Reached(
                pose, goal, options.tolerance,
                options.facing_tolerance_degrees * pmg::kPi / 180.0f)) {
            reached_at_seconds = static_cast<float>(frame) * dt;
            break;
        }

        const pmg::RuntimeControlRequest request =
            steering.RequestForPose(pose, goal);

        if (options.trace && frame % 30 == 0) {
            std::cout << "t=" << static_cast<float>(frame) * dt
                      << " pos=(" << pose.root_position.x << ", " << pose.root_position.z
                      << ") dist=" << distance
                      << " param=" << request.desired_parameter[0] << "\n";
        }

        controller.Update(dt, request);
    }

    const PopStats stats = MeasurePopping(artifact.skeleton, poses);
    std::cout << "target=(" << options.target_x << ", " << options.target_z << ")\n";
    std::cout << "min_distance=" << min_distance << "\n";
    std::cout << "reached=" << (reached_at_seconds >= 0.0f ? 1 : 0) << "\n";
    if (reached_at_seconds >= 0.0f) {
        std::cout << "reached_at_seconds=" << reached_at_seconds << "\n";
    }
    std::cout << "transitions=" << controller.CompletedTransitions() << "\n";
    std::cout << "median_step=" << stats.median_step << "\n";
    std::cout << "max_step=" << stats.max_step << "\n";
    std::cout << "pop_ratio=" << stats.Ratio() << "\n";

    bool failed = false;
    if (options.tolerance >= 0.0f && reached_at_seconds < 0.0f) {
        std::cout << "ASSERT FAIL: target not reached within tolerance "
                  << options.tolerance << " (min_distance=" << min_distance << ")\n";
        failed = true;
    }
    if (options.max_pop_ratio >= 0.0f && stats.Ratio() > options.max_pop_ratio) {
        std::cout << "ASSERT FAIL: pop_ratio=" << stats.Ratio() << " > "
                  << options.max_pop_ratio << "\n";
        failed = true;
    }
    std::cout << (failed ? "RESULT=FAIL" : "RESULT=PASS") << "\n";
    return failed ? 1 : 0;
}

void ParseSharedRuntimeOption(
    const std::string& option,
    const std::function<std::string(const char*)>& require_value,
    std::string& contact_joints_csv,
    std::string& cycle_joint,
    int& min_contact_frames,
    pmg::PmgBuilderConfig& builder,
    bool& handled) {
    handled = true;
    if (option == "--contact-joints") {
        contact_joints_csv = require_value("--contact-joints");
    } else if (option == "--cycle-joint") {
        cycle_joint = require_value("--cycle-joint");
    } else if (option == "--min-contact-frames") {
        min_contact_frames = std::stoi(require_value("--min-contact-frames"));
    } else if (option == "--tgood") {
        builder.good_transition_threshold = std::stof(require_value("--tgood"));
    } else if (option == "--tbad") {
        builder.bad_transition_threshold = std::stof(require_value("--tbad"));
    } else if (option == "--source-samples") {
        builder.source_sample_count = std::stoi(require_value("--source-samples"));
    } else if (option == "--target-samples") {
        builder.target_sample_count = std::stoi(require_value("--target-samples"));
    } else if (option == "--seed") {
        builder.seed = static_cast<unsigned int>(std::stoul(require_value("--seed")));
    } else {
        handled = false;
    }
}

RandomWalkOptions ParseRandomWalkOptions(int argc, char** argv) {
    if (argc < 3) {
        throw std::runtime_error("--random-walk needs <spec>");
    }
    RandomWalkOptions options;
    options.spec_path = argv[2];
    for (int index = 3; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        bool handled = false;
        ParseSharedRuntimeOption(option, require_value, options.contact_joints_csv,
                                 options.cycle_joint, options.min_contact_frames,
                                 options.builder, handled);
        if (handled) {
            continue;
        }
        if (option == "--seconds") {
            options.seconds = std::stof(require_value("--seconds"));
        } else if (option == "--walk-seed") {
            options.seed = static_cast<unsigned int>(std::stoul(require_value("--walk-seed")));
        } else if (option == "--hold-param") {
            options.hold_parameter = std::stof(require_value("--hold-param"));
        } else if (option == "--min-transitions") {
            options.min_transitions = std::stoi(require_value("--min-transitions"));
        } else if (option == "--max-pop-ratio") {
            options.max_pop_ratio = std::stof(require_value("--max-pop-ratio"));
        } else {
            throw std::runtime_error("unknown random-walk option '" + option + "'");
        }
    }
    return options;
}

GotoOptions ParseGotoOptions(int argc, char** argv) {
    if (argc < 5) {
        throw std::runtime_error("--goto needs <spec> <x> <z>");
    }
    GotoOptions options;
    options.spec_path = argv[2];
    options.target_x = std::stof(argv[3]);
    options.target_z = std::stof(argv[4]);
    for (int index = 5; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        bool handled = false;
        ParseSharedRuntimeOption(option, require_value, options.contact_joints_csv,
                                 options.cycle_joint, options.min_contact_frames,
                                 options.builder, handled);
        if (handled) {
            continue;
        }
        if (option == "--seconds") {
            options.seconds = std::stof(require_value("--seconds"));
        } else if (option == "--tolerance") {
            options.tolerance = std::stof(require_value("--tolerance"));
        } else if (option == "--max-pop-ratio") {
            options.max_pop_ratio = std::stof(require_value("--max-pop-ratio"));
        } else if (option == "--facing-degrees") {
            options.final_facing_degrees =
                std::stof(require_value("--facing-degrees"));
        } else if (option == "--facing-tolerance-degrees") {
            options.facing_tolerance_degrees =
                std::stof(require_value("--facing-tolerance-degrees"));
        } else if (option == "--trace") {
            options.trace = true;
        } else {
            throw std::runtime_error("unknown goto option '" + option + "'");
        }
    }
    return options;
}

SpaceSweepOptions ParseSpaceSweepOptions(int argc, char** argv) {
    if (argc < 4) {
        throw std::runtime_error("--space-sweep needs <spec> <node>");
    }
    SpaceSweepOptions options;
    options.spec_path = argv[2];
    options.node_name = argv[3];
    for (int index = 4; index < argc; ++index) {
        const std::string option = argv[index];
        auto require_value = [&](const char* name) -> std::string {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(name) + " requires a value");
            }
            ++index;
            return argv[index];
        };
        if (option == "--contact-joints") {
            options.contact_joints_csv = require_value("--contact-joints");
        } else if (option == "--cycle-joint") {
            options.cycle_joint = require_value("--cycle-joint");
        } else if (option == "--sweep-steps") {
            options.sweep_steps = std::stoi(require_value("--sweep-steps"));
        } else if (option == "--min-contact-frames") {
            options.min_contact_frames = std::stoi(require_value("--min-contact-frames"));
        } else if (option == "--min-contacts") {
            options.min_contacts = std::stoi(require_value("--min-contacts"));
        } else if (option == "--max-foot-slide") {
            options.max_foot_slide = std::stof(require_value("--max-foot-slide"));
        } else if (option == "--max-adjacent-step") {
            options.max_adjacent_step = std::stof(require_value("--max-adjacent-step"));
        } else if (option == "--assert-no-regression") {
            options.assert_no_regression = true;
        } else if (option == "--dtw-refine") {
            options.dtw_refine = true;
        } else if (option == "--foot-lock") {
            options.foot_lock = true;
        } else {
            throw std::runtime_error("unknown space-sweep option '" + option + "'");
        }
    }
    return options;
}

void PrintUsage() {
    std::cerr << "Usage:\n"
              << "  pmg_cli --bvh path/to/file.bvh\n"
              << "  pmg_cli --list-bvh-joints path/to/file.bvh\n"
              << "  pmg_cli --inspect-transition source.bvh target.bvh\n"
              << "  pmg_cli --dump-distance-grid source.bvh target.bvh out.csv\n"
              << "  pmg_cli --calibrate-thresholds bvh_directory [manifest.txt]\n"
              << "  pmg_cli --validate-graph-spec graph_spec.txt\n"
              << "  pmg_cli --diagnose-graph-edge graph_spec.txt source_node target_node [--tgood X --tbad Y]\n"
              << "  pmg_cli --build-graph graph_spec.txt out.pmg [--tgood X --tbad Y]\n"
              << "  pmg_cli --inspect-graph graph.pmg\n"
              << "  pmg_cli --inspect-contacts path/to/file.bvh LeftAnkle,RightAnkle\n"
              << "  pmg_cli --space-sweep graph_spec.txt node [--contact-joints a,b]\n"
              << "      [--cycle-joint name] [--sweep-steps N] [--min-contacts N]\n"
              << "      [--max-foot-slide X] [--max-adjacent-step X] [--assert-no-regression]\n"
              << "      [--dtw-refine] [--foot-lock]\n"
              << "  pmg_cli --validate-graph graph_spec.txt [--cycle-joint name]\n"
              << "      [--tgood X --tbad Y --source-samples N --target-samples N --seed S]\n"
              << "      [--min-edge-samples N] [--min-good-fraction F] [--assert-no-regression]\n"
              << "  pmg_cli --random-walk graph_spec.txt|graph.pmg [--seconds S] [--walk-seed N]\n"
              << "      [--min-transitions N] [--max-pop-ratio X] [builder/registration opts]\n"
              << "  pmg_cli --goto graph_spec.txt|graph.pmg x z [--seconds S] [--tolerance D]\n"
              << "      [--facing-degrees DEG --facing-tolerance-degrees DEG]\n"
              << "      [--max-pop-ratio X] [builder/registration opts]\n";
}

}  // namespace

int RunPmgCli(int argc, char** argv) {
    try {
        if (argc == 3 && std::string(argv[1]) == "--bvh") {
            return PrintBvhSummary(argv[2]);
        }

        if (argc == 3 && std::string(argv[1]) == "--list-bvh-joints") {
            return ListBvhJoints(argv[2]);
        }

        if (argc == 4 && std::string(argv[1]) == "--inspect-transition") {
            return InspectTransition(argv[2], argv[3]);
        }

        if (argc == 5 && std::string(argv[1]) == "--dump-distance-grid") {
            return DumpDistanceGrid(argv[2], argv[3], argv[4]);
        }

        if (argc == 3 && std::string(argv[1]) == "--calibrate-thresholds") {
            return CalibrateThresholds(argv[2]);
        }

        if (argc == 4 && std::string(argv[1]) == "--calibrate-thresholds") {
            return CalibrateThresholds(argv[2], argv[3]);
        }

        if (argc >= 4 && std::string(argv[1]) == "--build-graph") {
            const pmg::PmgBuilderConfig config = ParseBuilderConfigOptions(argc, argv, 4);
            return BuildGraphCommand(argv[2], argv[3], config);
        }

        if (argc == 3 && std::string(argv[1]) == "--validate-graph-spec") {
            return ValidateGraphSpecCommand(argv[2]);
        }

        if (argc >= 5 && std::string(argv[1]) == "--diagnose-graph-edge") {
            const pmg::PmgBuilderConfig config = ParseBuilderConfigOptions(argc, argv, 5);
            return DiagnoseGraphEdgeCommand(argv[2], argv[3], argv[4], config);
        }

        if (argc == 3 && std::string(argv[1]) == "--inspect-graph") {
            return InspectGraphCommand(argv[2]);
        }

        if (argc == 4 && std::string(argv[1]) == "--inspect-contacts") {
            return InspectContacts(argv[2], argv[3]);
        }

        if (argc >= 4 && std::string(argv[1]) == "--space-sweep") {
            return SpaceSweepCommand(ParseSpaceSweepOptions(argc, argv));
        }

        if (argc >= 3 && std::string(argv[1]) == "--validate-graph") {
            return ValidateGraphCommand(ParseValidateGraphOptions(argc, argv));
        }

        if (argc >= 3 && std::string(argv[1]) == "--random-walk") {
            return RandomWalkCommand(ParseRandomWalkOptions(argc, argv));
        }

        if (argc >= 5 && std::string(argv[1]) == "--goto") {
            return GotoCommand(ParseGotoOptions(argc, argv));
        }

        PrintUsage();
        return 2;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}
