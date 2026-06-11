#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GraphIo.h"
#include "pmg/GraphSpec.h"
#include "pmg/MotionDistance.h"
#include "pmg/MotionRegistration.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/RuntimeController.h"
#include "pmg/SkeletonCompatibility.h"

#include <algorithm>
#include <cctype>
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

pmg::MotionClip MakeSyntheticClip(
    const std::string& name,
    float root_x_end,
    float yaw_degrees_end) {
    pmg::MotionClip clip;
    clip.name = name;
    clip.frames_per_second = 30.0f;

    constexpr int kFrameCount = 30;
    for (int frame_index = 0; frame_index < kFrameCount; ++frame_index) {
        const float phase =
            static_cast<float>(frame_index) / static_cast<float>(kFrameCount - 1);

        pmg::Pose pose;
        pose.root_position = {phase * root_x_end, 0.0f, 0.0f};
        pose.local_rotations.push_back(
            pmg::EulerAxisRotation('Y', phase * yaw_degrees_end));
        clip.frames.push_back(pose);
    }

    return clip;
}

int RunSyntheticDemo() {
    pmg::ParametricMotionSpace walk_space("walk", 1);
    walk_space.AddExample({-1.0f}, MakeSyntheticClip("walk_left", 1.0f, -20.0f));
    walk_space.AddExample({0.0f}, MakeSyntheticClip("walk_straight", 1.0f, 0.0f));
    walk_space.AddExample({1.0f}, MakeSyntheticClip("walk_right", 1.0f, 20.0f));

    pmg::ParametricMotionGraph graph;
    const int walk_node = graph.AddNode("walk", walk_space);

    pmg::ParameterAabb target_box;
    target_box.min_corner = {-1.0f};
    target_box.max_corner = {1.0f};

    pmg::PmgEdge self_edge;
    self_edge.source_node = walk_node;
    self_edge.target_node = walk_node;
    self_edge.samples.push_back({{0.0f}, target_box, 0.80f, 0.10f});
    graph.AddEdge(self_edge);

    pmg::RootOnlyAlignment alignment;
    pmg::RuntimeController controller(graph, alignment);
    controller.Start(walk_node, {0.0f}, 30, 30.0f);

    pmg::RuntimeControlRequest request;
    request.desired_node = walk_node;
    request.desired_parameter = {0.75f};

    for (int step = 0; step < 45; ++step) {
        controller.Update(1.0f / 30.0f, request);
        const pmg::Pose pose = controller.CurrentPose();
        std::cout << "step=" << step
                  << " node=" << controller.CurrentNode()
                  << " phase=" << controller.CurrentPhase()
                  << " root_x=" << pose.root_position.x
                  << "\n";
    }

    return 0;
}

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

void PrintVec3(const std::string& label, const pmg::Vec3& value) {
    std::cout << label << "=("
              << value.x << ", "
              << value.y << ", "
              << value.z << ")\n";
}

float HorizontalLength(const pmg::Vec3& value) {
    return std::sqrt(value.x * value.x + value.z * value.z);
}

pmg::Vec3 NormalizeHorizontalOrZero(const pmg::Vec3& value) {
    const float length = HorizontalLength(value);
    if (length <= pmg::kSmallEpsilon) {
        return {0.0f, 0.0f, 0.0f};
    }
    return {value.x / length, 0.0f, value.z / length};
}

float HeadingDegreesFromVector(const pmg::Vec3& value) {
    return std::atan2(value.x, value.z) * 180.0f / pmg::kPi;
}

float HorizontalAngleDifferenceDegrees(
    const pmg::Vec3& first,
    const pmg::Vec3& second) {
    const pmg::Vec3 a = NormalizeHorizontalOrZero(first);
    const pmg::Vec3 b = NormalizeHorizontalOrZero(second);
    const float dot = std::clamp(pmg::Dot(a, b), -1.0f, 1.0f);
    return std::acos(dot) * 180.0f / pmg::kPi;
}

void PrintVectorAgainstMotion(
    const std::string& label,
    const pmg::Vec3& vector,
    const pmg::Vec3& motion_direction) {
    std::cout << label << "=("
              << vector.x << ", "
              << vector.y << ", "
              << vector.z << ")"
              << " yaw_deg=" << HeadingDegreesFromVector(vector)
              << " angle_to_motion_deg="
              << HorizontalAngleDifferenceDegrees(vector, motion_direction)
              << "\n";
}

void PrintAxisAgreement(
    const char* axis_name,
    const pmg::Vec3& local_axis,
    const pmg::Quaternion& root_rotation,
    const pmg::Vec3& motion_direction) {
    const pmg::Vec3 world_axis = pmg::Rotate(root_rotation, local_axis);

    std::cout << axis_name
              << " world=("
              << world_axis.x << ", "
              << world_axis.y << ", "
              << world_axis.z << ")"
              << " yaw_deg=" << HeadingDegreesFromVector(world_axis)
              << " angle_to_motion_deg="
              << HorizontalAngleDifferenceDegrees(world_axis, motion_direction)
              << "\n";
}

int PrintBvhSummary(const std::string& path) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);
    std::cout << "BVH: " << path << "\n";
    std::cout << "joints: " << data.skeleton.NumJoints() << "\n";
    std::cout << "frames: " << data.clip.NumFrames() << "\n";
    std::cout << "fps: " << data.clip.frames_per_second << "\n";
    return 0;
}

int InspectBvhHeading(const std::string& path) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);

    if (data.clip.NumFrames() < 2) {
        throw std::runtime_error("InspectBvhHeading: BVH needs at least 2 frames");
    }

    const pmg::Pose& first_pose = data.clip.frames.front();
    const pmg::Pose& last_pose = data.clip.frames.back();
    const pmg::Vec3 motion_direction =
        last_pose.root_position - first_pose.root_position;

    std::cout << "BVH: " << path << "\n";
    std::cout << "joints: " << data.skeleton.NumJoints() << "\n";
    std::cout << "frames: " << data.clip.NumFrames() << "\n";
    std::cout << "fps: " << data.clip.frames_per_second << "\n";
    PrintVec3("root displacement", motion_direction);
    std::cout << "motion_heading_yaw_deg="
              << HeadingDegreesFromVector(motion_direction)
              << "\n";

    const pmg::Quaternion root_rotation = first_pose.local_rotations.front();

    PrintAxisAgreement("+X", {1.0f, 0.0f, 0.0f}, root_rotation, motion_direction);
    PrintAxisAgreement("-X", {-1.0f, 0.0f, 0.0f}, root_rotation, motion_direction);
    PrintAxisAgreement("+Z", {0.0f, 0.0f, 1.0f}, root_rotation, motion_direction);
    PrintAxisAgreement("-Z", {0.0f, 0.0f, -1.0f}, root_rotation, motion_direction);

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

int DebugBvhJointPair(
    const std::string& path,
    const std::string& left_name,
    const std::string& right_name) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);

    const std::optional<int> left_index = FindJointExact(data.skeleton, left_name);
    const std::optional<int> right_index = FindJointExact(data.skeleton, right_name);

    if (!left_index || !right_index) {
        std::cout << "Joint lookup failed. Available joints:\n";
        for (int joint_index = 0; joint_index < data.skeleton.NumJoints(); ++joint_index) {
            std::cout << joint_index << ": "
                      << data.skeleton.joints[joint_index].name << "\n";
        }
        return 1;
    }

    const pmg::Joint& left_joint = data.skeleton.joints[*left_index];
    const pmg::Joint& right_joint = data.skeleton.joints[*right_index];
    const pmg::Pose& pose = data.clip.frames.front();
    const std::vector<pmg::JointWorldState> world =
        pmg::ComputeForwardKinematics(data.skeleton, pose);

    const pmg::Quaternion root_rotation = pose.local_rotations.front();
    const pmg::Vec3 local_offset_delta = left_joint.offset - right_joint.offset;
    const pmg::Vec3 expected_world_delta =
        pmg::Rotate(root_rotation, local_offset_delta);
    const pmg::Vec3 actual_world_delta =
        world[*left_index].position - world[*right_index].position;

    std::cout << "BVH: " << path << "\n\n";

    std::cout << "left index=" << *left_index
              << " name=" << left_joint.name
              << " parent=" << left_joint.parent_index
              << "\n";
    PrintVec3("left offset", left_joint.offset);
    PrintVec3("left world", world[*left_index].position);

    std::cout << "\n";

    std::cout << "right index=" << *right_index
              << " name=" << right_joint.name
              << " parent=" << right_joint.parent_index
              << "\n";
    PrintVec3("right offset", right_joint.offset);
    PrintVec3("right world", world[*right_index].position);

    std::cout << "\n";
    PrintVec3("local offset delta", local_offset_delta);
    PrintVec3("expected world delta", expected_world_delta);
    PrintVec3("actual world delta", actual_world_delta);

    const float error = (actual_world_delta - expected_world_delta).Norm();
    std::cout << "delta_error_norm=" << error << "\n";

    return 0;
}

void PrintPairGeometryAtFrame(
    const pmg::BvhData& data,
    const std::string& left_name,
    const std::string& right_name,
    int frame_index,
    const pmg::Vec3& motion_direction) {
    const std::optional<int> left_index = FindJointExact(data.skeleton, left_name);
    const std::optional<int> right_index = FindJointExact(data.skeleton, right_name);

    if (!left_index || !right_index) {
        std::cout << left_name << " / " << right_name << ": joint lookup failed\n";
        return;
    }

    const pmg::Pose& pose = data.clip.frames[frame_index];
    const std::vector<pmg::JointWorldState> world =
        pmg::ComputeForwardKinematics(data.skeleton, pose);

    const pmg::Vec3 left_to_right =
        world[*right_index].position - world[*left_index].position;
    const pmg::Vec3 right_to_left =
        world[*left_index].position - world[*right_index].position;

    const pmg::Vec3 world_up{0.0f, 1.0f, 0.0f};
    const pmg::Vec3 cross_right_to_left_up = pmg::Cross(right_to_left, world_up);
    const pmg::Vec3 cross_up_right_to_left = pmg::Cross(world_up, right_to_left);

    std::cout << left_name << " / " << right_name << "\n";
    PrintVectorAgainstMotion("  left_to_right", left_to_right, motion_direction);
    PrintVectorAgainstMotion("  right_to_left", right_to_left, motion_direction);
    PrintVectorAgainstMotion("  cross(right_to_left, up)", cross_right_to_left_up, motion_direction);
    PrintVectorAgainstMotion("  cross(up, right_to_left)", cross_up_right_to_left, motion_direction);
}

int InspectBvhGeometry(const std::string& path) {
    const pmg::BvhData data = pmg::BvhLoader::Load(path);

    if (data.clip.NumFrames() < 2) {
        throw std::runtime_error("InspectBvhGeometry: BVH needs at least 2 frames");
    }

    const pmg::Vec3 motion_direction =
        data.clip.frames.back().root_position - data.clip.frames.front().root_position;

    std::cout << "BVH: " << path << "\n";
    PrintVec3("root displacement", motion_direction);
    std::cout << "motion_heading_yaw_deg="
              << HeadingDegreesFromVector(motion_direction)
              << "\n\n";

    const int frame_indices[] = {0, data.clip.NumFrames() / 2, data.clip.NumFrames() - 1};

    for (const int frame_index : frame_indices) {
        std::cout << "frame=" << frame_index << "\n";
        PrintPairGeometryAtFrame(data, "LeftHip", "RightHip", frame_index, motion_direction);
        PrintPairGeometryAtFrame(data, "LeftShoulder", "RightShoulder", frame_index, motion_direction);
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


int BuildGraphCommand(const std::string& spec_path,
                      const std::string& output_path,
                      const pmg::PmgBuilderConfig& config) {
    // Keep CLI defaults conservative and paper-like; production thresholds should
    // still be calibrated with --calibrate-thresholds for the target corpus.
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(spec_path);
    const pmg::ParametricMotionGraph graph = pmg::BuildGraphFromSpec(spec, config);
    pmg::SaveGraphText(graph, output_path);
    std::cout << "wrote graph: " << output_path << "\n";
    std::cout << "nodes=" << graph.NumNodes()
              << " edges=" << graph.NumEdges() << "\n";
    return 0;
}

int InspectGraphCommand(const std::string& graph_path) {
    const pmg::ParametricMotionGraph graph = pmg::LoadGraphText(graph_path);
    std::cout << "graph: " << graph_path << "\n";
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
    const char* label) {
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

        const pmg::MotionClip clip =
            space.GenerateClip(parameter, generated_frame_count, frames_per_second);

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
    if (options.dtw_refine) {
        pmg::ParametricMotionSpace dtw_space = registered_space;
        pmg::RefineRegistrationByDtw(dtw_space, skeleton, {});
        std::cout << "dtw_refined=yes\n";
        dtw = MeasureSpaceSweep(
            dtw_space, skeleton, contact_joints, settings, options.sweep_steps,
            generated_frame_count, frames_per_second, "dtw");
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

struct RuntimeGraphBundle {
    pmg::Skeleton skeleton;
    pmg::ParametricMotionGraph graph;
    int generated_frame_count = 30;
    float frames_per_second = 30.0f;
};

// Build the full PMG from a spec with cycle-normalized, registered spaces.
RuntimeGraphBundle BuildRegisteredGraph(
    const std::string& spec_path,
    const std::string& cycle_joint,
    const std::string& contact_joints_csv,
    int min_contact_frames,
    const pmg::PmgBuilderConfig& builder_config) {
    const pmg::GraphSpec spec = pmg::LoadGraphSpec(spec_path);

    RuntimeGraphBundle bundle;
    std::map<std::string, int> node_indices;
    std::vector<pmg::ParametricMotionSpace> spaces;

    for (const pmg::GraphSpecNode& node : spec.nodes) {
        pmg::ParametricMotionSpace space =
            BuildSpaceForSweep(spec, node.name, cycle_joint, bundle.skeleton);

        const std::vector<int> contact_joints =
            ResolveJointList(bundle.skeleton, contact_joints_csv);
        pmg::ContactDetectionSettings settings = pmg::EstimateContactSettings(
            bundle.skeleton, space.Examples().front().clip, contact_joints);
        settings.min_contact_frames = min_contact_frames;
        pmg::RegisterSpaceByContacts(space, bundle.skeleton, contact_joints, settings);

        bundle.generated_frame_count = space.Examples().front().clip.NumFrames();
        bundle.frames_per_second = space.Examples().front().clip.frames_per_second;

        node_indices.emplace(node.name, bundle.graph.AddNode(node.name, space));
        spaces.push_back(std::move(space));
    }

    for (const pmg::GraphSpecEdge& spec_edge : spec.edges) {
        const int source_index = node_indices.at(spec_edge.source_node);
        const int target_index = node_indices.at(spec_edge.target_node);
        pmg::EdgeBuildResult result = pmg::PmgBuilder::BuildEdgeWithReport(
            bundle.skeleton, source_index, target_index,
            bundle.graph.Node(source_index).motion_space,
            bundle.graph.Node(target_index).motion_space, builder_config);
        if (!result.report.edge_created) {
            throw std::runtime_error("edge " + spec_edge.source_node + " -> " +
                                     spec_edge.target_node +
                                     " rejected: " + result.report.reject_reason);
        }
        bundle.graph.AddEdge(std::move(result.edge));
    }
    return bundle;
}

float HeadingOfPose(const pmg::Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const pmg::Vec3 forward = pmg::Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

float WrapPiCli(float angle_radians) {
    return angle_radians -
           2.0f * pmg::kPi * std::round(angle_radians / (2.0f * pmg::kPi));
}

// Constant angle from the root's +Z reference to the actual travel direction,
// estimated as a displacement-weighted circular mean over a clip. BVH corpora
// differ in which root axis faces travel (this one walks roughly along +X);
// steering must aim the travel direction, not the reference axis.
float EstimateTravelHeadingOffset(const pmg::MotionClip& clip) {
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    for (int frame = 0; frame + 1 < clip.NumFrames(); ++frame) {
        const pmg::Vec3 step =
            clip.frames[frame + 1].root_position - clip.frames[frame].root_position;
        const float step_length = HorizontalLength(step);
        if (step_length <= 1.0e-5f) {
            continue;
        }
        const float travel_heading = std::atan2(step.x, step.z);
        const float reference_heading = HeadingOfPose(clip.frames[frame]);
        const float offset = WrapPiCli(travel_heading - reference_heading);
        sum_sin += step_length * std::sin(offset);
        sum_cos += step_length * std::cos(offset);
    }
    return std::atan2(sum_sin, sum_cos);
}

// Achieved world turn rate when streaming the graph at one held parameter.
// This differs from the example clips' own turn rates: each self-transition
// plays only the slice between the target transition phase and the next
// source gate, so the net heading advance per hop is the heading change over
// that slice (including sway), not the full-cycle turn. Steering must be
// calibrated against these achieved rates.
float MeasureAchievedTurnRate(
    const RuntimeGraphBundle& bundle, float parameter, float seconds) {
    pmg::PointCloudAlignment alignment(bundle.skeleton);
    pmg::RuntimeController controller(bundle.graph, alignment);
    controller.Start(0, {parameter}, bundle.generated_frame_count, bundle.frames_per_second);

    pmg::RuntimeControlRequest request;
    request.desired_node = 0;
    request.desired_parameter = {parameter};

    const float dt = 1.0f / bundle.frames_per_second;
    const int total_frames = static_cast<int>(seconds * bundle.frames_per_second);
    float unwrapped = 0.0f;
    float previous_heading = HeadingOfPose(controller.CurrentPose());
    for (int frame = 0; frame < total_frames; ++frame) {
        controller.Update(dt, request);
        const float heading = HeadingOfPose(controller.CurrentPose());
        unwrapped += WrapPiCli(heading - previous_heading);
        previous_heading = heading;
    }
    return unwrapped / (static_cast<float>(total_frames) * dt);
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
    const RuntimeGraphBundle bundle = BuildRegisteredGraph(
        options.spec_path, options.cycle_joint, options.contact_joints_csv,
        options.min_contact_frames, options.builder);

    pmg::PointCloudAlignment alignment(bundle.skeleton);
    pmg::RuntimeController controller(bundle.graph, alignment);

    std::mt19937 rng(options.seed);
    std::uniform_int_distribution<int> node_picker(0, bundle.graph.NumNodes() - 1);
    const bool hold = !std::isnan(options.hold_parameter);

    const pmg::ParameterDomain start_domain = bundle.graph.Node(0).motion_space.Domain();
    controller.Start(0,
                     hold ? pmg::ParameterVector{options.hold_parameter}
                          : start_domain.SampleUniform(rng),
                     bundle.generated_frame_count, bundle.frames_per_second);

    pmg::RuntimeControlRequest request;
    request.desired_node = hold ? 0 : node_picker(rng);
    request.desired_parameter =
        hold ? pmg::ParameterVector{options.hold_parameter}
             : bundle.graph.Node(request.desired_node).motion_space.Domain().SampleUniform(rng);

    const float dt = 1.0f / bundle.frames_per_second;
    const int total_frames = static_cast<int>(options.seconds * bundle.frames_per_second);
    int last_transition_count = 0;

    std::vector<pmg::Pose> poses;
    poses.reserve(static_cast<std::size_t>(total_frames));
    poses.push_back(controller.CurrentPose());

    for (int frame = 0; frame < total_frames; ++frame) {
        controller.Update(dt, request);
        poses.push_back(controller.CurrentPose());

        if (!hold && controller.CompletedTransitions() != last_transition_count) {
            last_transition_count = controller.CompletedTransitions();
            request.desired_node = node_picker(rng);
            request.desired_parameter = bundle.graph.Node(request.desired_node)
                                            .motion_space.Domain()
                                            .SampleUniform(rng);
        }
    }

    if (hold) {
        // Achieved world turn rate: unwrapped heading change over the run.
        float unwrapped_heading = 0.0f;
        float previous_heading = HeadingOfPose(poses.front());
        for (std::size_t frame = 1; frame < poses.size(); ++frame) {
            const float heading = HeadingOfPose(poses[frame]);
            unwrapped_heading += WrapPiCli(heading - previous_heading);
            previous_heading = heading;
        }
        const float elapsed = static_cast<float>(poses.size() - 1) / bundle.frames_per_second;
        std::cout << "hold_parameter=" << options.hold_parameter << "\n";
        std::cout << "achieved_turn_rate=" << unwrapped_heading / elapsed << " rad/s\n";
    }

    const PopStats stats = MeasurePopping(bundle.skeleton, poses);
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
    const RuntimeGraphBundle bundle = BuildRegisteredGraph(
        options.spec_path, options.cycle_joint, options.contact_joints_csv,
        options.min_contact_frames, options.builder);

    const pmg::ParametricMotionSpace& walk_space = bundle.graph.Node(0).motion_space;
    if (walk_space.ParameterDimension() != 1) {
        throw std::runtime_error("--goto expects a 1-D (curvature) walk space");
    }

    // Calibrate "parameter -> achieved turn rate" by streaming the graph at a
    // few held parameters. The achieved curve is generally nonlinear and can
    // even change sign relative to the example clips' turn rates (see
    // MeasureAchievedTurnRate), so steering inverts this measured table.
    const float parameter_min = walk_space.MinParameter()[0];
    const float parameter_max = walk_space.MaxParameter()[0];
    constexpr int kCalibrationPoints = 5;
    constexpr float kCalibrationSeconds = 8.0f;
    std::vector<float> table_parameters;
    std::vector<float> table_rates;
    for (int point = 0; point < kCalibrationPoints; ++point) {
        const float alpha =
            static_cast<float>(point) / static_cast<float>(kCalibrationPoints - 1);
        const float parameter = parameter_min + alpha * (parameter_max - parameter_min);
        const float rate = MeasureAchievedTurnRate(bundle, parameter, kCalibrationSeconds);
        table_parameters.push_back(parameter);
        table_rates.push_back(rate);
        std::cout << "achieved_turn_rate[param=" << parameter << "]=" << rate << " rad/s\n";
    }
    const float lowest_rate = *std::min_element(table_rates.begin(), table_rates.end());
    const float highest_rate = *std::max_element(table_rates.begin(), table_rates.end());
    if (highest_rate - lowest_rate < 1.0e-4f) {
        throw std::runtime_error("--goto: graph has no achievable turn-rate variation");
    }

    // Invert the (possibly non-monotonic) table: prefer a bracketing segment,
    // fall back to the closest calibrated rate.
    const auto parameter_for_rate = [&](float desired_rate) {
        const float clamped = std::clamp(desired_rate, lowest_rate, highest_rate);
        for (std::size_t segment = 0; segment + 1 < table_rates.size(); ++segment) {
            const float rate_a = table_rates[segment];
            const float rate_b = table_rates[segment + 1];
            if ((clamped - rate_a) * (clamped - rate_b) <= 0.0f &&
                std::abs(rate_b - rate_a) > 1.0e-6f) {
                const float alpha = (clamped - rate_a) / (rate_b - rate_a);
                return table_parameters[segment] +
                       alpha * (table_parameters[segment + 1] - table_parameters[segment]);
            }
        }
        std::size_t best = 0;
        for (std::size_t point = 1; point < table_rates.size(); ++point) {
            if (std::abs(table_rates[point] - clamped) < std::abs(table_rates[best] - clamped)) {
                best = point;
            }
        }
        return table_parameters[best];
    };

    const float travel_offset =
        EstimateTravelHeadingOffset(walk_space.Examples().front().clip);
    std::cout << "travel_heading_offset_deg=" << travel_offset * 180.0f / pmg::kPi << "\n";

    pmg::PointCloudAlignment alignment(bundle.skeleton);
    pmg::RuntimeController controller(bundle.graph, alignment);
    controller.Start(0, {0.5f * (parameter_min + parameter_max)},
                     bundle.generated_frame_count, bundle.frames_per_second);

    const float dt = 1.0f / bundle.frames_per_second;
    const int total_frames = static_cast<int>(options.seconds * bundle.frames_per_second);
    // Ask for the full heading correction within one gait cycle.
    const float cycle_seconds =
        static_cast<float>(bundle.generated_frame_count) / bundle.frames_per_second;

    std::vector<pmg::Pose> poses;
    poses.reserve(static_cast<std::size_t>(total_frames));
    float min_distance = std::numeric_limits<float>::max();
    float reached_at_seconds = -1.0f;
    bool swinging = false;

    pmg::RuntimeControlRequest request;
    request.desired_node = 0;
    request.desired_parameter = {0.5f * (parameter_min + parameter_max)};

    for (int frame = 0; frame < total_frames; ++frame) {
        const pmg::Pose pose = controller.CurrentPose();
        poses.push_back(pose);

        const float dx = options.target_x - pose.root_position.x;
        const float dz = options.target_z - pose.root_position.z;
        const float distance = std::sqrt(dx * dx + dz * dz);
        min_distance = std::min(min_distance, distance);
        if (options.tolerance >= 0.0f && distance <= options.tolerance &&
            reached_at_seconds < 0.0f) {
            reached_at_seconds = static_cast<float>(frame) * dt;
            break;
        }

        const float desired_heading = std::atan2(dx, dz);
        const float travel_heading = WrapPiCli(HeadingOfPose(pose) + travel_offset);
        const float heading_error = WrapPiCli(desired_heading - travel_heading);
        const float desired_rate = heading_error / cycle_seconds;

        // When the demanded turn exceeds what its own direction offers (the
        // target sits inside that branch's minimum turning radius), going the
        // long way around with the tightest branch converges; the wide branch
        // orbits forever. Hysteresis keeps the swing from chattering.
        const float tightest_rate =
            std::abs(lowest_rate) > std::abs(highest_rate) ? lowest_rate : highest_rate;
        if (!swinging && std::abs(heading_error) > 0.5f &&
            (desired_rate > highest_rate || desired_rate < lowest_rate) &&
            std::abs(std::clamp(desired_rate, lowest_rate, highest_rate)) <
                0.5f * std::abs(tightest_rate)) {
            swinging = true;
        }
        if (swinging && std::abs(heading_error) < 0.2f) {
            swinging = false;
        }

        const float commanded_rate =
            swinging ? tightest_rate : std::clamp(desired_rate, lowest_rate, highest_rate);
        request.desired_parameter = {std::clamp(
            parameter_for_rate(commanded_rate), parameter_min, parameter_max)};

        if (options.trace && frame % 30 == 0) {
            std::cout << "t=" << static_cast<float>(frame) * dt
                      << " pos=(" << pose.root_position.x << ", " << pose.root_position.z
                      << ") dist=" << distance
                      << " err_deg=" << heading_error * 180.0f / pmg::kPi
                      << " rate=" << commanded_rate
                      << " param=" << request.desired_parameter[0] << "\n";
        }

        controller.Update(dt, request);
    }

    const PopStats stats = MeasurePopping(bundle.skeleton, poses);
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
        } else {
            throw std::runtime_error("unknown space-sweep option '" + option + "'");
        }
    }
    return options;
}

void PrintUsage() {
    std::cerr << "Usage:\n"
              << "  pmg_cli --synthetic\n"
              << "  pmg_cli --bvh path/to/file.bvh\n"
              << "  pmg_cli --inspect-bvh-heading path/to/file.bvh\n"
              << "  pmg_cli --inspect-bvh-geometry path/to/file.bvh\n"
              << "  pmg_cli --list-bvh-joints path/to/file.bvh\n"
              << "  pmg_cli --debug-bvh-pair path/to/file.bvh LeftHip RightHip\n"
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
              << "      [--dtw-refine]\n"
              << "  pmg_cli --validate-graph graph_spec.txt [--cycle-joint name]\n"
              << "      [--tgood X --tbad Y --source-samples N --target-samples N --seed S]\n"
              << "      [--min-edge-samples N] [--min-good-fraction F] [--assert-no-regression]\n"
              << "  pmg_cli --random-walk graph_spec.txt [--seconds S] [--walk-seed N]\n"
              << "      [--min-transitions N] [--max-pop-ratio X] [builder/registration opts]\n"
              << "  pmg_cli --goto graph_spec.txt x z [--seconds S] [--tolerance D]\n"
              << "      [--max-pop-ratio X] [builder/registration opts]\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--synthetic") {
            return RunSyntheticDemo();
        }

        if (argc == 3 && std::string(argv[1]) == "--bvh") {
            return PrintBvhSummary(argv[2]);
        }

        if (argc == 3 && std::string(argv[1]) == "--inspect-bvh-heading") {
            return InspectBvhHeading(argv[2]);
        }

        if (argc == 3 && std::string(argv[1]) == "--inspect-bvh-geometry") {
            return InspectBvhGeometry(argv[2]);
        }

        if (argc == 3 && std::string(argv[1]) == "--list-bvh-joints") {
            return ListBvhJoints(argv[2]);
        }

        if (argc == 5 && std::string(argv[1]) == "--debug-bvh-pair") {
            return DebugBvhJointPair(argv[2], argv[3], argv[4]);
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
