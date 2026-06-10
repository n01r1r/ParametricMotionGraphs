#include "pmg/BvhLoader.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/MotionDistance.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/RuntimeController.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
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

    pmg::RuntimeController controller(graph);
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
    std::cout << "alignment_yaw=" << transition.alignment.theta << "\n";
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
// distance scale (BVH offsets are x10, so distances are not the paper's literal
// numbers). Additive: no core/runtime change.
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
              << "  pmg_cli --calibrate-thresholds bvh_directory [manifest.txt]\n";
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

        PrintUsage();
        return 2;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }
}
