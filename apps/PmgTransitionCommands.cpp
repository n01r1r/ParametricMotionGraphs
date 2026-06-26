#include "PmgCommandModules.h"

#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/GraphIo.h"
#include "pmg/GraphSpec.h"
#include "pmg/MotionDistance.h"
#include "pmg/MotionRegistration.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/PmgBuilder.h"
#include "pmg/PoseBlend.h"
#include "pmg/RuntimeController.h"
#include "pmg/SkeletonCompatibility.h"
#include "pmg/TransitionWindow.h"

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

    // Starting thresholds remain corpus- and metric-specific because raw sums
    // scale with point count and configured weights. See --calibrate-thresholds.
    const pmg::PmgBuilderConfig builder_defaults;
    const float good_threshold =
        builder_defaults.good_transition_threshold;
    const float bad_threshold =
        builder_defaults.bad_transition_threshold;

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
        transition.distance <= good_threshold   ? "GOOD"
        : transition.distance >= bad_threshold  ? "BAD"
                                                : "NEUTRAL";

    std::cout << "source_frame=" << transition.source_frame << "\n";
    std::cout << "target_frame=" << transition.target_frame << "\n";
    std::cout << "source_phase=" << transition.source_phase << "\n";
    std::cout << "target_phase=" << transition.target_phase << "\n";
    std::cout << "distance=" << transition.distance << "\n";
    std::cout << "TGOOD=" << good_threshold << " TBAD=" << bad_threshold << "\n";
    std::cout << "classification=" << classification << "\n";
    std::cout << "alignment_yaw=" << transition.alignment.yaw << "\n";
    std::cout << "alignment_dx=" << transition.alignment.dx << "\n";
    std::cout << "alignment_dz=" << transition.alignment.dz << "\n";
    std::cout << "NOTE: distance is a raw weighted squared point-distance sum "
                 "in the BVH's native units; calibrate thresholds per corpus, "
                 "window, skeleton, and weighting.\n";
    return 0;
}

std::string FrameList(const std::vector<int>& frames) {
    std::ostringstream output;
    for (std::size_t index = 0; index < frames.size(); ++index) {
        if (index > 0) {
            output << ',';
        }
        output << frames[index];
    }
    return output.str();
}

float PoseHeading(const pmg::Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const pmg::Vec3 forward = pmg::Rotate(
        pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

float WrappedAngleDelta(float first, float second) {
    float delta = first - second;
    while (delta > pmg::kPi) {
        delta -= 2.0f * pmg::kPi;
    }
    while (delta < -pmg::kPi) {
        delta += 2.0f * pmg::kPi;
    }
    return delta;
}

struct SeamMetrics {
    float max_mean_joint_step = 0.0f;
    float max_root_velocity_delta = 0.0f;
    float max_facing_velocity_delta = 0.0f;
};

SeamMetrics MeasureSyntheticSeam(
    const pmg::Skeleton& skeleton,
    const pmg::MotionClip& source_clip,
    const pmg::MotionClip& target_clip,
    const pmg::TransitionFrameWindows& windows,
    const pmg::RigidTransform2D& alignment) {
    std::vector<pmg::Pose> poses;
    poses.reserve(windows.source.sampled_frames.size() + 2);

    const int source_before = std::max(
        0, windows.source.sampled_frames.front() - 1);
    poses.push_back(source_clip.frames[source_before]);

    const std::size_t blend_frame_count =
        windows.source.sampled_frames.size();
    for (std::size_t index = 0; index < blend_frame_count; ++index) {
        const float linear_alpha =
            blend_frame_count <= 1
                ? 1.0f
                : static_cast<float>(index) /
                      static_cast<float>(blend_frame_count - 1);
        const float alpha =
            linear_alpha * linear_alpha * (3.0f - 2.0f * linear_alpha);
        const pmg::Pose target_pose = alignment.Apply(
            target_clip.frames[windows.target.sampled_frames[index]]);
        poses.push_back(pmg::BlendPose(
            source_clip.frames[windows.source.sampled_frames[index]],
            target_pose, alpha));
    }

    const int target_after = std::min(
        target_clip.NumFrames() - 1,
        windows.target.sampled_frames.back() + 1);
    poses.push_back(alignment.Apply(target_clip.frames[target_after]));

    SeamMetrics metrics;
    std::vector<pmg::Vec3> root_steps;
    std::vector<float> facing_steps;
    for (std::size_t pose_index = 1; pose_index < poses.size(); ++pose_index) {
        const std::vector<pmg::Vec3> previous_positions =
            pmg::ComputeJointWorldPositions(
                skeleton, poses[pose_index - 1]);
        const std::vector<pmg::Vec3> current_positions =
            pmg::ComputeJointWorldPositions(
                skeleton, poses[pose_index]);
        float mean_joint_step = 0.0f;
        for (std::size_t joint = 0; joint < current_positions.size(); ++joint) {
            mean_joint_step +=
                (current_positions[joint] - previous_positions[joint]).Norm();
        }
        mean_joint_step /= static_cast<float>(current_positions.size());
        metrics.max_mean_joint_step =
            std::max(metrics.max_mean_joint_step, mean_joint_step);

        root_steps.push_back(
            poses[pose_index].root_position -
            poses[pose_index - 1].root_position);
        facing_steps.push_back(WrappedAngleDelta(
            PoseHeading(poses[pose_index]),
            PoseHeading(poses[pose_index - 1])));
    }

    for (std::size_t index = 1; index < root_steps.size(); ++index) {
        metrics.max_root_velocity_delta = std::max(
            metrics.max_root_velocity_delta,
            (root_steps[index] - root_steps[index - 1]).Norm());
        metrics.max_facing_velocity_delta = std::max(
            metrics.max_facing_velocity_delta,
            std::abs(WrappedAngleDelta(
                facing_steps[index], facing_steps[index - 1])));
    }
    return metrics;
}

void PrintConventionResult(
    const pmg::Skeleton& skeleton,
    const pmg::MotionClip& source_clip,
    const pmg::MotionClip& target_clip,
    const pmg::DistanceGridConfig& config,
    pmg::TransitionWindowConvention convention) {
    const pmg::OptimalTransition transition =
        pmg::MotionDistance::FindOptimalTransitionForConvention(
            skeleton, source_clip, target_clip, config, convention);
    if (transition.source_frame < 0) {
        throw std::runtime_error(
            "CompareTransitionConventions: empty distance grid");
    }
    const pmg::TransitionFrameWindows windows =
        pmg::ResolveTransitionFrameWindows(
            source_clip.NumFrames(), target_clip.NumFrames(),
            transition.source_frame, transition.target_frame,
            config.window_size, convention);
    const SeamMetrics seam = MeasureSyntheticSeam(
        skeleton, source_clip, target_clip, windows, transition.alignment);
    const char* name = pmg::TransitionWindowConventionName(convention);

    std::cout << "=== " << name << " ===\n";
    std::cout << "source_reference_frame=" << transition.source_frame << "\n";
    std::cout << "target_reference_frame=" << transition.target_frame << "\n";
    std::cout << "source_support="
              << FrameList(windows.source.sampled_frames) << "\n";
    std::cout << "target_support="
              << FrameList(windows.target.sampled_frames) << "\n";
    std::cout << "distance=" << transition.distance << "\n";
    std::cout << "alignment_yaw=" << transition.alignment.yaw << "\n";
    std::cout << "alignment_dx=" << transition.alignment.dx << "\n";
    std::cout << "alignment_dz=" << transition.alignment.dz << "\n";
    std::cout << "synthetic_max_mean_joint_step="
              << seam.max_mean_joint_step << "\n";
    std::cout << "synthetic_max_root_velocity_delta="
              << seam.max_root_velocity_delta << "\n";
    std::cout << "synthetic_max_facing_velocity_delta="
              << seam.max_facing_velocity_delta << "\n";
}

void PrintLegacyV7Reinterpretation(
    const pmg::Skeleton& skeleton,
    const pmg::MotionClip& source_clip,
    const pmg::MotionClip& target_clip,
    const pmg::DistanceGridConfig& config) {
    const pmg::OptimalTransition stored =
        pmg::MotionDistance::FindOptimalTransitionForConvention(
            skeleton, source_clip, target_clip, config,
            pmg::TransitionWindowConvention::kKovarDirectional);
    if (stored.source_frame < 0) {
        throw std::runtime_error(
            "PrintLegacyV7Reinterpretation: empty distance grid");
    }
    const pmg::TransitionFrameWindows offline_windows =
        pmg::ResolveTransitionFrameWindows(
            source_clip.NumFrames(), target_clip.NumFrames(),
            stored.source_frame, stored.target_frame, config.window_size,
            pmg::TransitionWindowConvention::kKovarDirectional);
    const pmg::TransitionFrameWindows runtime_windows =
        pmg::ResolveTransitionFrameWindows(
            source_clip.NumFrames(), target_clip.NumFrames(),
            stored.source_frame, stored.target_frame, config.window_size,
            pmg::TransitionWindowConvention::kPmgCentered);

    const pmg::PointCloud runtime_source_cloud =
        pmg::MotionDistance::BuildPointCloudFromFirstFrame(
            skeleton, source_clip,
            runtime_windows.source.first_unclamped_frame,
            config.window_size, config.weighting);
    const pmg::PointCloud runtime_target_cloud =
        pmg::MotionDistance::BuildPointCloudFromFirstFrame(
            skeleton, target_clip,
            runtime_windows.target.first_unclamped_frame,
            config.window_size, config.weighting);
    const pmg::AlignedDistanceResult runtime_alignment =
        pmg::MotionDistance::AlignedPointCloudDistance(
            runtime_source_cloud, runtime_target_cloud);
    const SeamMetrics runtime_seam = MeasureSyntheticSeam(
        skeleton, source_clip, target_clip, runtime_windows,
        runtime_alignment.alignment);

    const bool source_support_matches =
        offline_windows.source.sampled_frames ==
        runtime_windows.source.sampled_frames;
    const bool target_support_matches =
        offline_windows.target.sampled_frames ==
        runtime_windows.target.sampled_frames;

    std::cout << "=== legacy_v7_directional_phase_reinterpreted_centered ===\n";
    std::cout << "stored_source_reference_frame="
              << stored.source_frame << "\n";
    std::cout << "stored_target_reference_frame="
              << stored.target_frame << "\n";
    std::cout << "offline_source_support="
              << FrameList(offline_windows.source.sampled_frames) << "\n";
    std::cout << "offline_target_support="
              << FrameList(offline_windows.target.sampled_frames) << "\n";
    std::cout << "runtime_source_support="
              << FrameList(runtime_windows.source.sampled_frames) << "\n";
    std::cout << "runtime_target_support="
              << FrameList(runtime_windows.target.sampled_frames) << "\n";
    std::cout << "source_support_matches="
              << static_cast<int>(source_support_matches) << "\n";
    std::cout << "target_support_matches="
              << static_cast<int>(target_support_matches) << "\n";
    std::cout << "offline_scored_distance=" << stored.distance << "\n";
    std::cout << "runtime_reinterpreted_distance="
              << runtime_alignment.distance << "\n";
    std::cout << "runtime_synthetic_max_mean_joint_step="
              << runtime_seam.max_mean_joint_step << "\n";
    std::cout << "runtime_synthetic_max_root_velocity_delta="
              << runtime_seam.max_root_velocity_delta << "\n";
    std::cout << "runtime_synthetic_max_facing_velocity_delta="
              << runtime_seam.max_facing_velocity_delta << "\n";
}

int CompareTransitionConventions(
    const std::string& source_path,
    const std::string& target_path) {
    const pmg::BvhData source = pmg::BvhLoader::Load(source_path);
    const pmg::BvhData target = pmg::BvhLoader::Load(target_path);
    pmg::RequireSkeletonCompatible(
        source.skeleton, target.skeleton,
        "CompareTransitionConventions",
        /*offset_tolerance=*/1.0e-4f);

    const pmg::DistanceGridConfig config = TransitionGridConfig();
    std::cout << "source_bvh=" << source_path << "\n";
    std::cout << "target_bvh=" << target_path << "\n";
    std::cout << "window_sample_count=" << config.window_size << "\n";
    std::cout << "window_span_seconds="
              << pmg::TransitionWindowSpanSeconds(
                     config.window_size, source.clip.frames_per_second)
              << "\n";
    std::cout << "runtime_blend_duration_seconds="
              << pmg::TransitionWindowSpanSeconds(
                     config.window_size, source.clip.frames_per_second)
              << "\n";
    PrintConventionResult(
        source.skeleton, source.clip, target.clip, config,
        pmg::TransitionWindowConvention::kKovarDirectional);
    PrintConventionResult(
        source.skeleton, source.clip, target.clip, config,
        pmg::TransitionWindowConvention::kPmgCentered);
    PrintLegacyV7Reinterpretation(
        source.skeleton, source.clip, target.clip, config);
    std::cout << "NOTE: synthetic seam metrics compare one aligned cubic blend "
                 "plus one boundary frame. They are diagnostics, not perceptual "
                 "quality claims.\n";
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

    std::cout << "\n=== suggested thresholds (weighted squared-sum world-unit space) ===\n";
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
    std::cout << "NOTE: distances are raw weighted squared sums in the BVH's native units "
                 "(no loader scaling); scale depends on point count and weights.\n";
    std::cout << "NOTE: non-looping clips show large self/same distance over the late->early window; expected.\n";
    return 0;
}

}  // namespace

namespace pmgcli {

std::optional<int> TryRunTransitionCommand(int argc, char** argv) {
    const std::string command = argc > 1 ? argv[1] : "";
    if (command == "--inspect-transition" && argc == 4) {
        return InspectTransition(argv[2], argv[3]);
    }
    if (command == "--compare-transition-conventions" && argc == 4) {
        return CompareTransitionConventions(argv[2], argv[3]);
    }
    if (command == "--dump-distance-grid" && argc == 5) {
        return DumpDistanceGrid(argv[2], argv[3], argv[4]);
    }
    if (command == "--calibrate-thresholds" && argc == 3) {
        return CalibrateThresholds(argv[2]);
    }
    if (command == "--calibrate-thresholds" && argc == 4) {
        return CalibrateThresholds(argv[2], argv[3]);
    }
    return std::nullopt;
}

}  // namespace pmgcli
