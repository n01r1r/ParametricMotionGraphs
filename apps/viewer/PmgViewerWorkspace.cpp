#include "PmgViewerWorkspace.h"

#include "GraphAuthoringModel.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include "pmg/FootLocking.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/MathTypes.h"
#include "pmg/RootCanonicalization.h"
#include "pmg/SkeletonCompatibility.h"

#ifndef PMG_BVH_DIRECTORY
#define PMG_BVH_DIRECTORY "BVH"
#endif

namespace pmgviewer {

namespace {
constexpr float kEpsilon = 1.0e-6f;
constexpr int kSteeringCurveSamples = 24;  // turn-rate plot resolution

glm::vec3 ToGlm(const pmg::Vec3& v) { return glm::vec3(v.x, v.y, v.z); }

// Limb girth (capsule radius) by joint role, so the skeleton reads as a human
// mannequin instead of uniform wire. `unit` carries the display + skeleton scale
// so resizing keeps proportions. Matched against the lowercased child-joint name.
float MannequinBoneGirth(const std::string& child_joint_lower, float unit) {
    auto has = [&](const char* token) {
        return child_joint_lower.find(token) != std::string::npos;
    };
    float fraction = 0.20f;  // default limb
    if (has("hip") || has("pelvis") || has("spine") || has("chest") ||
        has("torso") || has("abdomen") || has("back") || has("belly")) {
        fraction = 0.45f;  // trunk
    } else if (has("head") || has("neck") || has("thigh") || has("upleg") ||
               has("upperleg") || has("femur") || has("shoulder") ||
               has("collar") || has("clavicle")) {
        fraction = 0.30f;  // head / proximal limb
    } else if (has("hand") || has("finger") || has("thumb") || has("toe") ||
               has("ball") || has("foot") || has("wrist") || has("ankle")) {
        fraction = 0.12f;  // extremity
    }
    return fraction * unit;
}

glm::vec3 TransitionMarkerColor(int source_node, int target_node) {
    const std::array<glm::vec3, 6> kEdgeColors = {
        glm::vec3(0.94f, 0.45f, 0.14f),
        glm::vec3(0.86f, 0.23f, 0.45f),
        glm::vec3(0.61f, 0.35f, 0.89f),
        glm::vec3(0.13f, 0.65f, 0.82f),
        glm::vec3(0.18f, 0.72f, 0.45f),
        glm::vec3(0.93f, 0.72f, 0.17f),
    };
    const int edge_key = source_node * 31 + target_node;
    const std::size_t edge_index =
        static_cast<std::size_t>(std::abs(edge_key));
    return kEdgeColors[edge_index % kEdgeColors.size()];
}

// Wrap a free-running phase into [0,1). The playback clock is stored directly
// in phase now, so this is just a fractional wrap (no division by a
// parameter-dependent duration, which is what used to teleport the pose when
// the blend parameter changed).
float WrapPhase(float phase) {
    phase -= std::floor(phase);
    if (phase < 0.0f) {
        phase += 1.0f;
    }
    return phase;
}

// Normalized [0,1] phase of a frame index within a clip.
float FramePhaseLocal(int frame_index, int frame_count) {
    if (frame_count <= 1) {
        return 0.0f;
    }
    return static_cast<float>(frame_index) / static_cast<float>(frame_count - 1);
}

// Jet-style colormap: t=0 -> blue (low distance, good), t=1 -> red (high, bad).
ImU32 HeatColor(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    const float r = std::clamp(1.5f - std::fabs(4.0f * t - 3.0f), 0.0f, 1.0f);
    const float g = std::clamp(1.5f - std::fabs(4.0f * t - 2.0f), 0.0f, 1.0f);
    const float b = std::clamp(1.5f - std::fabs(4.0f * t - 1.0f), 0.0f, 1.0f);
    return IM_COL32(static_cast<int>(r * 255.0f),
                    static_cast<int>(g * 255.0f),
                    static_cast<int>(b * 255.0f), 255);
}

}  // namespace

void PmgViewerWorkspace::Initialize(const std::string& artifact_path) {
    DiscoverBvhFiles();
    DiscoverSpecFiles();
    if (!bvh_files_.empty()) {
        LoadClip(0);
    }
    if (!artifact_path.empty()) {
        // First arg may be a built .pmg artifact or a .pmg_spec recipe; the spec
        // path builds in-process (same as the in-GUI picker / CLI --build-graph)
        // so the documented `pmg_viewer specs/foo.pmg_spec` launch works.
        if (std::filesystem::path(artifact_path).extension() == ".pmg_spec") {
            BuildArtifactFromSpec(artifact_path);
        } else {
            LoadGraphArtifact(artifact_path);
        }
    }
}

void PmgViewerWorkspace::DiscoverBvhFiles() {
    bvh_files_.clear();
    const std::filesystem::path directory(PMG_BVH_DIRECTORY);
    std::error_code error;
    if (!std::filesystem::exists(directory, error)) {
        status_message_ = std::string("BVH directory not found: ") + directory.string();
        return;
    }
    for (const auto& entry : std::filesystem::directory_iterator(directory, error)) {
        if (entry.is_regular_file() && entry.path().extension() == ".bvh") {
            bvh_files_.push_back(entry.path());
        }
    }
    std::sort(bvh_files_.begin(), bvh_files_.end());
}

void PmgViewerWorkspace::DiscoverSpecFiles() {
    spec_files_.clear();
    // Scan the curated specs/ dir plus the CMU experiment specs so the multi-node
    // cmubvh graphs are verifiable from the in-GUI spec picker. experiments/cmu
    // sits next to specs/ under the repo root, so derive it from the existing
    // PMG_SPEC_DIRECTORY macro rather than adding a second build define. Example
    // BVH paths resolve relative to each spec's own directory (GraphSpec.cpp), so
    // the CMU corpus loads correctly even though it lives outside PMG_BVH_DIRECTORY.
    // ponytail: two explicit dirs, not a recursive walk -- recursion would also
    // surface specs/fixtures/ stress specs that are intentionally off the picker.
    const std::filesystem::path repo_root =
        std::filesystem::path(PMG_SPEC_DIRECTORY).parent_path();
    const std::filesystem::path directories[] = {
        std::filesystem::path(PMG_SPEC_DIRECTORY),
        repo_root / "experiments" / "cmu",
    };
    for (const std::filesystem::path& directory : directories) {
        std::error_code error;
        if (!std::filesystem::exists(directory, error)) {
            continue;
        }
        // Sort each directory's contribution independently so the curated specs/
        // group stays first (preserving the default picker selection) with the
        // CMU specs appended after it.
        const std::size_t group_begin = spec_files_.size();
        for (const auto& entry :
             std::filesystem::directory_iterator(directory, error)) {
            if (entry.is_regular_file() &&
                entry.path().extension() == ".pmg_spec") {
                spec_files_.push_back(entry.path());
            }
        }
        std::sort(spec_files_.begin() +
                      static_cast<std::ptrdiff_t>(group_begin),
                  spec_files_.end());
    }
}

float PmgViewerWorkspace::ComputeGroundOffset(
    const pmg::Skeleton& skeleton, const pmg::MotionClip& clip) {
    float min_y = std::numeric_limits<float>::infinity();
    for (const pmg::Pose& frame : clip.frames) {
        if (frame.NumJoints() != skeleton.NumJoints()) {
            continue;
        }
        for (const pmg::Vec3& position : pmg::ComputeJointWorldPositions(skeleton, frame)) {
            min_y = std::min(min_y, position.y);
        }
    }
    if (!std::isfinite(min_y)) {
        return 0.0f;
    }
    return -min_y;  // raise the lowest point onto y = 0
}

void PmgViewerWorkspace::LoadClip(int file_index) {
    if (file_index < 0 || file_index >= static_cast<int>(bvh_files_.size())) {
        return;
    }
    try {
        const pmg::BvhData data = pmg::BvhLoader::Load(bvh_files_[file_index].string());
        skeleton_ = data.skeleton;
        clip_ = data.clip;
        ground_offset_ = ComputeGroundOffset(skeleton_, clip_);
        current_phase_ = 0.0f;
        selected_file_index_ = file_index;
        mode_ = ViewerPlaybackMode::ClipPlayback;
        status_message_ = "Loaded " + bvh_files_[file_index].filename().string();
    } catch (const std::exception& error) {
        status_message_ = std::string("Load failed: ") + error.what();
    }
}

bool PmgViewerWorkspace::ParametricBlendActive() const {
    return mode_ == ViewerPlaybackMode::ParametricBlend && pmg_space_ready_;
}

bool PmgViewerWorkspace::GraphRuntimeActive() const {
    return mode_ == ViewerPlaybackMode::GraphRuntime && runtime_.Ready();
}

void PmgViewerWorkspace::ResetPlayback() {
    current_phase_ = 0.0f;
    playback_speed_ = 1.0f;
    playing_ = true;
}

const pmg::Skeleton& PmgViewerWorkspace::ActiveSkeleton() const {
    if (ParametricBlendActive() || GraphRuntimeActive()) {
        return pmg_skeleton_;
    }
    return skeleton_;
}

float PmgViewerWorkspace::ActiveReferenceDuration() const {
    // The blended cycle inherits its timing from the generated preview clip, so
    // the phase timeline matches what is rendered (BlendedDurationSeconds), not
    // an arbitrary example's duration.
    if (ParametricBlendActive() && pmg_preview_clip_.NumFrames() > 0) {
        return pmg_preview_clip_.DurationSeconds();
    }
    if (clip_.NumFrames() > 0) {
        return clip_.DurationSeconds();
    }
    return 0.0f;
}

pmg::Pose PmgViewerWorkspace::CurrentPose() const {
    if (ParametricBlendActive()) {
        // Sample the cached GenerateClip output (kept fresh by Update) by phase,
        // instead of blending absolute root positions per frame.
        if (pmg_preview_clip_.NumFrames() == 0) {
            return pmg::Pose{};
        }
        pmg::Pose pose = pmg_preview_clip_.SampleNormalizedPhase(current_phase_);
        if (pmg_preview_in_place_) {
            // Lock the horizontal root to the cycle start; keep vertical (y).
            const pmg::Vec3& anchor =
                pmg_preview_clip_.frames.front().root_position;
            pose.root_position.x = anchor.x;
            pose.root_position.z = anchor.z;
        }
        return pose;
    }
    if (clip_.NumFrames() == 0) {
        return pmg::Pose{};
    }
    return clip_.SampleNormalizedPhase(current_phase_);
}

pmg::Pose PmgViewerWorkspace::DisplayPose() const {
    if (GraphRuntimeActive()) {
        return CurrentPose();
    }
    if (skeleton_pose_mode_ == ViewerSkeletonPoseMode::Current) {
        return CurrentPose();
    }
    if (clip_.NumFrames() == 0) {
        return CurrentPose();
    }
    if (skeleton_pose_mode_ == ViewerSkeletonPoseMode::Frame0) {
        return clip_.frames.front();
    }

    pmg::Pose rest_pose;
    rest_pose.local_rotations.assign(
        static_cast<std::size_t>(ActiveSkeleton().NumJoints()),
        pmg::Quaternion::Identity());
    rest_pose.root_position = clip_.frames.front().root_position;
    return rest_pose;
}

void PmgViewerWorkspace::RebuildScene(const pmg::Pose& pose) {
    const pmg::Skeleton& skeleton = ActiveSkeleton();
    if (pose.NumJoints() != skeleton.NumJoints() || skeleton.NumJoints() == 0) {
        return;
    }

    const float ground_offset =
        (ParametricBlendActive() || GraphRuntimeActive()) ? pmg_ground_offset_ : ground_offset_;
    const std::vector<glm::vec3> world_positions =
        PoseWorldPositions(pose, skeleton, ground_offset);
    const std::vector<pmg::JointWorldState> world_states =
        pmg::ComputeForwardKinematics(skeleton, pose);

    glm::vec3 centroid(0.0f);
    scene_.joints.clear();
    scene_.bones.clear();
    scene_.joints.reserve(world_positions.size());
    const bool separate_end_sites = show_end_sites_separately_ && !hide_end_sites_;
    const float axis_length = 0.35f * display_scale_ * skeleton_scale_;
    const float point_radius = 0.06f * display_scale_ * skeleton_scale_;
    const float end_site_radius = 0.045f * display_scale_ * skeleton_scale_;
    const float line_radius = 0.018f * display_scale_ * skeleton_scale_;
    for (int joint_index = 0; joint_index < skeleton.NumJoints(); ++joint_index) {
        const pmg::Joint& joint = skeleton.joints[static_cast<std::size_t>(joint_index)];
        const bool is_end_site = joint.channels.empty() && joint.parent_index >= 0;
        const std::string joint_name_lower = [&joint] {
            std::string lower = joint.name;
            std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) {
                return static_cast<char>(std::tolower(ch));
            });
            return lower;
        }();
        const bool is_likely_foot =
            mark_likely_foot_joints_ &&
            (joint_name_lower.find("foot") != std::string::npos ||
             joint_name_lower.find("ankle") != std::string::npos ||
             joint_name_lower.find("toe") != std::string::npos ||
             joint_name_lower.find("ball") != std::string::npos);
        const glm::vec3 position = world_positions[static_cast<std::size_t>(joint_index)];
        centroid += position;
        const float bone_girth =
            MannequinBoneGirth(joint_name_lower, display_scale_ * skeleton_scale_);

        if (is_end_site) {
            if (!hide_end_sites_ && separate_end_sites) {
                constexpr glm::vec3 kEndSiteColor(0.95f, 0.55f, 0.15f);
                scene_.diagnostic_points.push_back({position, kEndSiteColor, end_site_radius});
                if (joint.parent_index >= 0) {
                    scene_.diagnostic_lines.push_back({
                        world_positions[static_cast<std::size_t>(joint.parent_index)],
                        position,
                        kEndSiteColor,
                        line_radius,
                    });
                }
            } else if (!hide_end_sites_) {
                scene_.joints.push_back(position);
                if (joint.parent_index >= 0) {
                    scene_.bones.push_back({
                        world_positions[static_cast<std::size_t>(joint.parent_index)],
                        position, bone_girth});
                }
            }
        } else {
            scene_.joints.push_back(position);
            if (joint.parent_index >= 0) {
                scene_.bones.push_back({
                    world_positions[static_cast<std::size_t>(joint.parent_index)],
                    position, bone_girth});
            }
        }

        if (draw_local_joint_axes_) {
            const pmg::JointWorldState& state =
                world_states[static_cast<std::size_t>(joint_index)];
            const pmg::Vec3 axis_x_native = pmg::Rotate(state.rotation, {1.0f, 0.0f, 0.0f});
            const pmg::Vec3 axis_y_native = pmg::Rotate(state.rotation, {0.0f, 1.0f, 0.0f});
            const pmg::Vec3 axis_z_native = pmg::Rotate(state.rotation, {0.0f, 0.0f, 1.0f});
            scene_.diagnostic_lines.push_back({
                position,
                position + glm::vec3(
                    axis_x_native.x, axis_x_native.y, axis_x_native.z) * axis_length,
                glm::vec3(0.95f, 0.30f, 0.25f),
                line_radius,
            });
            scene_.diagnostic_lines.push_back({
                position,
                position + glm::vec3(
                    axis_y_native.x, axis_y_native.y, axis_y_native.z) * axis_length,
                glm::vec3(0.25f, 0.90f, 0.35f),
                line_radius,
            });
            scene_.diagnostic_lines.push_back({
                position,
                position + glm::vec3(
                    axis_z_native.x, axis_z_native.y, axis_z_native.z) * axis_length,
                glm::vec3(0.25f, 0.55f, 0.95f),
                line_radius,
            });
        }

        if (is_likely_foot) {
            scene_.diagnostic_points.push_back({
                position, glm::vec3(0.95f, 0.80f, 0.20f), point_radius * 1.25f});
        }
    }
    if (!world_positions.empty()) {
        centroid /= static_cast<float>(world_positions.size());
    }

    // Goto target gizmo: ground sphere plus a beacon roughly character-tall
    // (native target back into display space, same transform as the root).
    scene_.marker_points.clear();
    scene_.marker_lines.clear();
    scene_.diagnostic_points.clear();
    scene_.diagnostic_lines.clear();
    scene_.has_floor_origin = false;

    const ViewerRuntimeSnapshot runtime = runtime_.Snapshot();
    const auto& path_points = runtime.runtime_path_points;
    const auto& transition_markers = runtime.transition_markers;
    if (GraphRuntimeActive() && show_graph_path_trail_ &&
        path_points.size() >= 2) {
        constexpr glm::vec3 kPathTrailColor(0.10f, 0.70f, 0.95f);
        const float trail_height = 0.05f * display_scale_;
        const float trail_radius = 0.035f * display_scale_;
        for (std::size_t index = 1; index < path_points.size();
             ++index) {
            const glm::vec2& previous = path_points[index - 1];
            const glm::vec2& current = path_points[index];
            scene_.diagnostic_lines.push_back({
                glm::vec3(previous.x * display_scale_, trail_height,
                          previous.y * display_scale_),
                glm::vec3(current.x * display_scale_, trail_height,
                          current.y * display_scale_),
                kPathTrailColor,
                trail_radius,
            });
        }
    }

    if (GraphRuntimeActive() && show_graph_transition_markers_) {
        const float marker_height = 0.25f * display_scale_;
        const float marker_radius = 0.065f * display_scale_;
        const float line_radius = 0.022f * display_scale_;
        for (const TransitionTraceMarker& marker : transition_markers) {
            const glm::vec3 color =
                TransitionMarkerColor(marker.source_node, marker.target_node);
            const glm::vec3 ground_position(
                marker.root_position.x * display_scale_,
                0.08f * display_scale_,
                marker.root_position.y * display_scale_);
            scene_.diagnostic_points.push_back(
                {ground_position, color, marker_radius});
            scene_.diagnostic_lines.push_back({
                ground_position,
                ground_position + glm::vec3(0.0f, marker_height, 0.0f),
                color,
                line_radius,
            });
        }
    }

    if (path_following_active_ && GraphRuntimeActive()) {
        constexpr glm::vec3 kPathColor(0.95f, 0.55f, 0.15f);
        const float y = 0.08f * display_scale_;
        for (std::size_t index = 1; index < path_waypoints_.size(); ++index) {
            const glm::vec2& start = path_waypoints_[index - 1];
            const glm::vec2& end = path_waypoints_[index];
            scene_.diagnostic_lines.push_back({
                {start.x * display_scale_, y, start.y * display_scale_},
                {end.x * display_scale_, y, end.y * display_scale_},
                kPathColor, 0.04f * display_scale_});
        }
    }

    if (goto_active_ && GraphRuntimeActive()) {
        const glm::vec3 base(goto_target_.x * display_scale_, 0.0f,
                             goto_target_.y * display_scale_);
        const glm::vec3 top = base + glm::vec3(0.0f, 1.8f * display_scale_, 0.0f);
        scene_.marker_points.push_back(base);
        scene_.marker_lines.push_back({base, top});
    }

    AppendRootCanonicalizationMarkers(pose);
    AppendPathPreview();
    AppendParamSweepPaths();
    AppendOriginMarker();

    scene_.focus_point = centroid;
}

void PmgViewerWorkspace::AppendRootCanonicalizationMarkers(
    const pmg::Pose& current_pose) {
    if (!GraphRuntimeActive() || !show_root_canonicalization_markers_) {
        return;
    }
    constexpr glm::vec3 kRawColor(0.95f, 0.25f, 0.18f);
    constexpr glm::vec3 kNormalizedColor(0.18f, 0.80f, 0.35f);
    constexpr glm::vec3 kTrajectoryColor(0.95f, 0.75f, 0.18f);
    constexpr glm::vec3 kRuntimeColor(0.20f, 0.65f, 1.00f);
    const float y = 0.12f * display_scale_;
    const float point_radius = 0.06f * display_scale_;
    const float line_radius = 0.018f * display_scale_;
    const glm::vec3 canonical_origin(0.0f, y, 0.0f);

    scene_.diagnostic_points.push_back(
        {canonical_origin, kNormalizedColor, point_radius * 1.4f});
    for (const RootCanonicalizationMarker& marker :
         root_canonicalization_markers_) {
        const glm::vec3 raw(marker.raw_start.x * display_scale_, y,
                            marker.raw_start.z * display_scale_);
        scene_.diagnostic_points.push_back({raw, kRawColor, point_radius});
        scene_.diagnostic_lines.push_back({
            raw + glm::vec3(-point_radius, 0.0f, 0.0f),
            raw + glm::vec3(point_radius, 0.0f, 0.0f),
            kRawColor,
            line_radius,
        });
        scene_.diagnostic_lines.push_back({
            raw + glm::vec3(0.0f, 0.0f, -point_radius),
            raw + glm::vec3(0.0f, 0.0f, point_radius),
            kRawColor,
            line_radius,
        });
        if (!show_anchor_root_trajectories_) {
            continue;
        }
        for (std::size_t i = 1; i < marker.normalized_trajectory.size(); ++i) {
            const pmg::Vec3& previous = marker.normalized_trajectory[i - 1];
            const pmg::Vec3& current = marker.normalized_trajectory[i];
            scene_.diagnostic_lines.push_back({
                glm::vec3(previous.x * display_scale_, y,
                          previous.z * display_scale_),
                glm::vec3(current.x * display_scale_, y,
                          current.z * display_scale_),
                kTrajectoryColor,
                line_radius,
            });
        }
    }

    scene_.diagnostic_points.push_back({
        glm::vec3(current_pose.root_position.x * display_scale_,
                  0.22f * display_scale_,
                  current_pose.root_position.z * display_scale_),
        kRuntimeColor,
        point_radius * 1.5f,
    });
}

std::vector<glm::vec3> PmgViewerWorkspace::PoseWorldPositions(
    const pmg::Pose& pose, const pmg::Skeleton& skeleton,
    float ground_offset) const {
    const std::vector<pmg::Vec3> positions =
        pmg::ComputeJointWorldPositions(skeleton, pose);

    // Pass 1: lift onto the ground plane and find the horizontal centroid that
    // the skeleton scale pivots around.
    std::vector<glm::vec3> world_positions(positions.size());
    glm::vec2 horizontal_center(0.0f);
    for (std::size_t i = 0; i < positions.size(); ++i) {
        world_positions[i] = glm::vec3(positions[i].x,
                                       positions[i].y + ground_offset,
                                       positions[i].z);
        horizontal_center += glm::vec2(world_positions[i].x, world_positions[i].z);
    }
    if (!world_positions.empty()) {
        horizontal_center /= static_cast<float>(world_positions.size());
    }

    // Pass 2: two distinct scales with different pivots.
    //   display_scale_  (native -> display): applies to the WHOLE world about the
    //     origin, so the root translation scales together with the skeleton.
    //     Pivoting this on the per-frame centroid is what made locomotion look
    //     like a treadmill -- the body was display-sized while the root motion
    //     stayed native-sized, pinning the character at the centre.
    //   skeleton_scale_ (user size tweak): pivots on the horizontal centroid and
    //     anchors vertically at y = 0, so resizing the body neither slides it
    //     across the floor nor lifts the feet.
    const float render_scale = display_scale_ * skeleton_scale_;
    for (glm::vec3& world_position : world_positions) {
        world_position.x = display_scale_ *
            (horizontal_center.x + (world_position.x - horizontal_center.x) * skeleton_scale_);
        world_position.z = display_scale_ *
            (horizontal_center.y + (world_position.z - horizontal_center.y) * skeleton_scale_);
        world_position.y *= render_scale;
    }
    return world_positions;
}

void PmgViewerWorkspace::AppendPathPreview() {
    if (!path_preview_enabled_) {
        return;
    }
    // Pick the clip actually being played: the blended preview in blend mode,
    // the loaded clip in clip mode. Graph runtime is controller-driven and
    // already has its own path trail, so it is skipped.
    const pmg::MotionClip* clip = nullptr;
    const pmg::Skeleton* skeleton = nullptr;
    float ground_offset = 0.0f;
    if (ParametricBlendActive() && pmg_preview_clip_.NumFrames() > 1) {
        clip = &pmg_preview_clip_;
        skeleton = &pmg_skeleton_;
        ground_offset = pmg_ground_offset_;
    } else if (mode_ == ViewerPlaybackMode::ClipPlayback && clip_.NumFrames() > 1) {
        clip = &clip_;
        skeleton = &skeleton_;
        ground_offset = ground_offset_;
    }
    if (clip == nullptr || skeleton->NumJoints() == 0) {
        return;
    }

    const int ghost_count = std::clamp(path_preview_count_, 2, 12);
    // Roughly two-thirds of the live bone radius: clearly visible as a trail
    // while staying thinner than the active skeleton.
    const float ghost_radius = 0.065f * display_scale_;
    // Translucent, but solid enough to read clearly against the floor; per-ghost
    // alpha falls with the count so a dense trail does not saturate to a wall.
    const float ghost_alpha =
        std::clamp(3.0f / static_cast<float>(ghost_count), 0.45f, 0.85f);
    for (int ghost = 0; ghost < ghost_count; ++ghost) {
        const float phase =
            static_cast<float>(ghost) / static_cast<float>(ghost_count - 1);
        const pmg::Pose ghost_pose = clip->SampleNormalizedPhase(phase);
        if (ghost_pose.NumJoints() != skeleton->NumJoints()) {
            continue;
        }
        // Vivid, saturated blue; the lightness ramps dark->bright so start->end
        // direction reads while the color stays strong (not the washed-out grey of
        // the first pass).
        const float shade = glm::mix(0.55f, 1.0f, phase);
        const glm::vec3 ghost_color(0.10f * shade, 0.40f * shade, 1.0f * shade);
        const std::vector<glm::vec3> ghost_world =
            PoseWorldPositions(ghost_pose, *skeleton, ground_offset);
        for (int joint_index = 0; joint_index < skeleton->NumJoints();
             ++joint_index) {
            const int parent = skeleton->joints[joint_index].parent_index;
            if (parent < 0) {
                continue;
            }
            scene_.diagnostic_lines.push_back({
                ghost_world[static_cast<std::size_t>(parent)],
                ghost_world[static_cast<std::size_t>(joint_index)],
                ghost_color,
                ghost_radius,
                ghost_alpha,
            });
        }
    }
}

void PmgViewerWorkspace::AppendOriginMarker() {
    // Tint the single floor checkerboard cell under the motion's start (cycle
    // phase-0 root) gold. The rest of the floor keeps the grey grid; the shader
    // recolors only the cell containing this point, so no extra geometry is drawn.
    const pmg::MotionClip* clip = nullptr;
    if (ParametricBlendActive() && pmg_preview_clip_.NumFrames() > 0) {
        clip = &pmg_preview_clip_;
    } else if (mode_ == ViewerPlaybackMode::ClipPlayback && clip_.NumFrames() > 0) {
        clip = &clip_;
    }
    if (clip == nullptr) {
        return;
    }
    const pmg::Vec3& start = clip->frames.front().root_position;
    scene_.has_floor_origin = true;
    scene_.floor_origin =
        glm::vec3(start.x * display_scale_, 0.0f, start.z * display_scale_);
}

void PmgViewerWorkspace::AppendParamSweepPaths() {
    if (!show_param_sweep_paths_ || !ParametricBlendActive()) {
        return;
    }
    // Just above the floor and the per-anchor trajectories so the fan of paths
    // reads clearly; translucent so overlapping segments blend.
    const float y = 0.045f * display_scale_;
    const float radius = 0.03f * display_scale_;
    for (const auto& [color, path] : param_sweep_paths_) {
        for (std::size_t i = 1; i < path.size(); ++i) {
            scene_.diagnostic_lines.push_back({
                glm::vec3(path[i - 1].x * display_scale_, y,
                          path[i - 1].z * display_scale_),
                glm::vec3(path[i].x * display_scale_, y,
                          path[i].z * display_scale_),
                color,
                radius,
                0.55f,
            });
        }
    }
}

// --- Goto steering (viewer port of CLI --goto) ------------------------------

void PmgViewerWorkspace::ResetGotoState(const std::string& status) {
    goto_active_ = false;
    goto_desired_parameter_.clear();
    goto_status_ = status;
}

void PmgViewerWorkspace::ResetSteeringState(const std::string& status) {
    steering_.reset();
    ResetGotoState(status);
}

void PmgViewerWorkspace::SetDesiredRuntimeNode(int node) {
    if (node < 0 || node >= runtime_.Graph().NumNodes()) {
        return;
    }
    graph_desired_node_ = node;
    const pmg::ParametricMotionSpace& target_space =
        runtime_.Graph().Node(graph_desired_node_).motion_space;
    if (target_space.ParameterDimension() == 1) {
        graph_desired_parameter_ = std::clamp(
            graph_desired_parameter_,
            target_space.MinParameter().front(),
            target_space.MaxParameter().front());
    }
    ResetGotoState();
}

void PmgViewerWorkspace::PlaceGotoTarget(const glm::vec2& target) {
    goto_target_ = target;
    if (!steering_.has_value()) {
        CalibrateSteering();  // one-time; a few seconds of offline streaming
    }
    if (!steering_.has_value()) {
        return;
    }
    steering_->Reset();
    goto_desired_parameter_.clear();
    goto_active_ = true;
    playing_ = true;
    goto_status_ = "Walking to target.";
}

// Achieved world turn rate when streaming the graph at one held parameter.
// Differs from the example clips' own turn rates: each self-transition plays
// only the target-phase -> source-gate slice per cycle, so the net heading
// advance per hop is that slice's heading change (including sway).
void PmgViewerWorkspace::CalibrateSteering() {
    ResetSteeringState();
    if (!runtime_.Ready() || pmg_examples_.empty()) {
        goto_status_ = "Build a graph first.";
        return;
    }
    try {
        pmg::GoalDirectedLocomotionConfig steering_config;
        steering_config.runtime = runtime_.Config();
        steering_.emplace(
            runtime_.Graph(), pmg_skeleton_, 0, graph_fps_, steering_config);
        const pmg::SteeringCalibration& calibration =
            steering_->Calibration();
        goto_status_ = "Calibrated: achieved turn rates " +
                       std::to_string(calibration.LowestRate()) + " .. " +
                       std::to_string(calibration.HighestRate()) + " rad/s.";
    } catch (const std::exception& error) {
        ResetSteeringState(
            std::string("Calibration failed: ") + error.what());
    }
}

void PmgViewerWorkspace::UpdateGotoSteering(const pmg::Pose& pose) {
    const float dx = goto_target_.x - pose.root_position.x;
    const float dz = goto_target_.y - pose.root_position.z;
    const float distance = std::sqrt(dx * dx + dz * dz);
    if (distance <= goto_tolerance_) {
        if (path_following_active_ &&
            path_waypoint_index_ + 1 < path_waypoints_.size()) {
            ++path_waypoint_index_;
            goto_target_ = path_waypoints_[path_waypoint_index_];
            goto_status_ = "Following path waypoint " +
                std::to_string(path_waypoint_index_ + 1) + "/" +
                std::to_string(path_waypoints_.size()) + ".";
            return;
        }
        ResetGotoState("Target reached.");
        path_following_active_ = false;
        return;
    }
    pmg::GoalRequest goal;
    goal.target_position = {goto_target_.x, 0.0f, goto_target_.y};
    graph_desired_node_ = runtime_.Snapshot().current_node;
    // The core steering now drives every node axis (turn_rate + travel_speed);
    // keep the whole vector so multidimensional goto is not flattened back to
    // axis 0. graph_desired_parameter_ mirrors axis 0 for the 1-D slider UI.
    goto_desired_parameter_ =
        steering_->RequestForPose(pose, goal).desired_parameter;
    graph_desired_parameter_ = goto_desired_parameter_.front();
}

void PmgViewerWorkspace::SetDirectSteeringInput(float turn, float speed) {
    direct_steering_turn_input_ = std::clamp(turn, -1.0f, 1.0f);
    direct_steering_speed_input_ = std::clamp(speed, -1.0f, 1.0f);
}

void PmgViewerWorkspace::UpdateDirectSteering(const pmg::Pose& pose) {
    if (!steering_.has_value() ||
        (direct_steering_turn_input_ == 0.0f &&
         direct_steering_speed_input_ == 0.0f)) {
        return;
    }
    constexpr float kMaximumHeadingOffsetRadians = pmg::kPi * 0.5f;
    const float desired_heading = pmg::PoseFacingYaw(pose) +
        direct_steering_turn_input_ * kMaximumHeadingOffsetRadians;
    const pmg::SteeringCalibration& calibration = steering_->Calibration();
    std::optional<float> desired_speed;
    for (const pmg::SteeringAxis& axis : calibration.axes) {
        if (axis.metric == pmg::ParameterMetric::kTravelSpeed) {
            const float speed_fraction =
                0.5f * (direct_steering_speed_input_ + 1.0f);
            desired_speed = axis.lowest + speed_fraction * (axis.highest - axis.lowest);
            break;
        }
    }
    goto_desired_parameter_ = steering_->RequestForDirectionSpeed(
        pose, desired_heading, desired_speed).desired_parameter;
    graph_desired_parameter_ = goto_desired_parameter_.front();
}

void PmgViewerWorkspace::UpdateRootMotionDiagnostics(
    const pmg::Pose& pose, float delta_seconds) {
    if (pose.local_rotations.empty()) {
        root_heading_initialized_ = false;
        root_heading_radians_ = 0.0f;
        actual_turn_rate_radians_per_second_ = 0.0f;
        return;
    }

    const pmg::Vec3 forward =
        pmg::Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    const float heading = std::atan2(forward.x, forward.z);
    if (root_heading_initialized_ && delta_seconds > kEpsilon) {
        float heading_delta = heading - root_heading_radians_;
        heading_delta -=
            2.0f * pmg::kPi *
            std::round(heading_delta / (2.0f * pmg::kPi));
        actual_turn_rate_radians_per_second_ =
            heading_delta / delta_seconds;
    }
    root_heading_radians_ = heading;
    root_heading_initialized_ = true;
}

bool PmgViewerWorkspace::HandleGroundClick(const glm::vec3& ray_origin,
                                  const glm::vec3& ray_direction) {
    if (!GraphRuntimeActive()) {
        if (runtime_.Ready()) {
            status_message_ = "Switch to Graph runtime mode to place a walk target.";
        }
        return false;
    }
    if (std::abs(ray_direction.y) < 1.0e-6f) {
        return false;
    }
    const float t = -ray_origin.y / ray_direction.y;
    if (t <= 0.0f) {
        return false;
    }
    const glm::vec3 hit = ray_origin + t * ray_direction;
    // The scene is rendered at display scale; steering runs in native units.
    const glm::vec2 target =
        glm::vec2(hit.x, hit.z) / std::max(display_scale_, kEpsilon);
    if (path_following_active_) {
        path_waypoints_.push_back(target);
        if (path_waypoints_.size() == 1) {
            path_waypoint_index_ = 0;
            PlaceGotoTarget(target);
        }
    } else {
        PlaceGotoTarget(target);
    }
    return steering_.has_value();
}

void PmgViewerWorkspace::Update(float delta_seconds) {
    // Graph runtime: translate viewer state to a raw runtime request, then
    // render the module snapshot. Runtime scheduling/status stays in module.
    if (GraphRuntimeActive()) {
        ViewerRuntimeSnapshot runtime = runtime_.Snapshot();
        if (playing_) {
            if (goto_active_ && steering_.has_value()) {
                UpdateGotoSteering(runtime.current_pose.value_or(pmg::Pose{}));
            } else if (direct_steering_active_ && steering_.has_value()) {
                UpdateDirectSteering(runtime.current_pose.value_or(pmg::Pose{}));
            }
            ViewerRuntimeRequest request;
            request.desired_node = graph_desired_node_;
            request.desired_parameter_raw =
                ((goto_active_ || direct_steering_active_) &&
                 !goto_desired_parameter_.empty())
                    ? goto_desired_parameter_
                    : DesiredParameterForNode(graph_desired_node_);
            runtime_.Update(delta_seconds * playback_speed_, request);
            runtime = runtime_.Snapshot();
        }
        const pmg::Pose pose = runtime.current_pose.value_or(pmg::Pose{});
        UpdateRootMotionDiagnostics(
            pose, playing_ ? delta_seconds * playback_speed_ : 0.0f);
        RebuildScene(pose);
        return;
    }

    // Refresh the cached parametric preview FIRST so the duration used to pace
    // the phase clock matches the parameter we are about to render. The phase
    // itself is left untouched by a parameter change -- only its advance rate
    // (1/duration) moves -- so nudging the blend no longer teleports the pose.
    if (ParametricBlendActive() &&
        (pmg_preview_dirty_ || pmg_parameter_ != pmg_preview_parameter_)) {
        RegeneratePreviewClip();
    }
    if (playing_) {
        const float duration = ActiveReferenceDuration();
        if (duration > kEpsilon) {
            current_phase_ = WrapPhase(
                current_phase_ + delta_seconds * playback_speed_ / duration);
        }
    }
    const pmg::Pose pose = CurrentPose();
    UpdateRootMotionDiagnostics(
        pose, playing_ ? delta_seconds * playback_speed_ : 0.0f);
    RebuildScene(DisplayPose());
}

void PmgViewerWorkspace::AddCurrentClipToSpace(const pmg::ParameterVector& parameter) {
    if (clip_.NumFrames() == 0) {
        status_message_ = "Load a clip before adding it to the PMG space.";
        return;
    }
    if (static_cast<int>(parameter.size()) != pmg_dimension_) {
        status_message_ = "Parameter dimension mismatch.";
        return;
    }
    if (!pmg_examples_.empty()) {
        const pmg::SkeletonCompatibilityResult compatibility =
            pmg::CheckSkeletonCompatibility(pmg_skeleton_, skeleton_);
        if (!compatibility.compatible) {
            status_message_ = "Incompatible skeleton: " + compatibility.reason;
            return;
        }
    }
    if (pmg_examples_.empty()) {
        pmg_skeleton_ = skeleton_;
    }

    const std::string label =
        selected_file_index_ >= 0
            ? bvh_files_[selected_file_index_].filename().string()
            : "clip";
    PmgExample example{label, parameter, clip_, {}};
    RefreshExampleContacts(example);
    pmg_examples_.push_back(std::move(example));
    // Suggest the next sample one unit further along the primary axis.
    next_example_parameter_ = parameter;
    if (!next_example_parameter_.empty()) {
        next_example_parameter_[0] += 1.0f;
    }
    RebuildPmgSpace();
    status_message_ = "Added '" + label + "' (" +
                      std::to_string(pmg_dimension_) + "-D parameter).";
}

std::vector<int> PmgViewerWorkspace::ResolveContactJointIndices() const {
    std::vector<int> joint_indices;
    for (int joint_index = 0; joint_index < pmg_skeleton_.NumJoints(); ++joint_index) {
        const std::string& joint_name = pmg_skeleton_.joints[joint_index].name;
        if (!contact_joint_names_.empty()) {
            if (std::find(contact_joint_names_.begin(), contact_joint_names_.end(),
                          joint_name) != contact_joint_names_.end()) {
                joint_indices.push_back(joint_index);
            }
            continue;
        }

        std::string lowercase_name = joint_name;
        std::transform(
            lowercase_name.begin(), lowercase_name.end(), lowercase_name.begin(),
            [](unsigned char character) {
                return static_cast<char>(std::tolower(character));
            });
        if (lowercase_name.find("ankle") != std::string::npos ||
            lowercase_name.find("foot") != std::string::npos) {
            joint_indices.push_back(joint_index);
        }
    }
    return joint_indices;
}

void PmgViewerWorkspace::RefreshExampleContacts(PmgExample& example) {
    example.contact_intervals.clear();
    const std::vector<int> contact_joints = ResolveContactJointIndices();
    if (contact_joints.empty()) {
        return;
    }
    try {
        pmg::ContactDetectionSettings settings =
            pmg::EstimateContactSettings(
                pmg_skeleton_, example.clip, contact_joints);
        settings.min_contact_frames = 2;
        example.contact_intervals =
            pmg::DetectContacts(
                pmg_skeleton_, example.clip, contact_joints, settings);
    } catch (const std::exception&) {
        example.contact_intervals.clear();
    }
}

std::string PmgViewerWorkspace::CheckBlendableMotionFamily() const {
    // Gather the dim-matching members (the ones RebuildPmgSpace actually adds)
    // and delegate the decision to the pure, unit-tested
    // pmgviewer::FirstNonBlendableExample (see GraphAuthoringModel.h for the
    // single-family rationale).
    const std::size_t dim = static_cast<std::size_t>(std::max(1, pmg_dimension_));
    std::vector<const PmgExample*> members;
    std::vector<int> contact_counts;
    for (const PmgExample& example : pmg_examples_) {
        if (example.parameter.size() == dim) {
            members.push_back(&example);
            contact_counts.push_back(
                static_cast<int>(example.contact_intervals.size()));
        }
    }
    const bool have_foot_joints = !ResolveContactJointIndices().empty();
    const int offender =
        pmgviewer::FirstNonBlendableExample(contact_counts, have_foot_joints);
    if (offender < 0) {
        return {};
    }
    const PmgExample& example = *members[static_cast<std::size_t>(offender)];
    return "Example '" + example.label + "' is not a cyclic locomotion clip (" +
           std::to_string(example.contact_intervals.size()) +
           " foot contact). A parametric blend space holds one motion family; "
           "mixing an acyclic clip jolts the blend. Remove it, or give it its "
           "own graph node and a transition edge.";
}

void PmgViewerWorkspace::ResizeParameterVectors() {
    const std::size_t dim = static_cast<std::size_t>(std::max(1, pmg_dimension_));
    pmg_parameter_.resize(dim, 0.0f);
    next_example_parameter_.resize(dim, 0.0f);
    pmg_view_axis_ = std::clamp(pmg_view_axis_, 0, std::max(0, pmg_dimension_ - 1));
}

void PmgViewerWorkspace::RebuildPmgSpace() {
    pmg_preview_dirty_ = true;
    ResizeParameterVectors();
    if (pmg_examples_.empty()) {
        pmg_space_ready_ = false;
        pmg_space_blend_error_.clear();
        pmg_preview_clip_ = pmg::MotionClip{};
        steering_turn_rate_curve_.clear();
        steering_travel_speed_curve_.clear();
        mode_ = ViewerPlaybackMode::ClipPlayback;
        return;
    }

    const std::size_t dim = static_cast<std::size_t>(std::max(1, pmg_dimension_));
    pmg_space_ = pmg::ParametricMotionSpace("viewer_space", pmg_dimension_);
    pmg_parameter_min_.assign(dim, std::numeric_limits<float>::infinity());
    pmg_parameter_max_.assign(dim, -std::numeric_limits<float>::infinity());
    float min_y = std::numeric_limits<float>::infinity();
    for (const PmgExample& example : pmg_examples_) {
        if (example.parameter.size() != dim) {
            continue;  // skip samples authored at a different dimension
        }
        pmg_space_.AddExample(example.parameter, example.clip);
        for (std::size_t axis = 0; axis < dim; ++axis) {
            pmg_parameter_min_[axis] =
                std::min(pmg_parameter_min_[axis], example.parameter[axis]);
            pmg_parameter_max_[axis] =
                std::max(pmg_parameter_max_[axis], example.parameter[axis]);
        }
        // Ground offset uses the global lowest point across all examples.
        const float offset = ComputeGroundOffset(pmg_skeleton_, example.clip);
        min_y = std::min(min_y, -offset);
    }
    pmg_ground_offset_ = std::isfinite(min_y) ? -min_y : 0.0f;

    for (std::size_t axis = 0; axis < dim; ++axis) {
        if (!std::isfinite(pmg_parameter_min_[axis]) ||
            !std::isfinite(pmg_parameter_max_[axis])) {
            pmg_parameter_min_[axis] = 0.0f;
            pmg_parameter_max_[axis] = 1.0f;
        }
        if (pmg_parameter_max_[axis] <= pmg_parameter_min_[axis]) {
            pmg_parameter_max_[axis] = pmg_parameter_min_[axis] + 1.0f;
        }
        pmg_parameter_[axis] = std::clamp(
            pmg_parameter_[axis], pmg_parameter_min_[axis], pmg_parameter_max_[axis]);
    }
    // Reject a cross-family blend at the authoring boundary instead of silently
    // generating a jolting preview. Keep the space un-ready so Parametric blend
    // stays gated; the reason surfaces in the Motion-space section.
    pmg_space_blend_error_ = CheckBlendableMotionFamily();
    if (!pmg_space_blend_error_.empty()) {
        pmg_space_ready_ = false;
        status_message_ = pmg_space_blend_error_;
        mode_ = ViewerPlaybackMode::ClipPlayback;
        return;
    }
    pmg_space_ready_ = true;
    RecomputeSteeringCurve();
    RecomputeParamSweepPaths();
}

void PmgViewerWorkspace::RecomputeSteeringCurve() {
    steering_turn_rate_curve_.clear();
    steering_travel_speed_curve_.clear();
    if (!pmg_space_ready_ || pmg_space_.NumExamples() == 0) {
        return;
    }
    const int axis = std::clamp(pmg_view_axis_, 0, pmg_dimension_ - 1);
    if (axis >= static_cast<int>(pmg_parameter_min_.size()) ||
        pmg_parameter_.size() != pmg_parameter_min_.size()) {
        return;
    }
    const float fps = pmg_examples_.empty()
                          ? 30.0f
                          : std::max(pmg_examples_.front().clip.frames_per_second, 1.0f);
    // Sweep the view axis across its range, holding the other axes at the
    // current blend; this generalizes the 1-D curve to one axis of an N-D space.
    const float span =
        std::max(pmg_parameter_max_[axis] - pmg_parameter_min_[axis], kEpsilon);
    pmg::ParameterVector query = pmg_parameter_;
    steering_turn_rate_curve_.reserve(kSteeringCurveSamples);
    steering_travel_speed_curve_.reserve(kSteeringCurveSamples);
    for (int i = 0; i < kSteeringCurveSamples; ++i) {
        const float alpha =
            static_cast<float>(i) / static_cast<float>(kSteeringCurveSamples - 1);
        query[axis] = pmg_parameter_min_[axis] + alpha * span;
        float turn_rate = 0.0f;
        float travel_speed = 0.0f;
        try {
            const pmg::MotionClip sample = pmg_space_.GenerateClip(query, fps);
            turn_rate = pmg::MeasureParameterMetric(
                pmg::ParameterMetric::kTurnRate, sample);
            travel_speed = pmg::MeasureParameterMetric(
                pmg::ParameterMetric::kTravelSpeed, sample);
        } catch (const std::exception&) {
            turn_rate = 0.0f;
            travel_speed = 0.0f;
        }
        steering_turn_rate_curve_.push_back(turn_rate);
        steering_travel_speed_curve_.push_back(travel_speed);
    }
}

void PmgViewerWorkspace::RecomputeParamSweepPaths() {
    param_sweep_paths_.clear();
    if (!pmg_space_ready_ || pmg_space_.NumExamples() == 0) {
        return;
    }
    const int axis = std::clamp(pmg_view_axis_, 0, pmg_dimension_ - 1);
    if (axis >= static_cast<int>(pmg_parameter_min_.size()) ||
        pmg_parameter_.size() != pmg_parameter_min_.size()) {
        return;
    }
    const float fps = pmg_examples_.empty()
                          ? 30.0f
                          : std::max(pmg_examples_.front().clip.frames_per_second, 1.0f);
    const float span =
        std::max(pmg_parameter_max_[axis] - pmg_parameter_min_[axis], kEpsilon);
    // ponytail: holds the other axes at the current blend, so for an N-D space the
    // overlay only refreshes on rebuild, not when a sibling axis is dragged. Fine
    // for the 1-D spaces this targets; revisit if N-D cross-axis preview is wanted.
    pmg::ParameterVector query = pmg_parameter_;
    param_sweep_paths_.reserve(kParamSweepPathCount);
    for (int i = 0; i < kParamSweepPathCount; ++i) {
        const float alpha =
            static_cast<float>(i) / static_cast<float>(kParamSweepPathCount - 1);
        query[axis] = pmg_parameter_min_[axis] + alpha * span;
        try {
            const pmg::MotionClip clip = pmg_space_.GenerateClip(query, fps);
            std::vector<glm::vec3> path;
            path.reserve(clip.frames.size());
            for (const pmg::Pose& frame : clip.frames) {
                path.emplace_back(frame.root_position.x, 0.0f,
                                  frame.root_position.z);
            }
            // Blue (slow end of the axis) -> red (fast end), one hue per value so
            // the paths stay distinguishable where they bunch together.
            const glm::vec3 color = glm::mix(glm::vec3(0.25f, 0.55f, 0.95f),
                                             glm::vec3(0.95f, 0.45f, 0.25f), alpha);
            param_sweep_paths_.emplace_back(color, std::move(path));
        } catch (const std::exception&) {
            // Skip an un-generatable sample; the rest still draw.
        }
    }
}

void PmgViewerWorkspace::RegeneratePreviewClip() {
    pmg_preview_dirty_ = false;
    pmg_preview_parameter_ = pmg_parameter_;
    if (!pmg_space_ready_ || pmg_space_.NumExamples() == 0) {
        pmg_preview_clip_ = pmg::MotionClip{};
        return;
    }
    const float fps = pmg_examples_.empty()
                          ? 30.0f
                          : std::max(pmg_examples_.front().clip.frames_per_second, 1.0f);
    pmg_preview_clip_ = pmg_space_.GenerateClip(pmg_parameter_, fps);

    // IK foot-lock post-process: parametric blends leave the stance foot sliding
    // (foot-skate). Pin it on the generated output (Kovar cleanup). This needs no
    // contact Registration -- it detects contacts on the blended clip directly --
    // so it works even when the node declares no contact joints. Ankles resolved
    // from the skeleton by name; if absent, the raw blend is shown unchanged.
    if (pmg_foot_lock_enabled_ && pmg_preview_clip_.NumFrames() >= 2) {
        // Reuse the viewer's contact-joint resolver: it honors the
        // --contact-joints override and otherwise substring-matches
        // "ankle"/"foot" case-insensitively, so CMU skeletons (LeftFoot/
        // RightFoot) lock too. The old exact "LeftAnkle"/"RightAnkle" match
        // silently no-op'd on CMU.
        const std::vector<int> foot_joints = ResolveContactJointIndices();
        if (!foot_joints.empty()) {
            pmg::FootLockSettings lock_settings;
            lock_settings.contacts = pmg::EstimateContactSettings(
                pmg_skeleton_, pmg_preview_clip_, foot_joints);
            pmg::LockFootContacts(
                pmg_skeleton_, pmg_preview_clip_, foot_joints, lock_settings);
        }
    }
}

// --- ImGui UI ---------------------------------------------------------------
//
// Everything lives in one window: Workflow + Transport stay visible at the
// top (they are needed in every mode), the rest is tabbed. Seven floating
// windows buried the 3D view and each other.

void PmgViewerWorkspace::BuildUi() {
    HandleShortcuts();
    const ViewerUiState state = MakeUiState();
    const std::vector<ViewerUiCommand> commands = imgui_ui_.Build(
        state, [this] {
            if (ImGui::BeginTabItem("Transition Grid")) {
                BuildDistanceGridSection();
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Graph")) {
                BuildGraphSection();
                ImGui::EndTabItem();
            }
        });
    ApplyUiCommands(commands);
}

ViewerUiState PmgViewerWorkspace::MakeUiState() const {
    ViewerUiState state;
    state.playback_mode = mode_;
    state.playing = playing_;
    state.motion_space_ready = pmg_space_ready_;
    state.graph_runtime_ready = runtime_.Ready();
    state.graph_runtime_active = GraphRuntimeActive();
    state.path_preview_enabled = path_preview_enabled_;
    state.path_preview_count = path_preview_count_;
    state.param_sweep_paths_enabled = show_param_sweep_paths_;
    state.playback_speed = playback_speed_;
    state.phase = state.graph_runtime_active ? runtime_.Snapshot().current_phase
                                             : current_phase_;
    state.skeleton_scale = skeleton_scale_;
    state.display_scale = display_scale_;
    state.selected_clip_index = selected_file_index_;
    state.selected_spec_index = selected_spec_index_;
    state.status_message = status_message_;
    for (const std::filesystem::path& path : bvh_files_)
        state.clip_names.push_back(path.filename().string());
    state.loaded_frame_count = clip_.NumFrames();
    state.loaded_frames_per_second = clip_.frames_per_second;
    for (const pmg::Joint& joint : skeleton_.joints) {
        state.loaded_joints.push_back(
            {joint.name, joint.parent_index, static_cast<int>(joint.channels.size())});
    }
    state.artifact_units = artifact_units_;
    state.motion_space_dimension = pmg_dimension_;
    state.has_motion_samples = !pmg_examples_.empty();
    state.next_sample_parameter = next_example_parameter_;
    state.motion_space_name = pmg_space_ready_ ? pmg_space_.Name() : "";
    state.motion_space_blend_error = pmg_space_blend_error_;
    state.motion_space_parameter = pmg_parameter_;
    state.motion_space_parameter_min = pmg_parameter_min_;
    state.motion_space_parameter_max = pmg_parameter_max_;
    state.motion_space_view_axis = pmg_view_axis_;
    state.motion_space_preview_in_place = pmg_preview_in_place_;
    state.motion_space_foot_lock = pmg_foot_lock_enabled_;
    state.motion_space_turn_rate_curve = steering_turn_rate_curve_;
    state.motion_space_travel_speed_curve = steering_travel_speed_curve_;
    if (pmg_preview_clip_.NumFrames() >= 2) {
        try {
            state.motion_space_current_blend_turn_rate = pmg::MeasureParameterMetric(
                pmg::ParameterMetric::kTurnRate, pmg_preview_clip_);
            state.motion_space_current_blend_travel_speed = pmg::MeasureParameterMetric(
                pmg::ParameterMetric::kTravelSpeed, pmg_preview_clip_);
            state.motion_space_current_blend_metrics_valid = true;
        } catch (const std::exception&) {
            state.motion_space_current_blend_metrics_valid = false;
        }
    }
    if (pmg_space_ready_ && !pmg_examples_.empty()) {
        try {
            state.motion_space_local_blend_weights =
                pmg_space_.ComputeLocalBlendWeights(pmg_parameter_);
        } catch (const std::exception& error) {
            state.motion_space_weight_error = error.what();
        }
        if (pmg_space_.HasParameterCalibration()) {
            const pmg::ParameterCalibration& calibration =
                pmg_space_.ParameterCalibrationData();
            state.motion_space_has_parameter_calibration = true;
            state.motion_space_calibration_metric_count =
                static_cast<int>(calibration.metrics.size());
            state.motion_space_calibration_sample_count =
                static_cast<int>(calibration.samples.size());
            state.motion_space_calibration_samples_per_axis =
                calibration.samples_per_axis;
        }
    }
    const bool has_warps = pmg_space_ready_ && pmg_space_.HasExampleTimeWarps();
    const std::vector<pmg::TimeWarp>* warps =
        has_warps ? &pmg_space_.ExampleTimeWarps() : nullptr;
    for (std::size_t index = 0; index < pmg_examples_.size(); ++index) {
        const PmgExample& example = pmg_examples_[index];
        ViewerMotionSampleUiState sample_state;
        sample_state.label = example.label;
        sample_state.parameter = example.parameter;
        const bool sample_has_warp =
            has_warps && index < warps->size();
        sample_state.registered_phase =
            sample_has_warp ? (*warps)[index].Evaluate(state.phase) : state.phase;
        if (sample_has_warp) {
            sample_state.registration_anchor_phases =
                (*warps)[index].InteriorToPhases();
        }
        sample_state.contacts.reserve(example.contact_intervals.size());
        for (const pmg::ContactInterval& contact : example.contact_intervals) {
            sample_state.contacts.push_back(
                {contact.StrikePhase(example.clip.NumFrames()),
                 contact.LiftPhase(example.clip.NumFrames())});
        }
        state.motion_samples.push_back(std::move(sample_state));
    }
    state.show_joint_names = show_joint_names_;
    state.show_end_sites_separately = show_end_sites_separately_;
    state.hide_end_sites = hide_end_sites_;
    state.draw_local_joint_axes = draw_local_joint_axes_;
    state.mark_likely_foot_joints = mark_likely_foot_joints_;
    state.skeleton_pose_mode = skeleton_pose_mode_;
    return state;
}

void PmgViewerWorkspace::ApplyUiCommands(
    const std::vector<ViewerUiCommand>& commands) {
    for (const ViewerUiCommand& command : commands) ApplyUiCommand(command);
}

void PmgViewerWorkspace::ApplyUiCommand(const ViewerUiCommand& command) {
    switch (command.type) {
    case ViewerUiCommandType::LoadClip: LoadClip(command.index); break;
    case ViewerUiCommandType::SetPlaybackMode:
        if (command.index < 0 || command.index > 2)
            throw std::invalid_argument("playback mode index must be in [0, 2]");
        mode_ = static_cast<ViewerPlaybackMode>(command.index); break;
    case ViewerUiCommandType::TogglePlayback: playing_ = !playing_; break;
    case ViewerUiCommandType::ResetPlayback: ResetPlayback(); break;
    case ViewerUiCommandType::StepFrame: StepFrame(command.index); break;
    case ViewerUiCommandType::SetPlaybackSpeed:
        playback_speed_ = std::clamp(command.x, 0.1f, 3.0f); break;
    case ViewerUiCommandType::SetPhase:
        current_phase_ = std::clamp(command.x, 0.0f, 1.0f); playing_ = false; break;
    case ViewerUiCommandType::SetPathPreviewEnabled:
        path_preview_enabled_ = command.index != 0; break;
    case ViewerUiCommandType::SetPathPreviewCount:
        path_preview_count_ = std::clamp(command.index, 2, 12); break;
    case ViewerUiCommandType::SetParamSweepPaths:
        show_param_sweep_paths_ = command.index != 0; break;
    case ViewerUiCommandType::SetSkeletonScale:
        skeleton_scale_ = std::clamp(command.x, 0.1f, 5.0f); break;
    case ViewerUiCommandType::SetDisplayScale:
        display_scale_ = std::clamp(command.x, 1.0f, 40.0f); break;
    case ViewerUiCommandType::SetScalarParameter:
        if (command.index < 0 || command.index >= static_cast<int>(pmg_parameter_.size()))
            throw std::out_of_range("scalar parameter index is outside parameter vector");
        pmg_parameter_[command.index] = command.x; break;
    case ViewerUiCommandType::SetVectorParameter:
        pmg_parameter_ = command.values; pmg_preview_dirty_ = true; break;
    case ViewerUiCommandType::RebuildMotionSpace: RebuildPmgSpace(); break;
    case ViewerUiCommandType::RecomputeHeatmap: RecomputeHeatmap(); break;
    case ViewerUiCommandType::SaveHeatmapCsv: SaveHeatmapCsv(); break;
    case ViewerUiCommandType::BuildGraphRuntime: BuildGraphRuntime(); break;
    case ViewerUiCommandType::ResetGraphRuntime: ResetGraphRuntimeSession(false); break;
    case ViewerUiCommandType::SetDesiredRuntimeNode: SetDesiredRuntimeNode(command.index); break;
    case ViewerUiCommandType::SetDirectSteeringMode:
        direct_steering_active_ = command.index != 0; break;
    case ViewerUiCommandType::SetGotoMode:
        goto_active_ = command.index != 0; break;
    case ViewerUiCommandType::SaveArtifact: SaveArtifact(command.text); break;
    case ViewerUiCommandType::LoadGraphArtifact: LoadGraphArtifact(command.text); break;
    case ViewerUiCommandType::BuildArtifactFromSpec: BuildArtifactFromSpec(command.text); break;
    case ViewerUiCommandType::SetMotionSpaceDimension:
        if (!pmg_examples_.empty()) break;
        pmg_dimension_ = std::clamp(command.index, 1, 4);
        ResizeParameterVectors();
        break;
    case ViewerUiCommandType::SetNextSampleParameter:
        next_example_parameter_ = command.values;
        ResizeParameterVectors();
        break;
    case ViewerUiCommandType::AddMotionSample:
        AddCurrentClipToSpace(next_example_parameter_);
        break;
    case ViewerUiCommandType::SetMotionSpaceViewAxis:
        pmg_view_axis_ = std::clamp(command.index, 0, std::max(0, pmg_dimension_ - 1));
        RecomputeSteeringCurve();
        break;
    case ViewerUiCommandType::SetMotionSpacePreviewInPlace:
        pmg_preview_in_place_ = command.index != 0;
        break;
    case ViewerUiCommandType::SetMotionSpaceFootLock:
        // Foot-lock rewrites the preview clip's stance legs, so the cached clip
        // must be regenerated when the toggle changes.
        pmg_foot_lock_enabled_ = command.index != 0;
        pmg_preview_dirty_ = true;
        break;
    case ViewerUiCommandType::SetShowJointNames:
        show_joint_names_ = command.index != 0;
        break;
    case ViewerUiCommandType::SetShowEndSitesSeparately:
        show_end_sites_separately_ = command.index != 0;
        if (show_end_sites_separately_) {
            hide_end_sites_ = false;
        }
        break;
    case ViewerUiCommandType::SetHideEndSites:
        hide_end_sites_ = command.index != 0;
        if (hide_end_sites_) {
            show_end_sites_separately_ = false;
        }
        break;
    case ViewerUiCommandType::SetDrawLocalJointAxes:
        draw_local_joint_axes_ = command.index != 0;
        break;
    case ViewerUiCommandType::SetMarkLikelyFootJoints:
        mark_likely_foot_joints_ = command.index != 0;
        break;
    case ViewerUiCommandType::SetSkeletonPoseMode:
        if (command.index < 0 || command.index > 2) {
            throw std::invalid_argument("skeleton pose mode index must be in [0, 2]");
        }
        skeleton_pose_mode_ = static_cast<ViewerSkeletonPoseMode>(command.index);
        break;
    case ViewerUiCommandType::SetMotionSampleParameter:
        if (command.index < 0 ||
            command.index >= static_cast<int>(pmg_examples_.size()))
            throw std::out_of_range("motion sample index is outside sample list");
        if (static_cast<int>(command.values.size()) != pmg_dimension_)
            throw std::invalid_argument("motion sample parameter dimension mismatch");
        pmg_examples_[command.index].parameter = command.values;
        RebuildPmgSpace();
        break;
    case ViewerUiCommandType::RemoveMotionSample:
        if (command.index < 0 ||
            command.index >= static_cast<int>(pmg_examples_.size()))
            throw std::out_of_range("motion sample index is outside sample list");
        pmg_examples_.erase(pmg_examples_.begin() + command.index);
        RebuildPmgSpace();
        status_message_ = "Removed motion sample.";
        break;
    case ViewerUiCommandType::ClearMotionSpace:
        pmg_examples_.clear();
        RebuildPmgSpace();
        status_message_ = "Cleared parametric space.";
        break;
    }
}

void PmgViewerWorkspace::HandleShortcuts() {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput) {
        return;  // don't steal keys while the user types into an input
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Space, /*repeat=*/false)) {
        playing_ = !playing_;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_R, /*repeat=*/false)) {
        ResetPlayback();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow, /*repeat=*/true)) {
        StepFrame(-1);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, /*repeat=*/true)) {
        StepFrame(1);
    }
}

void PmgViewerWorkspace::StepFrame(int direction) {
    if (GraphRuntimeActive()) {
        return;  // graph runtime is controller-driven; stepping is for clip/blend
    }
    // Step by one frame of whatever clip is actually rendered: the blended
    // preview in blend mode, the loaded clip otherwise. Phase is the clock, so
    // one frame is 1/(frame_count - 1) of the cycle.
    const int frame_count =
        (ParametricBlendActive() && pmg_preview_clip_.NumFrames() > 1)
            ? pmg_preview_clip_.NumFrames()
            : clip_.NumFrames();
    if (frame_count <= 1) {
        return;
    }
    playing_ = false;
    const float frame_phase = 1.0f / static_cast<float>(frame_count - 1);
    current_phase_ =
        WrapPhase(current_phase_ + static_cast<float>(direction) * frame_phase);
}

// --- Distance Grid heatmap (transition visualization) ----------------------

void PmgViewerWorkspace::RecomputeHeatmap() {
    heatmap_ready_ = false;
    heatmap_min_source_index_ = -1;
    heatmap_min_target_index_ = -1;
    heatmap_selected_source_index_ = -1;
    heatmap_selected_target_index_ = -1;

    if (clip_.NumFrames() == 0 || skeleton_.NumJoints() == 0) {
        heatmap_status_ = "Load a source clip first.";
        return;
    }
    if (heatmap_target_index_ < 0 ||
        heatmap_target_index_ >= static_cast<int>(bvh_files_.size())) {
        heatmap_status_ = "Pick a target clip.";
        return;
    }

    // Keep phase windows ordered and inside [0,1] (BuildDistanceGrid throws otherwise).
    heatmap_config_.source_phase_start = std::clamp(heatmap_config_.source_phase_start, 0.0f, 1.0f);
    heatmap_config_.source_phase_end =
        std::clamp(heatmap_config_.source_phase_end, heatmap_config_.source_phase_start, 1.0f);
    heatmap_config_.target_phase_start = std::clamp(heatmap_config_.target_phase_start, 0.0f, 1.0f);
    heatmap_config_.target_phase_end =
        std::clamp(heatmap_config_.target_phase_end, heatmap_config_.target_phase_start, 1.0f);

    try {
        const pmg::BvhData target =
            pmg::BvhLoader::Load(bvh_files_[heatmap_target_index_].string());
        const pmg::SkeletonCompatibilityResult compatibility =
            pmg::CheckSkeletonCompatibility(skeleton_, target.skeleton);
        if (!compatibility.compatible) {
            heatmap_status_ = "Target skeleton differs from source: " + compatibility.reason;
            return;
        }
        heatmap_target_clip_ = target.clip;
        heatmap_grid_ = pmg::MotionDistance::BuildDistanceGrid(
            skeleton_, clip_, heatmap_target_clip_, heatmap_config_);
    } catch (const std::exception& error) {
        heatmap_status_ = std::string("Heatmap build failed: ") + error.what();
        return;
    }

    if (heatmap_grid_.SourceCount() == 0 || heatmap_grid_.TargetCount() == 0) {
        heatmap_status_ = "Empty grid.";
        return;
    }

    heatmap_min_distance_ = std::numeric_limits<float>::infinity();
    heatmap_max_distance_ = -std::numeric_limits<float>::infinity();
    float best = std::numeric_limits<float>::infinity();
    for (int source_index = 0; source_index < heatmap_grid_.SourceCount(); ++source_index) {
        for (int target_index = 0; target_index < heatmap_grid_.TargetCount(); ++target_index) {
            const float distance = heatmap_grid_.At(source_index, target_index);
            heatmap_min_distance_ = std::min(heatmap_min_distance_, distance);
            heatmap_max_distance_ = std::max(heatmap_max_distance_, distance);
            if (distance < best) {
                best = distance;
                heatmap_min_source_index_ = source_index;
                heatmap_min_target_index_ = target_index;
            }
        }
    }
    heatmap_selected_source_index_ = heatmap_min_source_index_;
    heatmap_selected_target_index_ = heatmap_min_target_index_;
    heatmap_ready_ = true;
    heatmap_status_ = "Grid " + std::to_string(heatmap_grid_.SourceCount()) + " x " +
                      std::to_string(heatmap_grid_.TargetCount()) + " built.";
}

void PmgViewerWorkspace::SaveHeatmapCsv() {
    if (!heatmap_ready_) {
        heatmap_status_ = "Nothing to save; Recompute first.";
        return;
    }
    std::ofstream out("distance_grid.csv");
    if (!out) {
        heatmap_status_ = "Cannot open distance_grid.csv for writing.";
        return;
    }
    out << "source_frame,target_frame,distance\n";
    for (int source_index = 0; source_index < heatmap_grid_.SourceCount(); ++source_index) {
        for (int target_index = 0; target_index < heatmap_grid_.TargetCount(); ++target_index) {
            out << heatmap_grid_.source_frames[source_index] << ","
                << heatmap_grid_.target_frames[target_index] << ","
                << heatmap_grid_.At(source_index, target_index) << "\n";
        }
    }
    heatmap_status_ = "Saved distance_grid.csv";
}

void PmgViewerWorkspace::BuildDistanceGridSection() {
    if (selected_file_index_ >= 0 &&
        selected_file_index_ < static_cast<int>(bvh_files_.size())) {
        ImGui::Text("Source: %s",
                    bvh_files_[selected_file_index_].filename().string().c_str());
    } else {
        ImGui::TextDisabled("Source: none (load a clip)");
    }

    const std::string target_name =
        (heatmap_target_index_ >= 0 &&
         heatmap_target_index_ < static_cast<int>(bvh_files_.size()))
            ? bvh_files_[heatmap_target_index_].filename().string()
            : std::string("(pick target)");
    if (ImGui::BeginCombo("Target", target_name.c_str())) {
        for (int i = 0; i < static_cast<int>(bvh_files_.size()); ++i) {
            const bool selected = (i == heatmap_target_index_);
            if (ImGui::Selectable(bvh_files_[i].filename().string().c_str(), selected)) {
                heatmap_target_index_ = i;
            }
        }
        ImGui::EndCombo();
    }

    // Shared GOOD/BAD thresholds for the heatmap and sandbox graph builds.
    // .pmg_spec builds keep their per-edge edge_config/edge_phase_range values.
    ImGui::InputFloat("TGOOD", &tgood_, 1.0f, 10.0f, "%.3f");
    ImGui::InputFloat("TBAD", &tbad_, 1.0f, 10.0f, "%.3f");
    tgood_ = std::max(0.0f, tgood_);
    tbad_ = std::max(tgood_, tbad_);

    if (ImGui::Button("Recompute")) {
        RecomputeHeatmap();
    }
    ImGui::SameLine();
    if (ImGui::Button("Save CSV")) {
        SaveHeatmapCsv();
    }

    // Grid-sampling knobs are advanced; collapse them so the panel stays
    // approachable. The preset restores the paper's transition-region window.
    if (ImGui::CollapsingHeader("Advanced grid settings")) {
        if (ImGui::Button("Transition-region preset")) {
            heatmap_config_.window_size = 5;
            heatmap_config_.source_frame_stride = 2;
            heatmap_config_.target_frame_stride = 2;
            heatmap_config_.source_phase_start = 0.70f;
            heatmap_config_.source_phase_end = 0.95f;
            heatmap_config_.target_phase_start = 0.05f;
            heatmap_config_.target_phase_end = 0.30f;
        }
        ImGui::SliderInt("Window", &heatmap_config_.window_size, 1, 15);
        ImGui::SliderInt("Source stride", &heatmap_config_.source_frame_stride, 1, 5);
        ImGui::SliderInt("Target stride", &heatmap_config_.target_frame_stride, 1, 5);
        ImGui::SliderFloat("Source phase start", &heatmap_config_.source_phase_start, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Source phase end", &heatmap_config_.source_phase_end, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Target phase start", &heatmap_config_.target_phase_start, 0.0f, 1.0f, "%.2f");
        ImGui::SliderFloat("Target phase end", &heatmap_config_.target_phase_end, 0.0f, 1.0f, "%.2f");
    }

    ImGui::TextWrapped("%s", heatmap_status_.c_str());
    ImGui::Separator();

    if (!heatmap_ready_) {
        ImGui::TextDisabled("No grid yet. Pick a target and Recompute.");
        return;
    }

    const int source_count = heatmap_grid_.SourceCount();
    const int target_count = heatmap_grid_.TargetCount();
    const float canvas_size = 300.0f;
    const float cell_w = canvas_size / static_cast<float>(target_count);
    const float cell_h = canvas_size / static_cast<float>(source_count);
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    const float range = std::max(heatmap_max_distance_ - heatmap_min_distance_, kEpsilon);
    for (int source_index = 0; source_index < source_count; ++source_index) {
        for (int target_index = 0; target_index < target_count; ++target_index) {
            const float distance = heatmap_grid_.At(source_index, target_index);
            const float norm = (distance - heatmap_min_distance_) / range;
            const ImVec2 p0(origin.x + target_index * cell_w, origin.y + source_index * cell_h);
            const ImVec2 p1(p0.x + cell_w + 1.0f, p0.y + cell_h + 1.0f);
            draw_list->AddRectFilled(p0, p1, HeatColor(norm));
        }
    }

    // White outline: global minimum cell (optimal transition).
    if (heatmap_min_source_index_ >= 0) {
        const ImVec2 p0(origin.x + heatmap_min_target_index_ * cell_w,
                        origin.y + heatmap_min_source_index_ * cell_h);
        const ImVec2 p1(p0.x + cell_w, p0.y + cell_h);
        draw_list->AddRect(p0, p1, IM_COL32(255, 255, 255, 255), 0.0f, 0, 2.0f);
    }
    // Yellow outline: clicked / selected cell.
    if (heatmap_selected_source_index_ >= 0) {
        const ImVec2 p0(origin.x + heatmap_selected_target_index_ * cell_w,
                        origin.y + heatmap_selected_source_index_ * cell_h);
        const ImVec2 p1(p0.x + cell_w, p0.y + cell_h);
        draw_list->AddRect(p0, p1, IM_COL32(255, 255, 0, 255), 0.0f, 0, 1.5f);
    }

    ImGui::InvisibleButton("##heatmap_canvas", ImVec2(canvas_size, canvas_size));
    if (ImGui::IsItemHovered()) {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const int target_index =
            std::clamp(static_cast<int>((mouse.x - origin.x) / cell_w), 0, target_count - 1);
        const int source_index =
            std::clamp(static_cast<int>((mouse.y - origin.y) / cell_h), 0, source_count - 1);
        const float distance = heatmap_grid_.At(source_index, target_index);
        ImGui::BeginTooltip();
        ImGui::Text("src frame %d (phase %.2f)", heatmap_grid_.source_frames[source_index],
                    FramePhaseLocal(heatmap_grid_.source_frames[source_index], clip_.NumFrames()));
        ImGui::Text("tgt frame %d (phase %.2f)", heatmap_grid_.target_frames[target_index],
                    FramePhaseLocal(heatmap_grid_.target_frames[target_index],
                                    heatmap_target_clip_.NumFrames()));
        ImGui::Text("distance %.4f", distance);
        ImGui::EndTooltip();
        if (ImGui::IsItemClicked()) {
            heatmap_selected_source_index_ = source_index;
            heatmap_selected_target_index_ = target_index;
        }
    }

    ImGui::Spacing();
    ImGui::TextDisabled(
        "x target | y source | blue low | red high | white min");
    ImGui::Text("distance range  min %.4f   max %.4f",
                heatmap_min_distance_, heatmap_max_distance_);
    ImGui::Separator();

    auto print_cell = [&](const char* label, int source_index, int target_index) {
        if (source_index < 0 || target_index < 0) {
            return;
        }
        const int source_frame = heatmap_grid_.source_frames[source_index];
        const int target_frame = heatmap_grid_.target_frames[target_index];
        const float distance = heatmap_grid_.At(source_index, target_index);
        const char* classification = distance <= tgood_   ? "GOOD"
                                     : distance >= tbad_   ? "BAD"
                                                           : "NEUTRAL";
        ImGui::Text("%s: src f%d (%.2f) -> tgt f%d (%.2f)  d=%.4f  [%s]",
                    label, source_frame,
                    FramePhaseLocal(source_frame, clip_.NumFrames()),
                    target_frame,
                    FramePhaseLocal(target_frame, heatmap_target_clip_.NumFrames()),
                    distance, classification);
    };
    print_cell("optimal", heatmap_min_source_index_, heatmap_min_target_index_);
    print_cell("selected", heatmap_selected_source_index_, heatmap_selected_target_index_);

    if (heatmap_selected_source_index_ >= 0) {
        const pmg::RigidTransform2D& alignment = heatmap_grid_.alignments[
            static_cast<std::size_t>(heatmap_selected_source_index_) * target_count +
            heatmap_selected_target_index_];
        ImGui::Text("selected alignment: yaw %.3f rad  dx %.2f  dz %.2f",
                    alignment.yaw, alignment.dx, alignment.dz);
    }
}

}  // namespace pmgviewer
