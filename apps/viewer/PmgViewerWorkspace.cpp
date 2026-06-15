#include "PmgViewerWorkspace.h"

#include "PmgViewerWorkspaceFactory.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

#include "pmg/ForwardKinematics.h"
#include "pmg/MathTypes.h"
#include "pmg/SkeletonCompatibility.h"

#ifndef PMG_BVH_DIRECTORY
#define PMG_BVH_DIRECTORY "BVH"
#endif

namespace pmgviewer {

namespace {
constexpr float kEpsilon = 1.0e-6f;
constexpr float kParameterCanvasHeight = 88.0f;
constexpr float kPhaseRowHeight = 26.0f;
constexpr int kSteeringCurveSamples = 24;  // turn-rate plot resolution
constexpr std::size_t kMaxRuntimePathPoints = 600;
constexpr std::size_t kMaxTransitionMarkers = 128;
constexpr float kMinimumTracePointDistance = 0.01f;

glm::vec3 ToGlm(const pmg::Vec3& v) { return glm::vec3(v.x, v.y, v.z); }

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

float ParameterToCanvasX(float parameter, float min_parameter, float max_parameter,
                         float left, float width) {
    const float span = std::max(max_parameter - min_parameter, kEpsilon);
    const float alpha = std::clamp(
        (parameter - min_parameter) / span, 0.0f, 1.0f);
    return left + alpha * width;
}

void DrawPhaseMarker(ImDrawList* draw_list, float phase, float left, float top,
                     float width, float height, ImU32 color) {
    const float x = left + std::clamp(phase, 0.0f, 1.0f) * width;
    draw_list->AddTriangleFilled(
        ImVec2(x, top),
        ImVec2(x - 5.0f, top - 7.0f),
        ImVec2(x + 5.0f, top - 7.0f),
        color);
    draw_list->AddLine(
        ImVec2(x, top), ImVec2(x, top + height), color, 1.5f);
}

}  // namespace

std::unique_ptr<ViewerWorkspace> CreatePmgViewerWorkspace() {
    return std::make_unique<PmgViewerWorkspace>();
}

void PmgViewerWorkspace::Initialize(const std::string& artifact_path) {
    DiscoverBvhFiles();
    DiscoverSpecFiles();
    if (!bvh_files_.empty()) {
        LoadClip(0);
    }
    if (!artifact_path.empty()) {
        LoadGraphArtifact(artifact_path);
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
    const std::filesystem::path directory(PMG_SPEC_DIRECTORY);
    std::error_code error;
    if (!std::filesystem::exists(directory, error)) {
        return;
    }
    for (const auto& entry :
         std::filesystem::directory_iterator(directory, error)) {
        if (entry.is_regular_file() &&
            entry.path().extension() == ".pmg_spec") {
            spec_files_.push_back(entry.path());
        }
    }
    std::sort(spec_files_.begin(), spec_files_.end());
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
    return mode_ == ViewerPlaybackMode::GraphRuntime && graph_ready_ &&
           graph_controller_.has_value();
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

void PmgViewerWorkspace::RebuildScene(const pmg::Pose& pose) {
    const pmg::Skeleton& skeleton = ActiveSkeleton();
    if (pose.NumJoints() != skeleton.NumJoints() || skeleton.NumJoints() == 0) {
        return;
    }

    const float ground_offset =
        (ParametricBlendActive() || GraphRuntimeActive()) ? pmg_ground_offset_ : ground_offset_;
    const std::vector<glm::vec3> world_positions =
        PoseWorldPositions(pose, skeleton, ground_offset);

    glm::vec3 centroid(0.0f);
    scene_.joints.clear();
    scene_.joints.reserve(world_positions.size());
    for (const glm::vec3& world_position : world_positions) {
        centroid += world_position;
        scene_.joints.push_back(world_position);
    }
    if (!world_positions.empty()) {
        centroid /= static_cast<float>(world_positions.size());
    }

    scene_.bones.clear();
    for (int joint_index = 0; joint_index < skeleton.NumJoints(); ++joint_index) {
        const int parent = skeleton.joints[joint_index].parent_index;
        if (parent < 0) {
            continue;
        }
        scene_.bones.push_back(
            {world_positions[static_cast<std::size_t>(parent)],
             world_positions[static_cast<std::size_t>(joint_index)]});
    }

    // Goto target gizmo: ground sphere plus a beacon roughly character-tall
    // (native target back into display space, same transform as the root).
    scene_.marker_points.clear();
    scene_.marker_lines.clear();
    scene_.diagnostic_points.clear();
    scene_.diagnostic_lines.clear();

    if (GraphRuntimeActive() && show_graph_path_trail_ &&
        graph_path_points_.size() >= 2) {
        constexpr glm::vec3 kPathTrailColor(0.10f, 0.70f, 0.95f);
        const float trail_height = 0.05f * display_scale_;
        const float trail_radius = 0.035f * display_scale_;
        for (std::size_t index = 1; index < graph_path_points_.size();
             ++index) {
            const glm::vec2& previous = graph_path_points_[index - 1];
            const glm::vec2& current = graph_path_points_[index];
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
        for (const TransitionTraceMarker& marker :
             graph_transition_markers_) {
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

    if (goto_active_ && GraphRuntimeActive()) {
        const glm::vec3 base(goto_target_.x * display_scale_, 0.0f,
                             goto_target_.y * display_scale_);
        const glm::vec3 top = base + glm::vec3(0.0f, 1.8f * display_scale_, 0.0f);
        scene_.marker_points.push_back(base);
        scene_.marker_lines.push_back({base, top});
    }

    AppendPathPreview();

    scene_.focus_point = centroid;
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
    for (int ghost = 0; ghost < ghost_count; ++ghost) {
        const float phase =
            static_cast<float>(ghost) / static_cast<float>(ghost_count - 1);
        const pmg::Pose ghost_pose = clip->SampleNormalizedPhase(phase);
        if (ghost_pose.NumJoints() != skeleton->NumJoints()) {
            continue;
        }
        // Cool (cyan) at the start fading to warm (amber) at the end, so the
        // motion's direction in time reads at a glance.
        const glm::vec3 ghost_color =
            glm::mix(glm::vec3(0.20f, 0.75f, 0.95f),
                     glm::vec3(0.98f, 0.62f, 0.20f), phase);
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
            });
        }
    }
}

void PmgViewerWorkspace::ResetRuntimeTrace() {
    graph_path_points_.clear();
    graph_transition_markers_.clear();
    if (graph_controller_.has_value()) {
        RecordRuntimeTracePoint(graph_controller_->CurrentPose());
    }
}

void PmgViewerWorkspace::RecordRuntimeTracePoint(const pmg::Pose& pose) {
    if (!graph_ready_) {
        return;
    }
    const glm::vec2 root_position(
        pose.root_position.x, pose.root_position.z);
    if (!graph_path_points_.empty()) {
        const glm::vec2 delta = root_position - graph_path_points_.back();
        if (glm::dot(delta, delta) <
            kMinimumTracePointDistance * kMinimumTracePointDistance) {
            return;
        }
    }
    graph_path_points_.push_back(root_position);
    if (graph_path_points_.size() > kMaxRuntimePathPoints) {
        graph_path_points_.erase(graph_path_points_.begin());
    }
}

void PmgViewerWorkspace::RecordTransitionMarker(
    const pmg::RuntimeTransitionDiagnostics& transition) {
    if (!graph_controller_.has_value()) {
        return;
    }
    const pmg::Pose pose = graph_controller_->CurrentPose();
    graph_transition_markers_.push_back({
        glm::vec2(pose.root_position.x, pose.root_position.z),
        transition.source_node,
        transition.target_node,
    });
    if (graph_transition_markers_.size() > kMaxTransitionMarkers) {
        graph_transition_markers_.erase(
            graph_transition_markers_.begin());
    }
}

// --- Goto steering (viewer port of CLI --goto) ------------------------------

// Achieved world turn rate when streaming the graph at one held parameter.
// Differs from the example clips' own turn rates: each self-transition plays
// only the target-phase -> source-gate slice per cycle, so the net heading
// advance per hop is that slice's heading change (including sway).
void PmgViewerWorkspace::CalibrateSteering() {
    steering_.reset();
    if (!graph_ready_ || pmg_examples_.empty()) {
        goto_status_ = "Build a graph first.";
        return;
    }
    try {
        pmg::GoalDirectedLocomotionConfig steering_config;
        steering_config.runtime = graph_runtime_config_;
        steering_.emplace(
            graph_, pmg_skeleton_, 0, graph_fps_, steering_config);
        const pmg::SteeringCalibration& calibration =
            steering_->Calibration();
        goto_status_ = "Calibrated: achieved turn rates " +
                       std::to_string(calibration.LowestRate()) + " .. " +
                       std::to_string(calibration.HighestRate()) + " rad/s.";
    } catch (const std::exception& error) {
        steering_.reset();
        goto_status_ = std::string("Calibration failed: ") + error.what();
    }
}

void PmgViewerWorkspace::UpdateGotoSteering(const pmg::Pose& pose) {
    const float dx = goto_target_.x - pose.root_position.x;
    const float dz = goto_target_.y - pose.root_position.z;
    const float distance = std::sqrt(dx * dx + dz * dz);
    if (distance <= goto_tolerance_) {
        goto_active_ = false;
        goto_status_ = "Target reached.";
        return;
    }
    pmg::GoalRequest goal;
    goal.target_position = {goto_target_.x, 0.0f, goto_target_.y};
    graph_desired_node_ = graph_controller_->CurrentNode();
    // The core steering now drives every node axis (turn_rate + travel_speed);
    // keep the whole vector so multidimensional goto is not flattened back to
    // axis 0. graph_desired_parameter_ mirrors axis 0 for the 1-D slider UI.
    goto_desired_parameter_ =
        steering_->RequestForPose(pose, goal).desired_parameter;
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
        if (graph_ready_) {
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
    goto_target_ = glm::vec2(hit.x, hit.z) / std::max(display_scale_, kEpsilon);

    if (!steering_.has_value()) {
        CalibrateSteering();  // one-time; a few seconds of offline streaming
    }
    if (steering_.has_value()) {
        steering_->Reset();
    }
    goto_active_ = steering_.has_value();
    if (goto_active_) {
        playing_ = true;
        goto_status_ = "Walking to target.";
    }
    return true;
}

void PmgViewerWorkspace::Update(float delta_seconds) {
    // Graph runtime: drive the RuntimeController and render its streamed pose.
    if (GraphRuntimeActive()) {
        if (playing_) {
            const bool was_transitioning =
                graph_controller_->IsTransitioning();
            if (goto_active_ && steering_.has_value()) {
                UpdateGotoSteering(graph_controller_->CurrentPose());
            }
            pmg::RuntimeControlRequest request;
            // A PMG stream repeatedly traverses edges, including a node's
            // self-edge when the requested node and parameter stay unchanged.
            // That transition blends the end of one generated clip into the
            // start of the next; leaving the request empty would raw-wrap the
            // extracted cycle and expose its joint-space seam.
            request.desired_node = graph_desired_node_;
            // Active goto supplies a full per-axis steering vector; manual
            // streaming uses the 1-D slider (axis 0) padded to node dimension.
            request.desired_parameter =
                (goto_active_ && !goto_desired_parameter_.empty())
                    ? goto_desired_parameter_
                    : DesiredParameterForNode(graph_desired_node_);
            graph_controller_->Update(delta_seconds * playback_speed_, request);
            if (!was_transitioning &&
                graph_controller_->IsTransitioning()) {
                const auto transition =
                    graph_controller_->ActiveTransitionDiagnostics();
                if (transition.has_value()) {
                    RecordTransitionMarker(*transition);
                }
            }
        }
        const pmg::Pose pose = graph_controller_->CurrentPose();
        RecordRuntimeTracePoint(pose);
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
    RebuildScene(pose);
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
    pmg_space_ready_ = true;
    RecomputeSteeringCurve();
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
}

// --- ImGui UI ---------------------------------------------------------------
//
// Everything lives in one window: Workflow + Transport stay visible at the
// top (they are needed in every mode), the rest is tabbed. Seven floating
// windows buried the 3D view and each other.

void PmgViewerWorkspace::BuildUi() {
    HandleShortcuts();

    ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520.0f, 780.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("PMG Viewer");

    BuildWorkflowSection();
    ImGui::Separator();
    BuildTransportSection();
    ImGui::Separator();

    if (ImGui::BeginTabBar("##viewer_tabs")) {
        if (ImGui::BeginTabItem("Inputs")) {
            BuildInputsSection();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Motion Space")) {
            BuildMotionSpaceSection();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Transition Grid")) {
            BuildDistanceGridSection();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("PMG Runtime")) {
            BuildGraphSection();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Display")) {
            BuildDisplaySection();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
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

bool PmgViewerWorkspace::ParameterSliderWithTicks(const char* label, float* value,
                                         float min_value, float max_value, int axis) {
    const float frame_width = ImGui::CalcItemWidth();
    const bool changed = ImGui::SliderFloat(label, value, min_value, max_value, "%.3f");

    // Tick marks at the example parameters along the slider frame's bottom
    // edge, so the user sees where real clips sit on this blend axis.
    const float span = max_value - min_value;
    if (span > kEpsilon && !pmg_examples_.empty()) {
        const ImVec2 rect_min = ImGui::GetItemRectMin();
        const float bottom = ImGui::GetItemRectMax().y;
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        for (const PmgExample& example : pmg_examples_) {
            if (axis >= static_cast<int>(example.parameter.size())) {
                continue;
            }
            const float alpha =
                std::clamp((example.parameter[axis] - min_value) / span, 0.0f, 1.0f);
            const float x = rect_min.x + alpha * frame_width;
            draw_list->AddLine(ImVec2(x, bottom - 4.0f), ImVec2(x, bottom + 2.0f),
                               IM_COL32(255, 255, 255, 170), 1.0f);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::TextDisabled("ticks = example clips (ctrl+click to type)");
            for (const PmgExample& example : pmg_examples_) {
                const float tick = axis < static_cast<int>(example.parameter.size())
                                       ? example.parameter[axis]
                                       : 0.0f;
                ImGui::Text("%.3f  %s", tick, example.label.c_str());
            }
            ImGui::EndTooltip();
        }
    }
    return changed;
}

void PmgViewerWorkspace::BuildWorkflowSection() {
    ImGui::TextWrapped("%s", status_message_.c_str());
    ImGui::Separator();

    // One-line next-step hint: keeps the build order (clip -> samples -> graph)
    // discoverable without the multi-line step strip.
    const char* next_hint = nullptr;
    if (clip_.NumFrames() == 0) {
        next_hint = "Next: load a clip in Inputs";
    } else if (pmg_examples_.empty()) {
        next_hint = "Next: add motion samples in Inputs";
    } else if (!graph_ready_) {
        next_hint = "Next: build a graph in PMG Runtime";
    }
    if (next_hint != nullptr) {
        ImGui::TextColored(ImVec4(0.55f, 0.70f, 0.95f, 1.0f), "%s", next_hint);
        ImGui::Separator();
    }

    ImGui::TextDisabled("Mode");

    // Mode lives only here. Clip playback is always available; parametric blend
    // unlocks with a space; graph runtime unlocks once a graph is built.
    int mode_int = static_cast<int>(mode_);
    ImGui::RadioButton("Clip playback", &mode_int,
                       static_cast<int>(ViewerPlaybackMode::ClipPlayback));

    auto gated_radio = [&](const char* label, ViewerPlaybackMode value, bool enabled,
                           const char* hint) {
        if (enabled) {
            ImGui::RadioButton(label, &mode_int, static_cast<int>(value));
            return;
        }
        ImGui::BeginDisabled();
        int disabled = mode_int;
        ImGui::RadioButton(label, &disabled, static_cast<int>(value));
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::TextDisabled("(%s)", hint);
    };
    gated_radio("Parametric blend", ViewerPlaybackMode::ParametricBlend, pmg_space_ready_,
                "add a clip");
    gated_radio("Graph runtime", ViewerPlaybackMode::GraphRuntime,
                graph_ready_ && graph_controller_.has_value(), "build graph");
    mode_ = static_cast<ViewerPlaybackMode>(mode_int);
}

void PmgViewerWorkspace::BuildTransportSection() {
    if (ImGui::Button(playing_ ? "Pause" : "Play ")) {
        playing_ = !playing_;
    }
    const bool time_mode = !GraphRuntimeActive();
    ImGui::SameLine();
    ImGui::BeginDisabled(!time_mode);
    if (ImGui::Button("< Frame")) {
        StepFrame(-1);
    }
    ImGui::SameLine();
    if (ImGui::Button("Frame >")) {
        StepFrame(1);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Reset")) {
        ResetPlayback();
    }
    if (GraphRuntimeActive() && graph_controller_.has_value()) {
        ImGui::SameLine();
        if (ImGui::Button("Restart graph")) {
            graph_controller_->Start(graph_controller_->CurrentNode(),
                                     graph_controller_->CurrentParameter(), graph_fps_);
            ResetRuntimeTrace();
        }
    }

    ImGui::SliderFloat("Speed", &playback_speed_, 0.1f, 3.0f, "%.2fx");

    // Phase scrub: clip & blend only (the graph's phase is owned by the
    // controller). Phase is the stored clock, so the slider reads/writes it
    // directly. Auto-pause while dragging so playback doesn't fight it.
    if (time_mode) {
        float phase = current_phase_;
        if (ImGui::SliderFloat("Phase", &phase, 0.0f, 1.0f, "%.3f")) {
            current_phase_ = phase;
        }
        if (ImGui::IsItemActive()) {
            playing_ = false;
        }
    } else if (graph_controller_.has_value()) {
        ImGui::Text("Phase %.3f", graph_controller_->CurrentPhase());
    }

    // Path preview: ghost skeletons sampled across the active clip so the start,
    // middle(s), and end are visible at once (clip & blend modes).
    if (time_mode) {
        ImGui::Checkbox("Path preview", &path_preview_enabled_);
        if (path_preview_enabled_) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(160.0f);
            ImGui::SliderInt("Ghosts", &path_preview_count_, 2, 12);
            ImGui::SameLine();
            ImGui::TextDisabled("(cyan=start -> amber=end)");
        }
    }
}

void PmgViewerWorkspace::BuildInputsSection() {
    ImGui::Text("%zu BVH files", bvh_files_.size());

    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputTextWithHint("##clip_filter", "filter by name...", clip_filter_,
                             sizeof(clip_filter_));

    std::string needle = clip_filter_;
    std::transform(needle.begin(), needle.end(), needle.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (ImGui::BeginListBox("##clips", ImVec2(-1.0f, 220.0f))) {
        for (int i = 0; i < static_cast<int>(bvh_files_.size()); ++i) {
            const std::string name = bvh_files_[i].filename().string();
            if (!needle.empty()) {
                std::string hay = name;
                std::transform(hay.begin(), hay.end(), hay.begin(),
                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (hay.find(needle) == std::string::npos) {
                    continue;
                }
            }
            const bool selected = (i == selected_file_index_);
            if (ImGui::Selectable(name.c_str(), selected)) {
                LoadClip(i);
            }
        }
        ImGui::EndListBox();
    }

    if (clip_.NumFrames() > 0) {
        ImGui::TextDisabled("Loaded: %d frames @ %.0f fps, %d joints", clip_.NumFrames(),
                            clip_.frames_per_second, skeleton_.NumJoints());
        if (ImGui::CollapsingHeader("BVH skeleton / channel diagnostics")) {
            for (int joint_index = 0; joint_index < skeleton_.NumJoints(); ++joint_index) {
                const pmg::Joint& joint = skeleton_.joints[joint_index];
                ImGui::BulletText("%s  parent=%d  channels=%zu",
                                  joint.name.c_str(), joint.parent_index,
                                  joint.channels.size());
            }
            ImGui::TextDisabled("Units: %s", artifact_units_.c_str());
        }
    }

    ImGui::Separator();
    // Parameter dimension is locked once the space has samples (every sample
    // must share one dimension); edit it only on an empty space.
    ImGui::BeginDisabled(!pmg_examples_.empty());
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::InputInt("Parameter dimension", &pmg_dimension_)) {
        pmg_dimension_ = std::clamp(pmg_dimension_, 1, 4);
        ResizeParameterVectors();
    }
    ImGui::EndDisabled();

    ResizeParameterVectors();  // keep next_example_parameter_ sized to dimension
    ImGui::SetNextItemWidth(220.0f);
    ImGui::InputScalarN("Blend parameter", ImGuiDataType_Float,
                        next_example_parameter_.data(), pmg_dimension_);
    ImGui::SameLine();
    if (ImGui::Button("Add motion sample")) {
        AddCurrentClipToSpace(next_example_parameter_);
    }
}

void PmgViewerWorkspace::BuildDisplaySection() {
    ImGui::SliderFloat("Skeleton scale", &skeleton_scale_, 0.1f, 5.0f, "%.2fx");
    ImGui::SliderFloat("Display scale", &display_scale_, 1.0f, 40.0f, "%.1fx");
}

void PmgViewerWorkspace::BuildMotionSpaceSection() {
    ImGui::Text("Space: %s   dimension: %d   samples: %zu",
                pmg_space_ready_ ? pmg_space_.Name().c_str() : "(none)",
                pmg_space_ready_ ? pmg_space_.ParameterDimension() : 0,
                pmg_examples_.size());
    ImGui::TextDisabled(
        "Graph NODE = this parametric motion space: a blend of its sample clips.");
    ImGui::TextDisabled(
        "Graph EDGES = sampled transitions between nodes (see the Graph tab).");

    if (pmg_space_ready_) {
        ImGui::Separator();
        ImGui::TextDisabled("Evaluation parameter");
        const int dim = std::max(1, pmg_dimension_);
        for (int axis = 0; axis < dim; ++axis) {
            if (axis >= static_cast<int>(pmg_parameter_.size()) ||
                axis >= static_cast<int>(pmg_parameter_min_.size())) {
                break;
            }
            ImGui::PushID(axis);
            const std::string label =
                "Blend parameter [" + std::to_string(axis) + "]";
            ParameterSliderWithTicks(label.c_str(), &pmg_parameter_[axis],
                                     pmg_parameter_min_[axis],
                                     pmg_parameter_max_[axis], axis);
            ImGui::PopID();
        }
        if (dim > 1) {
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputInt("View axis (1-D plots)", &pmg_view_axis_)) {
                pmg_view_axis_ = std::clamp(pmg_view_axis_, 0, dim - 1);
                RecomputeSteeringCurve();
            }
        }
        DrawParameterSpace(std::clamp(pmg_view_axis_, 0, dim - 1));

        // Preview comes from GenerateClip (root-delta integration). The toggle
        // chooses whether the character traces that integrated trajectory or
        // cycles in place for pose inspection.
        ImGui::Checkbox("In-place preview", &pmg_preview_in_place_);
        ImGui::SameLine();
        ImGui::TextDisabled(pmg_preview_in_place_
                                ? "(root locked: cycle in place)"
                                : "(trajectory: integrated root path)");

        // Steering diagnostic: measured locomotion metrics across the parameter
        // axis. Smooth steering reads as monotone, spike-free curves; the marker
        // shows where the current blend parameter sits on that response.
        if (steering_turn_rate_curve_.size() >= 2) {
            const int axis =
                std::clamp(pmg_view_axis_, 0, std::max(0, pmg_dimension_ - 1));
            const float axis_min = axis < static_cast<int>(pmg_parameter_min_.size())
                                       ? pmg_parameter_min_[axis] : 0.0f;
            const float axis_max = axis < static_cast<int>(pmg_parameter_max_.size())
                                       ? pmg_parameter_max_[axis] : 1.0f;
            const float axis_val = axis < static_cast<int>(pmg_parameter_.size())
                                       ? pmg_parameter_[axis] : 0.0f;
            const float span = std::max(axis_max - axis_min, kEpsilon);
            const float marker_alpha =
                std::clamp((axis_val - axis_min) / span, 0.0f, 1.0f);

            const auto plot_with_marker =
                [marker_alpha](const char* id, const char* caption,
                               const std::vector<float>& curve) {
                    ImGui::TextDisabled("%s", caption);
                    ImGui::PlotLines(id, curve.data(),
                                     static_cast<int>(curve.size()), 0, nullptr,
                                     FLT_MAX, FLT_MAX, ImVec2(0.0f, 48.0f));
                    const ImVec2 plot_min = ImGui::GetItemRectMin();
                    const ImVec2 plot_max = ImGui::GetItemRectMax();
                    const float marker_x =
                        plot_min.x + marker_alpha * (plot_max.x - plot_min.x);
                    ImGui::GetWindowDrawList()->AddLine(
                        ImVec2(marker_x, plot_min.y), ImVec2(marker_x, plot_max.y),
                        IM_COL32(245, 180, 65, 255), 1.5f);
                };

            plot_with_marker("##steering_turn_rate",
                             "Turn rate vs parameter (rad/s) -- smooth = monotone",
                             steering_turn_rate_curve_);
            plot_with_marker("##steering_travel_speed",
                             "Travel speed vs parameter (units/s)",
                             steering_travel_speed_curve_);

            if (pmg_preview_clip_.NumFrames() >= 2) {
                float turn_rate = 0.0f;
                float travel_speed = 0.0f;
                try {
                    turn_rate = pmg::MeasureParameterMetric(
                        pmg::ParameterMetric::kTurnRate, pmg_preview_clip_);
                    travel_speed = pmg::MeasureParameterMetric(
                        pmg::ParameterMetric::kTravelSpeed, pmg_preview_clip_);
                } catch (const std::exception&) {
                    turn_rate = 0.0f;
                    travel_speed = 0.0f;
                }
                ImGui::TextDisabled("current blend: %.3f rad/s, %.3f units/s",
                                    turn_rate, travel_speed);
            }
        }

        const float canonical_phase =
            GraphRuntimeActive() && graph_controller_.has_value()
                ? graph_controller_->CurrentPhase()
                : current_phase_;
        DrawPhaseTimeline(canonical_phase);

        if (!ParametricBlendActive()) {
            ImGui::TextDisabled(
                "Switch mode to Parametric blend to render this evaluation parameter.");
        }
    } else {
        ImGui::TextDisabled("Add motion samples from Inputs.");
    }

    if (!pmg_examples_.empty()) {
        ImGui::Separator();
        ImGui::TextDisabled("Samples (edit parameter / remove)");
        int remove_index = -1;
        bool params_changed = false;
        for (int i = 0; i < static_cast<int>(pmg_examples_.size()); ++i) {
            ImGui::PushID(i);
            pmg::ParameterVector& sample = pmg_examples_[i].parameter;
            sample.resize(static_cast<std::size_t>(std::max(1, pmg_dimension_)), 0.0f);
            ImGui::SetNextItemWidth(60.0f * pmg_dimension_);
            // EnterReturnsTrue: rebuild on commit, not on every per-frame edit.
            if (ImGui::InputScalarN("##param", ImGuiDataType_Float, sample.data(),
                                    pmg_dimension_, nullptr, nullptr, "%.3f",
                                    ImGuiInputTextFlags_EnterReturnsTrue)) {
                params_changed = true;
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("x")) {
                remove_index = i;
            }
            ImGui::SameLine();
            ImGui::TextUnformatted(pmg_examples_[i].label.c_str());
            ImGui::PopID();
        }
        if (remove_index >= 0) {
            pmg_examples_.erase(pmg_examples_.begin() + remove_index);
            RebuildPmgSpace();
            status_message_ = "Removed motion sample.";
        } else if (params_changed) {
            RebuildPmgSpace();
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Clear motion space")) {
        pmg_examples_.clear();
        RebuildPmgSpace();
        status_message_ = "Cleared parametric space.";
    }
}

void PmgViewerWorkspace::DrawParameterSpace(int axis) {
    if (!pmg_space_ready_ || pmg_examples_.empty()) {
        return;
    }
    axis = std::clamp(axis, 0, std::max(0, pmg_dimension_ - 1));
    if (axis >= static_cast<int>(pmg_parameter_min_.size()) ||
        axis >= static_cast<int>(pmg_parameter_.size())) {
        return;
    }
    const float axis_min = pmg_parameter_min_[axis];
    const float axis_max = pmg_parameter_max_[axis];
    const float query_value = pmg_parameter_[axis];

    std::vector<float> weights;
    try {
        // Stencil weights use the full N-D blend; the canvas projects onto axis.
        weights = pmg_space_.ComputeLocalBlendWeights(pmg_parameter_);
    } catch (const std::exception& error) {
        ImGui::TextWrapped("Weight evaluation failed: %s", error.what());
        return;
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Parameter-space samples and local stencil (axis %d)", axis);
    const ImVec2 canvas_origin = ImGui::GetCursorScreenPos();
    const float canvas_width = std::max(120.0f, ImGui::GetContentRegionAvail().x);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const float axis_left = canvas_origin.x + 14.0f;
    const float axis_width = canvas_width - 28.0f;
    const float axis_y = canvas_origin.y + 42.0f;

    draw_list->AddRectFilled(
        canvas_origin,
        ImVec2(canvas_origin.x + canvas_width,
               canvas_origin.y + kParameterCanvasHeight),
        IM_COL32(28, 31, 38, 255), 4.0f);
    draw_list->AddLine(
        ImVec2(axis_left, axis_y),
        ImVec2(axis_left + axis_width, axis_y),
        IM_COL32(150, 155, 165, 255), 2.0f);

    for (std::size_t index = 0; index < pmg_examples_.size(); ++index) {
        const PmgExample& example = pmg_examples_[index];
        const float coord = axis < static_cast<int>(example.parameter.size())
                                ? example.parameter[axis] : 0.0f;
        const float x = ParameterToCanvasX(
            coord, axis_min, axis_max, axis_left, axis_width);
        const float weight = index < weights.size() ? weights[index] : 0.0f;
        const float radius = 5.0f + 7.0f * std::sqrt(std::max(0.0f, weight));
        const ImU32 color =
            weight > kEpsilon ? IM_COL32(245, 180, 65, 255)
                              : IM_COL32(110, 115, 125, 255);
        draw_list->AddCircleFilled(ImVec2(x, axis_y), radius, color);
        draw_list->AddText(
            ImVec2(x - 18.0f, axis_y + 16.0f),
            IM_COL32(205, 208, 215, 255),
            std::to_string(coord).substr(0, 4).c_str());
    }

    const float query_x = ParameterToCanvasX(
        query_value, axis_min, axis_max, axis_left, axis_width);
    draw_list->AddLine(
        ImVec2(query_x, canvas_origin.y + 8.0f),
        ImVec2(query_x, axis_y - 7.0f),
        IM_COL32(80, 205, 255, 255), 2.0f);
    draw_list->AddTriangleFilled(
        ImVec2(query_x, axis_y - 2.0f),
        ImVec2(query_x - 6.0f, axis_y - 10.0f),
        ImVec2(query_x + 6.0f, axis_y - 10.0f),
        IM_COL32(80, 205, 255, 255));
    ImGui::InvisibleButton(
        "##parameter_space_canvas",
        ImVec2(canvas_width, kParameterCanvasHeight));

    for (std::size_t index = 0; index < pmg_examples_.size(); ++index) {
        const float weight = index < weights.size() ? weights[index] : 0.0f;
        ImGui::Text("%s", pmg_examples_[index].label.c_str());
        ImGui::SameLine(265.0f);
        ImGui::ProgressBar(weight, ImVec2(-1.0f, 0.0f),
                           (std::to_string(static_cast<int>(std::lround(weight * 100.0f))) +
                            "%").c_str());
    }

    if (pmg_space_.HasParameterCalibration()) {
        const pmg::ParameterCalibration& calibration =
            pmg_space_.ParameterCalibrationData();
        if (calibration.samples_per_axis > 0) {
            ImGui::TextDisabled(
                "Parameter accuracy: calibrated inversion (%zu metrics, %zu "
                "samples, %d/axis)",
                calibration.metrics.size(), calibration.samples.size(),
                calibration.samples_per_axis);
        } else {
            ImGui::TextDisabled(
                "Parameter accuracy: calibrated inversion (%zu metrics, %zu "
                "imported samples)",
                calibration.metrics.size(), calibration.samples.size());
        }
    } else {
        ImGui::TextDisabled("Parameter accuracy: local Shepard interpolation");
    }
}

void PmgViewerWorkspace::DrawPhaseTimeline(float canonical_phase) {
    if (!pmg_space_ready_ || pmg_examples_.empty()) {
        return;
    }

    ImGui::Separator();
    ImGui::TextDisabled("Canonical phase -> registered example phase / foot contact");
    const float label_width = 210.0f;
    const float row_width =
        std::max(120.0f, ImGui::GetContentRegionAvail().x - label_width);
    const ImVec2 start = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const bool has_warps = pmg_space_.HasExampleTimeWarps();
    const std::vector<pmg::TimeWarp>& warps = pmg_space_.ExampleTimeWarps();

    for (std::size_t index = 0; index < pmg_examples_.size(); ++index) {
        const float row_top = start.y + static_cast<float>(index) * kPhaseRowHeight;
        draw_list->AddText(
            ImVec2(start.x, row_top + 2.0f),
            IM_COL32(210, 212, 218, 255),
            pmg_examples_[index].label.c_str());
        const float timeline_left = start.x + label_width;
        const float timeline_y = row_top + 11.0f;
        draw_list->AddLine(
            ImVec2(timeline_left, timeline_y),
            ImVec2(timeline_left + row_width, timeline_y),
            IM_COL32(100, 104, 112, 255), 2.0f);

        for (const pmg::ContactInterval& contact :
             pmg_examples_[index].contact_intervals) {
            const float x0 = timeline_left +
                contact.StrikePhase(pmg_examples_[index].clip.NumFrames()) * row_width;
            const float x1 = timeline_left +
                contact.LiftPhase(pmg_examples_[index].clip.NumFrames()) * row_width;
            draw_list->AddRectFilled(
                ImVec2(x0, timeline_y - 5.0f),
                ImVec2(std::max(x0 + 1.0f, x1), timeline_y + 5.0f),
                IM_COL32(72, 165, 105, 150), 2.0f);
        }

        const float example_phase =
            has_warps ? warps[index].Evaluate(canonical_phase) : canonical_phase;
        DrawPhaseMarker(
            draw_list, example_phase, timeline_left, timeline_y - 6.0f,
            row_width, 12.0f, IM_COL32(80, 205, 255, 255));

        if (has_warps) {
            for (const float anchor : warps[index].InteriorToPhases()) {
                const float x = timeline_left + anchor * row_width;
                draw_list->AddCircleFilled(
                    ImVec2(x, timeline_y), 2.5f,
                    IM_COL32(240, 185, 70, 255));
            }
        }
    }

    const float total_height =
        static_cast<float>(pmg_examples_.size()) * kPhaseRowHeight;
    ImGui::InvisibleButton(
        "##phase_timeline",
        ImVec2(label_width + row_width, total_height));
    ImGui::TextDisabled(
        "blue = current phase   green = detected contact   gold = registration anchor");
    ImGui::Text("Canonical phase %.3f", canonical_phase);
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

    // Shared GOOD/BAD thresholds -- also drive the graph edge build.
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
