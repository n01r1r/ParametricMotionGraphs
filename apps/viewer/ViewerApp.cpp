#include "ViewerApp.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>

#include "pmg/ForwardKinematics.h"

#ifndef PMG_BVH_DIRECTORY
#define PMG_BVH_DIRECTORY "BVH"
#endif

namespace pmgviewer {

namespace {
constexpr float kEpsilon = 1.0e-6f;

glm::vec3 ToGlm(const pmg::Vec3& v) { return glm::vec3(v.x, v.y, v.z); }

float NormalizedPhase(float time_seconds, float duration_seconds) {
    if (duration_seconds <= kEpsilon) {
        return 0.0f;
    }
    float phase = std::fmod(time_seconds, duration_seconds) / duration_seconds;
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

void ViewerApp::Initialize() {
    DiscoverBvhFiles();
    if (!bvh_files_.empty()) {
        LoadClip(0);
    }
}

void ViewerApp::DiscoverBvhFiles() {
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

float ViewerApp::ComputeGroundOffset(const pmg::Skeleton& skeleton,
                                     const pmg::MotionClip& clip) {
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

void ViewerApp::LoadClip(int file_index) {
    if (file_index < 0 || file_index >= static_cast<int>(bvh_files_.size())) {
        return;
    }
    try {
        const pmg::BvhData data = pmg::BvhLoader::Load(bvh_files_[file_index].string());
        skeleton_ = data.skeleton;
        clip_ = data.clip;
        ground_offset_ = ComputeGroundOffset(skeleton_, clip_);
        current_time_seconds_ = 0.0f;
        selected_file_index_ = file_index;
        mode_ = ViewerPlaybackMode::ClipPlayback;
        status_message_ = "Loaded " + bvh_files_[file_index].filename().string();
    } catch (const std::exception& error) {
        status_message_ = std::string("Load failed: ") + error.what();
    }
}

bool ViewerApp::ParametricBlendActive() const {
    return mode_ == ViewerPlaybackMode::ParametricBlend && pmg_space_ready_;
}

bool ViewerApp::GraphRuntimeActive() const {
    return mode_ == ViewerPlaybackMode::GraphRuntime && graph_ready_ &&
           graph_controller_.has_value();
}

const char* ViewerApp::ModeName(ViewerPlaybackMode mode) {
    switch (mode) {
        case ViewerPlaybackMode::ClipPlayback:
            return "Clip playback";
        case ViewerPlaybackMode::ParametricBlend:
            return "Parametric blend";
        case ViewerPlaybackMode::GraphRuntime:
            return "Graph runtime";
    }
    return "Unknown";
}

void ViewerApp::ResetPlayback() {
    current_time_seconds_ = 0.0f;
    playback_speed_ = 1.0f;
    playing_ = true;
}

const pmg::Skeleton& ViewerApp::ActiveSkeleton() const {
    if (ParametricBlendActive() || GraphRuntimeActive()) {
        return pmg_skeleton_;
    }
    return skeleton_;
}

float ViewerApp::ActiveReferenceDuration() const {
    if (ParametricBlendActive() && !pmg_examples_.empty()) {
        return pmg_examples_.front().clip.DurationSeconds();
    }
    if (clip_.NumFrames() > 0) {
        return clip_.DurationSeconds();
    }
    return 0.0f;
}

pmg::Pose ViewerApp::CurrentPose() const {
    const float phase = NormalizedPhase(current_time_seconds_, ActiveReferenceDuration());
    if (ParametricBlendActive()) {
        return pmg_space_.EvaluatePose({pmg_parameter_}, phase);
    }
    if (clip_.NumFrames() == 0) {
        return pmg::Pose{};
    }
    return clip_.SampleNormalizedPhase(phase);
}

void ViewerApp::RebuildScene(const pmg::Pose& pose) {
    const pmg::Skeleton& skeleton = ActiveSkeleton();
    if (pose.NumJoints() != skeleton.NumJoints() || skeleton.NumJoints() == 0) {
        return;
    }

    const std::vector<pmg::Vec3> positions =
        pmg::ComputeJointWorldPositions(skeleton, pose);
    const float ground_offset =
        (ParametricBlendActive() || GraphRuntimeActive()) ? pmg_ground_offset_ : ground_offset_;

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

    // Pass 2: apply the render scale = display_scale_ (native->display, replaces
    // the old loader x10) times the user skeleton_scale_. Horizontal scaling
    // pivots on the centroid; vertical scaling anchors at y = 0 so the feet stay
    // on the floor.
    const float render_scale = display_scale_ * skeleton_scale_;
    glm::vec3 centroid(0.0f);
    scene_.joints.clear();
    scene_.joints.reserve(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        glm::vec3& world_position = world_positions[i];
        world_position.x =
            horizontal_center.x + (world_position.x - horizontal_center.x) * render_scale;
        world_position.z =
            horizontal_center.y + (world_position.z - horizontal_center.y) * render_scale;
        world_position.y *= render_scale;
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

    scene_.focus_point = centroid;
    if (follow_centroid_) {
        camera_.SetTarget(centroid);
    }
}

void ViewerApp::Update(float delta_seconds) {
    // Graph runtime: drive the RuntimeController and render its streamed pose.
    if (GraphRuntimeActive()) {
        if (playing_) {
            pmg::RuntimeControlRequest request;
            request.desired_node = graph_controller_->CurrentNode();
            request.desired_parameter = {graph_desired_parameter_};
            graph_controller_->Update(delta_seconds * playback_speed_, request);
        }
        RebuildScene(graph_controller_->CurrentPose());
        return;
    }

    if (playing_) {
        current_time_seconds_ += delta_seconds * playback_speed_;
    }
    RebuildScene(CurrentPose());
}

void ViewerApp::AddCurrentClipToSpace(float parameter) {
    if (clip_.NumFrames() == 0) {
        status_message_ = "Load a clip before adding it to the PMG space.";
        return;
    }
    if (!pmg_examples_.empty() &&
        skeleton_.NumJoints() != pmg_skeleton_.NumJoints()) {
        status_message_ = "Incompatible skeleton: PMG space needs matching joints.";
        return;
    }
    if (pmg_examples_.empty()) {
        pmg_skeleton_ = skeleton_;
    }

    const std::string label =
        selected_file_index_ >= 0
            ? bvh_files_[selected_file_index_].filename().string()
            : "clip";
    pmg_examples_.push_back({label, parameter, clip_});
    next_example_parameter_ = parameter + 1.0f;
    RebuildPmgSpace();
    status_message_ = "Added '" + label + "' at parameter " + std::to_string(parameter);
}

void ViewerApp::RebuildPmgSpace() {
    if (pmg_examples_.empty()) {
        pmg_space_ready_ = false;
        mode_ = ViewerPlaybackMode::ClipPlayback;
        return;
    }

    pmg_space_ = pmg::ParametricMotionSpace("viewer_space", 1);
    pmg_parameter_min_ = std::numeric_limits<float>::infinity();
    pmg_parameter_max_ = -std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    for (const PmgExample& example : pmg_examples_) {
        pmg_space_.AddExample({example.parameter}, example.clip);
        pmg_parameter_min_ = std::min(pmg_parameter_min_, example.parameter);
        pmg_parameter_max_ = std::max(pmg_parameter_max_, example.parameter);
        // Ground offset uses the global lowest point across all examples.
        const float offset = ComputeGroundOffset(pmg_skeleton_, example.clip);
        min_y = std::min(min_y, -offset);
    }
    pmg_ground_offset_ = std::isfinite(min_y) ? -min_y : 0.0f;

    if (pmg_parameter_max_ <= pmg_parameter_min_) {
        pmg_parameter_max_ = pmg_parameter_min_ + 1.0f;
    }
    pmg_parameter_ = std::clamp(pmg_parameter_, pmg_parameter_min_, pmg_parameter_max_);
    pmg_space_ready_ = true;
}

// --- ImGui panels ----------------------------------------------------------

void ViewerApp::BuildUi() {
    BuildPlaybackPanel();
    BuildClipPanel();
    BuildCameraPanel();
    BuildPmgPanel();
    BuildHeatmapPanel();
    BuildGraphPanel();
}

void ViewerApp::BuildPlaybackPanel() {
    ImGui::Begin("Playback");
    ImGui::TextWrapped("%s", status_message_.c_str());
    ImGui::Separator();

    ImGui::Text("Mode: %s", ModeName(mode_));
    if (selected_file_index_ >= 0 &&
        selected_file_index_ < static_cast<int>(bvh_files_.size())) {
        ImGui::TextWrapped("Clip: %s",
                           bvh_files_[selected_file_index_].string().c_str());
    } else {
        ImGui::TextDisabled("Clip: none");
    }
    if (clip_.NumFrames() > 0) {
        ImGui::Text("Frames: %d   FPS: %.1f", clip_.NumFrames(), clip_.frames_per_second);
        ImGui::Text("Joints: %d", skeleton_.NumJoints());
    }
    ImGui::Separator();

    ImGui::Checkbox("Playing", &playing_);
    ImGui::SliderFloat("Speed", &playback_speed_, 0.1f, 3.0f, "%.2fx");
    ImGui::SliderFloat("Skeleton scale", &skeleton_scale_, 0.1f, 5.0f, "%.2fx");

    const float duration = ActiveReferenceDuration();
    float phase = NormalizedPhase(current_time_seconds_, duration);
    if (ImGui::SliderFloat("Phase", &phase, 0.0f, 1.0f, "%.3f")) {
        current_time_seconds_ = phase * duration;
    }
    if (ImGui::Button("Reset Playback")) {
        ResetPlayback();
    }
    ImGui::End();
}

void ViewerApp::BuildClipPanel() {
    ImGui::Begin("Clips");
    ImGui::Text("%zu BVH files", bvh_files_.size());
    if (ImGui::BeginListBox("##clips", ImVec2(-1.0f, 240.0f))) {
        for (int i = 0; i < static_cast<int>(bvh_files_.size()); ++i) {
            const bool selected = (i == selected_file_index_);
            if (ImGui::Selectable(bvh_files_[i].filename().string().c_str(), selected)) {
                LoadClip(i);
            }
        }
        ImGui::EndListBox();
    }
    ImGui::Separator();
    ImGui::InputFloat("New example parameter", &next_example_parameter_);
    if (ImGui::Button("Add loaded clip to PMG space")) {
        AddCurrentClipToSpace(next_example_parameter_);
    }
    ImGui::End();
}

void ViewerApp::BuildCameraPanel() {
    ImGui::Begin("Camera");
    ImGui::Checkbox("Follow motion centroid", &follow_centroid_);
    const glm::vec3 target = camera_.Target();
    ImGui::Text("Target: %.1f, %.1f, %.1f", target.x, target.y, target.z);
    ImGui::TextDisabled("Drag left mouse to orbit, scroll to zoom.");
    if (ImGui::Button("Reset Camera")) {
        camera_.Reset();
        follow_centroid_ = true;
    }
    ImGui::End();
}

void ViewerApp::BuildPmgPanel() {
    ImGui::Begin("Parametric Blend");
    ImGui::TextWrapped(
        "Live parametric blend over added example clips "
        "(pmg_core ParametricMotionSpace).");
    ImGui::Text("Examples: %zu", pmg_examples_.size());
    for (const PmgExample& example : pmg_examples_) {
        ImGui::BulletText("%s  (param %.2f)", example.label.c_str(), example.parameter);
    }
    ImGui::Separator();

    // Playback-mode selector. Clip playback is always available; parametric
    // blend unlocks once a space exists; graph runtime unlocks once a graph is
    // built (see the Graph Runtime panel).
    int mode_int = static_cast<int>(mode_);
    ImGui::TextDisabled("Playback mode");
    ImGui::RadioButton("Clip playback", &mode_int,
                       static_cast<int>(ViewerPlaybackMode::ClipPlayback));

    if (pmg_space_ready_) {
        ImGui::RadioButton("Parametric blend", &mode_int,
                           static_cast<int>(ViewerPlaybackMode::ParametricBlend));
    } else {
        ImGui::BeginDisabled();
        int blend_disabled = mode_int;
        ImGui::RadioButton("Parametric blend", &blend_disabled,
                           static_cast<int>(ViewerPlaybackMode::ParametricBlend));
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::TextDisabled("(add a clip)");
    }

    if (graph_ready_) {
        ImGui::RadioButton("Graph runtime", &mode_int,
                           static_cast<int>(ViewerPlaybackMode::GraphRuntime));
    } else {
        ImGui::BeginDisabled();
        int graph_disabled = mode_int;
        ImGui::RadioButton("Graph runtime", &graph_disabled,
                           static_cast<int>(ViewerPlaybackMode::GraphRuntime));
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::TextDisabled("(build graph)");
    }

    mode_ = static_cast<ViewerPlaybackMode>(mode_int);

    if (ParametricBlendActive()) {
        ImGui::SliderFloat("Blend parameter", &pmg_parameter_,
                           pmg_parameter_min_, pmg_parameter_max_, "%.3f");
    }

    ImGui::Separator();
    if (ImGui::Button("Clear Parametric Space")) {
        pmg_examples_.clear();
        RebuildPmgSpace();
        status_message_ = "Cleared parametric space.";
    }
    ImGui::End();
}

// --- Distance Grid heatmap (transition visualization) ----------------------

void ViewerApp::RecomputeHeatmap() {
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
        if (target.skeleton.NumJoints() != skeleton_.NumJoints()) {
            heatmap_status_ = "Target skeleton joint count differs from source.";
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

void ViewerApp::SaveHeatmapCsv() {
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

void ViewerApp::BuildHeatmapPanel() {
    ImGui::Begin("Distance Grid");

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

    ImGui::SliderInt("Window", &heatmap_config_.window_size, 1, 15);
    ImGui::SliderInt("Source stride", &heatmap_config_.source_frame_stride, 1, 5);
    ImGui::SliderInt("Target stride", &heatmap_config_.target_frame_stride, 1, 5);
    ImGui::SliderFloat("Source phase start", &heatmap_config_.source_phase_start, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("Source phase end", &heatmap_config_.source_phase_end, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("Target phase start", &heatmap_config_.target_phase_start, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("Target phase end", &heatmap_config_.target_phase_end, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("TGOOD", &heatmap_tgood_, 0.0f, 2.0f, "%.3f");
    ImGui::SliderFloat("TBAD", &heatmap_tbad_, 0.0f, 2.0f, "%.3f");

    if (ImGui::Button("Recompute")) {
        RecomputeHeatmap();
    }
    ImGui::SameLine();
    if (ImGui::Button("Save CSV")) {
        SaveHeatmapCsv();
    }
    ImGui::TextWrapped("%s", heatmap_status_.c_str());
    ImGui::Separator();

    if (!heatmap_ready_) {
        ImGui::TextDisabled("No grid yet. Pick a target and Recompute.");
        ImGui::End();
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
    ImGui::TextDisabled("y = source phase, x = target phase");
    ImGui::TextDisabled("blue = low (good)  red = high (bad)  white = min cell");
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
        const char* classification = distance <= heatmap_tgood_   ? "GOOD"
                                     : distance >= heatmap_tbad_   ? "BAD"
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
        const pmg::RigidAlignment2D& alignment = heatmap_grid_.alignments[
            static_cast<std::size_t>(heatmap_selected_source_index_) * target_count +
            heatmap_selected_target_index_];
        ImGui::Text("selected alignment: yaw %.3f rad  dx %.2f  dz %.2f",
                    alignment.theta, alignment.dx, alignment.dz);
    }

    ImGui::End();
}

// --- Graph runtime (PMG streaming) -----------------------------------------

void ViewerApp::BuildGraphRuntime() {
    graph_ready_ = false;
    graph_controller_.reset();

    if (!pmg_space_ready_ || pmg_examples_.empty()) {
        graph_status_ = "Add at least one clip to the parametric space first.";
        return;
    }
    if (graph_tbad_ < graph_tgood_) {
        graph_tbad_ = graph_tgood_;
    }

    try {
        graph_ = pmg::ParametricMotionGraph{};
        const int node = graph_.AddNode("viewer_node", pmg_space_);

        pmg::PmgBuilderConfig config;
        config.source_sample_count = 8;     // small for interactive build
        config.target_sample_count = 32;
        config.generated_frame_count = graph_frame_count_;
        config.generated_frames_per_second = graph_fps_;
        config.good_transition_threshold = graph_tgood_;
        config.bad_transition_threshold = graph_tbad_;

        pmg::PmgEdge edge = pmg::PmgBuilder::BuildEdge(
            pmg_skeleton_, node, node, pmg_space_, pmg_space_, config);
        if (edge.samples.empty()) {
            graph_status_ =
                "No valid self-edge (empty box). Raise TGOOD or use a looping clip.";
            return;
        }
        graph_.AddEdge(edge);

        graph_controller_.emplace(graph_);
        graph_controller_->SetSkeleton(pmg_skeleton_);  // paper-faithful runtime align
        graph_desired_parameter_ =
            std::clamp(graph_desired_parameter_, pmg_parameter_min_, pmg_parameter_max_);
        graph_controller_->Start(node, {graph_desired_parameter_}, graph_frame_count_, graph_fps_);

        graph_ready_ = true;
        mode_ = ViewerPlaybackMode::GraphRuntime;
        graph_status_ = "Graph built: " + std::to_string(edge.samples.size()) +
                        " transition samples. Switched to Graph Runtime.";
    } catch (const std::exception& error) {
        graph_status_ = std::string("Graph build failed: ") + error.what();
    }
}

void ViewerApp::BuildGraphPanel() {
    ImGui::Begin("Graph Runtime");
    ImGui::TextWrapped(
        "PMG streaming (paper Sec 4-5): builds a self-edge over the parametric "
        "space and streams motion with point-cloud-aligned transitions.");
    ImGui::Text("Space examples: %zu", pmg_examples_.size());

    ImGui::SliderFloat("Edge TGOOD", &graph_tgood_, 0.0f, 2.0f, "%.3f");
    ImGui::SliderFloat("Edge TBAD", &graph_tbad_, 0.0f, 4.0f, "%.3f");
    if (ImGui::Button("Build Graph")) {
        BuildGraphRuntime();
    }
    ImGui::TextWrapped("%s", graph_status_.c_str());
    ImGui::Separator();

    if (graph_ready_ && graph_controller_.has_value()) {
        bool active = (mode_ == ViewerPlaybackMode::GraphRuntime);
        if (ImGui::Checkbox("Run (Graph Runtime mode)", &active)) {
            mode_ = active ? ViewerPlaybackMode::GraphRuntime
                           : ViewerPlaybackMode::ClipPlayback;
        }
        ImGui::SliderFloat("Desired parameter", &graph_desired_parameter_,
                           pmg_parameter_min_, pmg_parameter_max_, "%.3f");
        ImGui::Text("Node %d   phase %.2f   transitions %d   %s",
                    graph_controller_->CurrentNode(),
                    graph_controller_->CurrentPhase(),
                    graph_controller_->CompletedTransitions(),
                    graph_controller_->IsTransitioning() ? "[transitioning]" : "");
        if (ImGui::Button("Restart graph")) {
            graph_controller_->Start(graph_controller_->CurrentNode(),
                                     {graph_desired_parameter_},
                                     graph_frame_count_, graph_fps_);
        }
    } else {
        ImGui::TextDisabled("No graph yet. Add clips to the space and Build Graph.");
    }
    ImGui::End();
}

}  // namespace pmgviewer
