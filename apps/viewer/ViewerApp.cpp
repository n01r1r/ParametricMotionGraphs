#include "ViewerApp.h"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
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

float WrapPi(float angle_radians) {
    return angle_radians -
           2.0f * pmg::kPi * std::round(angle_radians / (2.0f * pmg::kPi));
}

// World yaw of the root's +Z reference axis.
float HeadingOfPose(const pmg::Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const pmg::Vec3 forward = pmg::Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

// Constant angle from the root's +Z reference to the actual travel direction,
// estimated as a displacement-weighted circular mean over a clip. BVH corpora
// differ in which root axis faces travel; steering must aim the travel
// direction, not the reference axis. (Mirrors the CLI --goto estimate.)
float EstimateTravelHeadingOffset(const pmg::MotionClip& clip) {
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    for (int frame = 0; frame + 1 < clip.NumFrames(); ++frame) {
        const pmg::Vec3 step =
            clip.frames[frame + 1].root_position - clip.frames[frame].root_position;
        const float step_length = std::sqrt(step.x * step.x + step.z * step.z);
        if (step_length <= 1.0e-5f) {
            continue;
        }
        const float travel_heading = std::atan2(step.x, step.z);
        const float reference_heading = HeadingOfPose(clip.frames[frame]);
        const float offset = WrapPi(travel_heading - reference_heading);
        sum_sin += step_length * std::sin(offset);
        sum_cos += step_length * std::cos(offset);
    }
    return std::atan2(sum_sin, sum_cos);
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
    glm::vec3 centroid(0.0f);
    scene_.joints.clear();
    scene_.joints.reserve(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        glm::vec3& world_position = world_positions[i];
        world_position.x = display_scale_ *
            (horizontal_center.x + (world_position.x - horizontal_center.x) * skeleton_scale_);
        world_position.z = display_scale_ *
            (horizontal_center.y + (world_position.z - horizontal_center.y) * skeleton_scale_);
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

    // Goto target gizmo: ground sphere plus a beacon roughly character-tall
    // (native target back into display space, same transform as the root).
    scene_.marker_points.clear();
    scene_.marker_lines.clear();
    if (goto_active_ && GraphRuntimeActive()) {
        const glm::vec3 base(goto_target_.x * display_scale_, 0.0f,
                             goto_target_.y * display_scale_);
        const glm::vec3 top = base + glm::vec3(0.0f, 1.8f * display_scale_, 0.0f);
        scene_.marker_points.push_back(base);
        scene_.marker_lines.push_back({base, top});
    }

    scene_.focus_point = centroid;
    if (follow_centroid_) {
        camera_.SetTarget(centroid);
    }
}

// --- Goto steering (viewer port of CLI --goto) ------------------------------

// Achieved world turn rate when streaming the graph at one held parameter.
// Differs from the example clips' own turn rates: each self-transition plays
// only the target-phase -> source-gate slice per cycle, so the net heading
// advance per hop is that slice's heading change (including sway).
void ViewerApp::CalibrateSteering() {
    steering_ = SteeringCalibration{};
    if (!graph_ready_ || pmg_examples_.empty()) {
        goto_status_ = "Build a graph first.";
        return;
    }

    constexpr int kCalibrationPoints = 5;
    constexpr float kCalibrationSeconds = 8.0f;
    const float dt = 1.0f / graph_fps_;
    const int total_frames = static_cast<int>(kCalibrationSeconds * graph_fps_);

    for (int point = 0; point < kCalibrationPoints; ++point) {
        const float alpha =
            static_cast<float>(point) / static_cast<float>(kCalibrationPoints - 1);
        const float parameter =
            pmg_parameter_min_ + alpha * (pmg_parameter_max_ - pmg_parameter_min_);

        pmg::PointCloudAlignment alignment(pmg_skeleton_);
        pmg::RuntimeController controller(graph_, alignment);
        controller.Start(0, {parameter}, graph_frame_count_, graph_fps_);

        pmg::RuntimeControlRequest request;
        request.desired_node = 0;
        request.desired_parameter = {parameter};

        float unwrapped = 0.0f;
        float previous_heading = HeadingOfPose(controller.CurrentPose());
        for (int frame = 0; frame < total_frames; ++frame) {
            controller.Update(dt, request);
            const float heading = HeadingOfPose(controller.CurrentPose());
            unwrapped += WrapPi(heading - previous_heading);
            previous_heading = heading;
        }
        steering_.parameters.push_back(parameter);
        steering_.rates.push_back(unwrapped / (static_cast<float>(total_frames) * dt));
    }

    steering_.lowest_rate =
        *std::min_element(steering_.rates.begin(), steering_.rates.end());
    steering_.highest_rate =
        *std::max_element(steering_.rates.begin(), steering_.rates.end());
    if (steering_.highest_rate - steering_.lowest_rate < 1.0e-4f) {
        goto_status_ = "Calibration found no turn-rate variation; cannot steer.";
        return;
    }
    steering_.travel_offset = EstimateTravelHeadingOffset(pmg_examples_.front().clip);
    steering_.cycle_seconds = static_cast<float>(graph_frame_count_) / graph_fps_;
    steering_.ready = true;
    goto_status_ = "Calibrated: achieved turn rates " +
                   std::to_string(steering_.lowest_rate) + " .. " +
                   std::to_string(steering_.highest_rate) + " rad/s.";
}

float ViewerApp::ParameterForRate(float desired_rate) const {
    const float clamped =
        std::clamp(desired_rate, steering_.lowest_rate, steering_.highest_rate);
    for (std::size_t segment = 0; segment + 1 < steering_.rates.size(); ++segment) {
        const float rate_a = steering_.rates[segment];
        const float rate_b = steering_.rates[segment + 1];
        if ((clamped - rate_a) * (clamped - rate_b) <= 0.0f &&
            std::abs(rate_b - rate_a) > 1.0e-6f) {
            const float alpha = (clamped - rate_a) / (rate_b - rate_a);
            return steering_.parameters[segment] +
                   alpha * (steering_.parameters[segment + 1] - steering_.parameters[segment]);
        }
    }
    std::size_t best = 0;
    for (std::size_t point = 1; point < steering_.rates.size(); ++point) {
        if (std::abs(steering_.rates[point] - clamped) <
            std::abs(steering_.rates[best] - clamped)) {
            best = point;
        }
    }
    return steering_.parameters[best];
}

void ViewerApp::UpdateGotoSteering(const pmg::Pose& pose) {
    const float dx = goto_target_.x - pose.root_position.x;
    const float dz = goto_target_.y - pose.root_position.z;
    const float distance = std::sqrt(dx * dx + dz * dz);
    if (distance <= goto_tolerance_) {
        goto_active_ = false;
        goto_swinging_ = false;
        goto_status_ = "Target reached.";
        return;
    }

    const float desired_heading = std::atan2(dx, dz);
    const float travel_heading = WrapPi(HeadingOfPose(pose) + steering_.travel_offset);
    const float heading_error = WrapPi(desired_heading - travel_heading);
    const float desired_rate = heading_error / steering_.cycle_seconds;

    // When the demanded turn exceeds what its own direction offers (target
    // inside that branch's minimum turning radius), going the long way around
    // with the tightest branch converges; the wide branch orbits forever.
    // Hysteresis keeps the swing from chattering.
    const float tightest_rate =
        std::abs(steering_.lowest_rate) > std::abs(steering_.highest_rate)
            ? steering_.lowest_rate
            : steering_.highest_rate;
    if (!goto_swinging_ && std::abs(heading_error) > 0.5f &&
        (desired_rate > steering_.highest_rate || desired_rate < steering_.lowest_rate) &&
        std::abs(std::clamp(desired_rate, steering_.lowest_rate, steering_.highest_rate)) <
            0.5f * std::abs(tightest_rate)) {
        goto_swinging_ = true;
    }
    if (goto_swinging_ && std::abs(heading_error) < 0.2f) {
        goto_swinging_ = false;
    }

    const float commanded_rate =
        goto_swinging_
            ? tightest_rate
            : std::clamp(desired_rate, steering_.lowest_rate, steering_.highest_rate);
    graph_desired_parameter_ = std::clamp(
        ParameterForRate(commanded_rate), pmg_parameter_min_, pmg_parameter_max_);
}

bool ViewerApp::HandleGroundClick(const glm::vec3& ray_origin,
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

    if (!steering_.ready) {
        CalibrateSteering();  // one-time; a few seconds of offline streaming
    }
    goto_swinging_ = false;
    goto_active_ = steering_.ready;
    if (goto_active_) {
        playing_ = true;
        goto_status_ = "Walking to target.";
    }
    return true;
}

void ViewerApp::Update(float delta_seconds) {
    // Graph runtime: drive the RuntimeController and render its streamed pose.
    if (GraphRuntimeActive()) {
        if (playing_) {
            if (goto_active_ && steering_.ready) {
                UpdateGotoSteering(graph_controller_->CurrentPose());
            }
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

// --- ImGui UI ---------------------------------------------------------------
//
// Everything lives in one window: Workflow + Transport stay visible at the
// top (they are needed in every mode), the rest is tabbed. Seven floating
// windows buried the 3D view and each other.

void ViewerApp::BuildUi() {
    HandleShortcuts();

    ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(430.0f, 740.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("PMG Viewer");

    BuildWorkflowSection();
    ImGui::Separator();
    BuildTransportSection();
    ImGui::Separator();

    if (ImGui::BeginTabBar("##viewer_tabs")) {
        if (ImGui::BeginTabItem("Clips")) {
            BuildClipsSection();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Blend")) {
            BuildBlendSection();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Distance Grid")) {
            BuildDistanceGridSection();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Graph")) {
            BuildGraphSection();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("View")) {
            BuildViewSection();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
}

void ViewerApp::HandleShortcuts() {
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

void ViewerApp::StepFrame(int direction) {
    if (GraphRuntimeActive()) {
        return;  // graph runtime is controller-driven; stepping is for clip/blend
    }
    const float duration = ActiveReferenceDuration();
    if (duration <= kEpsilon) {
        return;
    }
    const int frame_count = (ParametricBlendActive() && !pmg_examples_.empty())
                                ? pmg_examples_.front().clip.NumFrames()
                                : clip_.NumFrames();
    const float fps = (clip_.NumFrames() > 0) ? clip_.frames_per_second : 30.0f;
    const float frame_seconds =
        frame_count > 1 ? duration / static_cast<float>(frame_count - 1) : 1.0f / fps;

    playing_ = false;
    // Fold the free-running play clock into one loop, step, then wrap. Clamping
    // here used to pin the clock at exactly `duration` once playback had run
    // past one loop; that displays as phase 0 (fmod) and made forward stepping
    // a dead end until something moved the clock back inside the loop.
    float time = std::fmod(current_time_seconds_, duration);
    if (time < 0.0f) {
        time += duration;
    }
    time += static_cast<float>(direction) * frame_seconds;
    time = std::fmod(time, duration);
    if (time < 0.0f) {
        time += duration;
    }
    current_time_seconds_ = time;
}

bool ViewerApp::ParameterSliderWithTicks(const char* label, float* value,
                                         float min_value, float max_value) {
    const float frame_width = ImGui::CalcItemWidth();
    const bool changed = ImGui::SliderFloat(label, value, min_value, max_value, "%.3f");

    // Tick marks at the example parameters along the slider frame's bottom
    // edge, so the user sees where real clips sit on the blend axis.
    const float span = max_value - min_value;
    if (span > kEpsilon && !pmg_examples_.empty()) {
        const ImVec2 rect_min = ImGui::GetItemRectMin();
        const float bottom = ImGui::GetItemRectMax().y;
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        for (const PmgExample& example : pmg_examples_) {
            const float alpha = std::clamp((example.parameter - min_value) / span, 0.0f, 1.0f);
            const float x = rect_min.x + alpha * frame_width;
            draw_list->AddLine(ImVec2(x, bottom - 4.0f), ImVec2(x, bottom + 2.0f),
                               IM_COL32(255, 255, 255, 170), 1.0f);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::TextDisabled("ticks = example clips (ctrl+click to type)");
            for (const PmgExample& example : pmg_examples_) {
                ImGui::Text("%.3f  %s", example.parameter, example.label.c_str());
            }
            ImGui::EndTooltip();
        }
    }
    return changed;
}

void ViewerApp::BuildWorkflowSection() {
    ImGui::TextWrapped("%s", status_message_.c_str());
    ImGui::Separator();

    // Numbered workflow so the order (clip -> blend space -> graph) is
    // discoverable. Each step reports done / locked / available.
    const bool clip_loaded = clip_.NumFrames() > 0;
    const bool has_examples = !pmg_examples_.empty();

    auto step = [](int number, const char* title, bool done, bool locked, const char* hint) {
        if (locked) {
            ImGui::TextDisabled("%d. %s  (%s)", number, title, hint);
        } else if (done) {
            ImGui::TextColored(ImVec4(0.40f, 0.80f, 0.45f, 1.0f), "%d. %s  [done]", number, title);
        } else {
            ImGui::Text("%d. %s  (%s)", number, title, hint);
        }
    };
    step(1, "Load a clip", clip_loaded, false, "pick one in Clips");
    step(2, "Add clips to blend space", has_examples, !clip_loaded, "needs a clip");
    step(3, "Build graph", graph_ready_, !pmg_space_ready_, "needs blend space");

    ImGui::Separator();
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

void ViewerApp::BuildTransportSection() {
    ImGui::Text("Mode: %s", ModeName(mode_));
    if (clip_.NumFrames() > 0) {
        ImGui::SameLine();
        ImGui::TextDisabled("| %d frames @ %.0f fps", clip_.NumFrames(), clip_.frames_per_second);
    }
    ImGui::Separator();

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
                                     {graph_desired_parameter_}, graph_frame_count_, graph_fps_);
        }
    }

    ImGui::SliderFloat("Speed", &playback_speed_, 0.1f, 3.0f, "%.2fx");

    // Phase scrub: clip & blend only (the graph's phase is owned by the
    // controller). Auto-pause while dragging so playback doesn't fight it.
    const float duration = ActiveReferenceDuration();
    if (time_mode) {
        float phase = NormalizedPhase(current_time_seconds_, duration);
        if (ImGui::SliderFloat("Phase", &phase, 0.0f, 1.0f, "%.3f")) {
            current_time_seconds_ = phase * duration;
        }
        if (ImGui::IsItemActive()) {
            playing_ = false;
        }
    } else if (graph_controller_.has_value()) {
        ImGui::Text("Phase %.3f  (graph-driven)", graph_controller_->CurrentPhase());
    }

    ImGui::TextDisabled("space play/pause   .   arrows step   .   R reset");
}

void ViewerApp::BuildClipsSection() {
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
    }

    ImGui::Separator();
    ImGui::TextWrapped("Add the loaded clip to the parametric blend space. The parameter is "
                       "its coordinate on the blend axis (e.g. turn rate or speed); poses are "
                       "interpolated between examples.");
    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputFloat("Blend parameter", &next_example_parameter_);
    ImGui::SameLine();
    if (ImGui::Button("Add to blend space")) {
        AddCurrentClipToSpace(next_example_parameter_);
    }
}

void ViewerApp::BuildViewSection() {
    ImGui::Checkbox("Follow motion centroid", &follow_centroid_);
    const glm::vec3 target = camera_.Target();
    ImGui::Text("Camera target: %.1f, %.1f, %.1f", target.x, target.y, target.z);
    ImGui::TextDisabled("Drag left mouse to orbit, scroll to zoom.");
    if (ImGui::Button("Reset camera")) {
        camera_.Reset();
        follow_centroid_ = true;
    }
    ImGui::Separator();
    ImGui::SliderFloat("Skeleton scale", &skeleton_scale_, 0.1f, 5.0f, "%.2fx");
    ImGui::SliderFloat("Display scale", &display_scale_, 1.0f, 40.0f, "%.1fx");
    ImGui::TextDisabled("Display scale magnifies the whole world, travel included.");
}

void ViewerApp::BuildBlendSection() {
    ImGui::TextWrapped("Live parametric blend over the added example clips "
                       "(pmg_core ParametricMotionSpace). Enter Parametric blend mode "
                       "above to scrub it.");
    ImGui::Text("Examples: %zu", pmg_examples_.size());
    for (const PmgExample& example : pmg_examples_) {
        ImGui::BulletText("%s  (param %.2f)", example.label.c_str(), example.parameter);
    }
    ImGui::Separator();

    if (ParametricBlendActive()) {
        ParameterSliderWithTicks("Blend parameter", &pmg_parameter_,
                                 pmg_parameter_min_, pmg_parameter_max_);
    } else {
        ImGui::TextDisabled("Enter Parametric blend mode to scrub the blend parameter.");
    }

    if (ImGui::Button("Clear blend space")) {
        pmg_examples_.clear();
        RebuildPmgSpace();
        status_message_ = "Cleared parametric space.";
    }
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

void ViewerApp::BuildDistanceGridSection() {
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
    ImGui::SliderFloat("TGOOD", &tgood_, 0.0f, 2.0f, "%.3f");
    ImGui::SliderFloat("TBAD", &tbad_, 0.0f, 2.0f, "%.3f");

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

// --- Graph runtime (PMG streaming) -----------------------------------------

void ViewerApp::BuildGraphRuntime() {
    graph_ready_ = false;
    graph_controller_.reset();
    // A new graph invalidates the achieved-turn-rate table and any active goto.
    steering_ = SteeringCalibration{};
    goto_active_ = false;
    goto_swinging_ = false;
    goto_status_.clear();

    if (!pmg_space_ready_ || pmg_examples_.empty()) {
        graph_status_ = "Add at least one clip to the parametric space first.";
        return;
    }
    if (tbad_ < tgood_) {
        tbad_ = tgood_;
    }

    try {
        graph_ = pmg::ParametricMotionGraph{};
        const int node = graph_.AddNode("viewer_node", pmg_space_);

        pmg::PmgBuilderConfig config;
        config.source_sample_count = 8;     // small for interactive build
        config.target_sample_count = 32;
        config.generated_frame_count = graph_frame_count_;
        config.generated_frames_per_second = graph_fps_;
        config.good_transition_threshold = tgood_;
        config.bad_transition_threshold = tbad_;

        pmg::PmgEdge edge = pmg::PmgBuilder::BuildEdge(
            pmg_skeleton_, node, node, pmg_space_, pmg_space_, config);
        if (edge.samples.empty()) {
            graph_status_ =
                "No valid self-edge (empty box). Raise TGOOD or use a looping clip.";
            return;
        }
        graph_.AddEdge(edge);

        // Paper-faithful runtime align: inject the point-cloud strategy. It holds
        // a reference to pmg_skeleton_ (a stable member), so build it first.
        graph_alignment_.emplace(pmg_skeleton_);
        graph_controller_.emplace(graph_, *graph_alignment_);
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

void ViewerApp::BuildGraphSection() {
    ImGui::TextWrapped(
        "PMG streaming (paper Sec 4-5): builds a self-edge over the parametric "
        "space and streams motion with point-cloud-aligned transitions.");
    ImGui::Text("Space examples: %zu", pmg_examples_.size());
    ImGui::TextDisabled("Edge GOOD/BAD use the shared TGOOD/TBAD in the Distance Grid tab.");
    if (ImGui::Button("Build Graph")) {
        BuildGraphRuntime();
    }
    ImGui::TextWrapped("%s", graph_status_.c_str());
    ImGui::Separator();

    if (graph_ready_ && graph_controller_.has_value()) {
        // While goto steering is active it owns the parameter; show it live
        // but don't let the slider fight the controller.
        ImGui::BeginDisabled(goto_active_);
        ParameterSliderWithTicks("Desired parameter", &graph_desired_parameter_,
                                 pmg_parameter_min_, pmg_parameter_max_);
        ImGui::EndDisabled();
        ImGui::Text("Node %d   phase %.2f   transitions %d   %s",
                    graph_controller_->CurrentNode(),
                    graph_controller_->CurrentPhase(),
                    graph_controller_->CompletedTransitions(),
                    graph_controller_->IsTransitioning() ? "[transitioning]" : "");
        ImGui::TextDisabled("Switch to Graph runtime mode above to run; "
                            "Restart graph is next to Play.");

        ImGui::Separator();
        ImGui::TextDisabled("Goto steering");
        ImGui::TextWrapped("Right-click the floor (in Graph runtime mode) to set a "
                           "walk target. First click calibrates achieved turn rates "
                           "(a few seconds).");
        if (ImGui::Button(steering_.ready ? "Recalibrate" : "Calibrate steering")) {
            CalibrateSteering();
        }
        if (steering_.ready) {
            ImGui::SameLine();
            ImGui::TextDisabled("rates %.2f .. %.2f rad/s",
                                steering_.lowest_rate, steering_.highest_rate);
        }
        if (goto_active_) {
            const pmg::Pose pose = graph_controller_->CurrentPose();
            const float dx = goto_target_.x - pose.root_position.x;
            const float dz = goto_target_.y - pose.root_position.z;
            ImGui::Text("Target (%.1f, %.1f)   distance %.2f%s",
                        goto_target_.x, goto_target_.y, std::sqrt(dx * dx + dz * dz),
                        goto_swinging_ ? "   [swinging long way]" : "");
            ImGui::SameLine();
            if (ImGui::Button("Clear target")) {
                goto_active_ = false;
                goto_swinging_ = false;
                goto_status_ = "Target cleared.";
            }
        }
        if (!goto_status_.empty()) {
            ImGui::TextWrapped("%s", goto_status_.c_str());
        }
    } else {
        ImGui::TextDisabled("No graph yet. Add clips to the space and Build Graph.");
    }
}

}  // namespace pmgviewer
