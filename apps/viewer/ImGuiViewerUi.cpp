#include "ImGuiViewerUi.h"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <string>

namespace pmgviewer {
namespace {
constexpr float kEpsilon = 1.0e-6f;
constexpr float kParameterCanvasHeight = 88.0f;
constexpr float kPhaseRowHeight = 26.0f;

void Push(std::vector<ViewerUiCommand>& commands, ViewerUiCommandType type,
          int index = -1, float x = 0.0f) {
    commands.push_back({type, index, x});
}

void BuildInputs(const ViewerUiState& state, char* clip_filter,
                 std::vector<ViewerUiCommand>& commands) {
    ImGui::Text("%zu BVH files", state.clip_names.size());
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputTextWithHint("##clip_filter", "filter by name...", clip_filter, 64);

    std::string needle = clip_filter;
    std::transform(needle.begin(), needle.end(), needle.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ImGui::BeginListBox("##clips", ImVec2(-1.0f, 220.0f))) {
        for (int i = 0; i < static_cast<int>(state.clip_names.size()); ++i) {
            std::string hay = state.clip_names[i];
            std::transform(hay.begin(), hay.end(), hay.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (!needle.empty() && hay.find(needle) == std::string::npos) continue;
            if (ImGui::Selectable(state.clip_names[i].c_str(),
                                  i == state.selected_clip_index))
                Push(commands, ViewerUiCommandType::LoadClip, i);
        }
        ImGui::EndListBox();
    }

    if (state.loaded_frame_count > 0) {
        ImGui::TextDisabled("Loaded: %d frames @ %.0f fps, %zu joints",
                            state.loaded_frame_count, state.loaded_frames_per_second,
                            state.loaded_joints.size());
        if (ImGui::CollapsingHeader("BVH skeleton / channel diagnostics")) {
            bool show_joint_names = state.show_joint_names;
            if (ImGui::Checkbox("Show joint names", &show_joint_names)) {
                Push(commands, ViewerUiCommandType::SetShowJointNames,
                     show_joint_names ? 1 : 0);
            }
            bool show_end_sites = state.show_end_sites_separately;
            if (ImGui::Checkbox("Show end sites separately", &show_end_sites)) {
                Push(commands, ViewerUiCommandType::SetShowEndSitesSeparately,
                     show_end_sites ? 1 : 0);
            }
            bool hide_end_sites = state.hide_end_sites;
            if (ImGui::Checkbox("Hide end sites", &hide_end_sites)) {
                Push(commands, ViewerUiCommandType::SetHideEndSites,
                     hide_end_sites ? 1 : 0);
            }
            bool draw_axes = state.draw_local_joint_axes;
            if (ImGui::Checkbox("Draw local joint axes", &draw_axes)) {
                Push(commands, ViewerUiCommandType::SetDrawLocalJointAxes,
                     draw_axes ? 1 : 0);
            }
            bool mark_feet = state.mark_likely_foot_joints;
            if (ImGui::Checkbox("Mark likely foot joints", &mark_feet)) {
                Push(commands, ViewerUiCommandType::SetMarkLikelyFootJoints,
                     mark_feet ? 1 : 0);
            }
            int pose_mode = static_cast<int>(state.skeleton_pose_mode);
            ImGui::TextDisabled("Pose view");
            ImGui::RadioButton("Current frame", &pose_mode,
                               static_cast<int>(ViewerSkeletonPoseMode::Current));
            ImGui::SameLine();
            ImGui::RadioButton("Frame 0", &pose_mode,
                               static_cast<int>(ViewerSkeletonPoseMode::Frame0));
            ImGui::SameLine();
            ImGui::RadioButton("Rest pose", &pose_mode,
                               static_cast<int>(ViewerSkeletonPoseMode::Rest));
            if (pose_mode != static_cast<int>(state.skeleton_pose_mode)) {
                Push(commands, ViewerUiCommandType::SetSkeletonPoseMode, pose_mode);
            }
            if (state.show_joint_names) {
                ImGui::Separator();
                for (const ViewerJointUiState& joint : state.loaded_joints) {
                    const char* kind = joint.channel_count == 0 ? "end" : "joint";
                    ImGui::BulletText("%s (%s) parent=%d channels=%d",
                                      joint.name.c_str(), kind, joint.parent_index,
                                      joint.channel_count);
                }
            } else {
                for (std::size_t index = 0; index < state.loaded_joints.size(); ++index) {
                    const ViewerJointUiState& joint = state.loaded_joints[index];
                    ImGui::BulletText("joint %zu parent=%d channels=%d", index,
                                      joint.parent_index, joint.channel_count);
                }
            }
            ImGui::TextDisabled("Units: %s", state.artifact_units.c_str());
        }
    }

    ImGui::SeparatorText("Motion samples");
    int dimension = state.motion_space_dimension;
    ImGui::BeginDisabled(state.has_motion_samples);
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::InputInt("Parameter dimension", &dimension)) {
        dimension = std::clamp(dimension, 1, 4);
        Push(commands, ViewerUiCommandType::SetMotionSpaceDimension, dimension);
    }
    ImGui::EndDisabled();

    std::vector<float> parameter = state.next_sample_parameter;
    parameter.resize(static_cast<std::size_t>(std::max(1, dimension)), 0.0f);
    ImGui::SetNextItemWidth(220.0f);
    if (ImGui::InputScalarN("Blend parameter", ImGuiDataType_Float,
                            parameter.data(), dimension)) {
        ViewerUiCommand command{ViewerUiCommandType::SetNextSampleParameter};
        command.values = std::move(parameter);
        commands.push_back(std::move(command));
    }
    ImGui::SameLine();
    if (ImGui::Button("Add motion sample"))
        Push(commands, ViewerUiCommandType::AddMotionSample);
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

bool ParameterSliderWithTicks(const ViewerUiState& state, const char* label, int axis,
                              std::vector<ViewerUiCommand>& commands) {
    if (axis >= static_cast<int>(state.motion_space_parameter.size()) ||
        axis >= static_cast<int>(state.motion_space_parameter_min.size()) ||
        axis >= static_cast<int>(state.motion_space_parameter_max.size())) {
        return false;
    }
    float value = state.motion_space_parameter[axis];
    const float min_value = state.motion_space_parameter_min[axis];
    const float max_value = state.motion_space_parameter_max[axis];
    const float frame_width = ImGui::CalcItemWidth();
    const bool changed = ImGui::SliderFloat(label, &value, min_value, max_value, "%.3f");
    if (changed) {
        Push(commands, ViewerUiCommandType::SetScalarParameter, axis, value);
    }

    const float span = max_value - min_value;
    if (span > kEpsilon && !state.motion_samples.empty()) {
        const ImVec2 rect_min = ImGui::GetItemRectMin();
        const float bottom = ImGui::GetItemRectMax().y;
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        for (const ViewerMotionSampleUiState& sample : state.motion_samples) {
            if (axis >= static_cast<int>(sample.parameter.size())) {
                continue;
            }
            const float alpha =
                std::clamp((sample.parameter[axis] - min_value) / span, 0.0f, 1.0f);
            const float x = rect_min.x + alpha * frame_width;
            draw_list->AddLine(ImVec2(x, bottom - 4.0f), ImVec2(x, bottom + 2.0f),
                               IM_COL32(255, 255, 255, 170), 1.0f);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::TextDisabled("ticks = example clips (ctrl+click to type)");
            for (const ViewerMotionSampleUiState& sample : state.motion_samples) {
                const float tick = axis < static_cast<int>(sample.parameter.size())
                                       ? sample.parameter[axis]
                                       : 0.0f;
                ImGui::Text("%.3f  %s", tick, sample.label.c_str());
            }
            ImGui::EndTooltip();
        }
    }
    return changed;
}

void DrawParameterSpace(const ViewerUiState& state) {
    if (!state.motion_space_ready || state.motion_samples.empty()) {
        return;
    }
    const int axis = std::clamp(
        state.motion_space_view_axis, 0, std::max(0, state.motion_space_dimension - 1));
    if (axis >= static_cast<int>(state.motion_space_parameter_min.size()) ||
        axis >= static_cast<int>(state.motion_space_parameter.size())) {
        return;
    }
    const float axis_min = state.motion_space_parameter_min[axis];
    const float axis_max = state.motion_space_parameter_max[axis];
    const float query_value = state.motion_space_parameter[axis];

    if (!state.motion_space_weight_error.empty()) {
        ImGui::TextWrapped("Weight evaluation failed: %s",
                           state.motion_space_weight_error.c_str());
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

    for (std::size_t index = 0; index < state.motion_samples.size(); ++index) {
        const ViewerMotionSampleUiState& sample = state.motion_samples[index];
        const float coord = axis < static_cast<int>(sample.parameter.size())
                                ? sample.parameter[axis] : 0.0f;
        const float x = ParameterToCanvasX(
            coord, axis_min, axis_max, axis_left, axis_width);
        const float weight = index < state.motion_space_local_blend_weights.size()
                                 ? state.motion_space_local_blend_weights[index]
                                 : 0.0f;
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

    for (std::size_t index = 0; index < state.motion_samples.size(); ++index) {
        const float weight = index < state.motion_space_local_blend_weights.size()
                                 ? state.motion_space_local_blend_weights[index]
                                 : 0.0f;
        ImGui::Text("%s", state.motion_samples[index].label.c_str());
        ImGui::SameLine(265.0f);
        ImGui::ProgressBar(weight, ImVec2(-1.0f, 0.0f),
                           (std::to_string(static_cast<int>(std::lround(weight * 100.0f))) +
                            "%").c_str());
    }

    if (state.motion_space_has_parameter_calibration) {
        if (state.motion_space_calibration_samples_per_axis > 0) {
            ImGui::TextDisabled(
                "Parameter accuracy: calibrated inversion (%d metrics, %d "
                "samples, %d/axis)",
                state.motion_space_calibration_metric_count,
                state.motion_space_calibration_sample_count,
                state.motion_space_calibration_samples_per_axis);
        } else {
            ImGui::TextDisabled(
                "Parameter accuracy: calibrated inversion (%d metrics, %d "
                "imported samples)",
                state.motion_space_calibration_metric_count,
                state.motion_space_calibration_sample_count);
        }
    } else {
        ImGui::TextDisabled("Parameter accuracy: local Shepard interpolation");
    }
}

void DrawSteeringPlots(const ViewerUiState& state) {
    if (state.motion_space_turn_rate_curve.size() < 2) {
        return;
    }
    const int axis = std::clamp(
        state.motion_space_view_axis, 0, std::max(0, state.motion_space_dimension - 1));
    const float axis_min = axis < static_cast<int>(state.motion_space_parameter_min.size())
                               ? state.motion_space_parameter_min[axis] : 0.0f;
    const float axis_max = axis < static_cast<int>(state.motion_space_parameter_max.size())
                               ? state.motion_space_parameter_max[axis] : 1.0f;
    const float axis_val = axis < static_cast<int>(state.motion_space_parameter.size())
                               ? state.motion_space_parameter[axis] : 0.0f;
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
                     state.motion_space_turn_rate_curve);
    plot_with_marker("##steering_travel_speed",
                     "Travel speed vs parameter (units/s)",
                     state.motion_space_travel_speed_curve);

    if (state.motion_space_current_blend_metrics_valid) {
        ImGui::TextDisabled("current blend: %.3f rad/s, %.3f units/s",
                            state.motion_space_current_blend_turn_rate,
                            state.motion_space_current_blend_travel_speed);
    }
}

void DrawPhaseTimeline(const ViewerUiState& state) {
    if (!state.motion_space_ready || state.motion_samples.empty()) {
        return;
    }

    ImGui::Separator();
    ImGui::TextDisabled("Canonical phase -> registered example phase / foot contact");
    const float label_width = 210.0f;
    const float row_width =
        std::max(120.0f, ImGui::GetContentRegionAvail().x - label_width);
    const ImVec2 start = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    for (std::size_t index = 0; index < state.motion_samples.size(); ++index) {
        const ViewerMotionSampleUiState& sample = state.motion_samples[index];
        const float row_top = start.y + static_cast<float>(index) * kPhaseRowHeight;
        draw_list->AddText(
            ImVec2(start.x, row_top + 2.0f),
            IM_COL32(210, 212, 218, 255),
            sample.label.c_str());
        const float timeline_left = start.x + label_width;
        const float timeline_y = row_top + 11.0f;
        draw_list->AddLine(
            ImVec2(timeline_left, timeline_y),
            ImVec2(timeline_left + row_width, timeline_y),
            IM_COL32(100, 104, 112, 255), 2.0f);

        for (const ViewerMotionSampleContactUiState& contact : sample.contacts) {
            const float x0 = timeline_left + contact.strike_phase * row_width;
            const float x1 = timeline_left + contact.lift_phase * row_width;
            draw_list->AddRectFilled(
                ImVec2(x0, timeline_y - 5.0f),
                ImVec2(std::max(x0 + 1.0f, x1), timeline_y + 5.0f),
                IM_COL32(72, 165, 105, 150), 2.0f);
        }

        DrawPhaseMarker(
            draw_list, sample.registered_phase, timeline_left, timeline_y - 6.0f,
            row_width, 12.0f, IM_COL32(80, 205, 255, 255));

        for (const float anchor : sample.registration_anchor_phases) {
            const float x = timeline_left + anchor * row_width;
            draw_list->AddCircleFilled(
                ImVec2(x, timeline_y), 2.5f,
                IM_COL32(240, 185, 70, 255));
        }
    }

    const float total_height =
        static_cast<float>(state.motion_samples.size()) * kPhaseRowHeight;
    ImGui::InvisibleButton(
        "##phase_timeline",
        ImVec2(label_width + row_width, total_height));
    ImGui::TextDisabled(
        "blue = current phase   green = detected contact   gold = registration anchor");
    ImGui::Text("Canonical phase %.3f", state.phase);
}

void BuildMotionSpace(const ViewerUiState& state,
                      std::vector<ViewerUiCommand>& commands) {
    ImGui::Text("Space: %s   dimension: %d   samples: %zu",
                state.motion_space_ready ? state.motion_space_name.c_str() : "(none)",
                state.motion_space_ready ? state.motion_space_dimension : 0,
                state.motion_samples.size());
    ImGui::TextDisabled(
        "Graph NODE = this parametric motion space: a blend of its sample clips.");
    ImGui::TextDisabled(
        "Graph EDGES = sampled transitions between nodes (see the Graph tab).");

    if (!state.motion_space_blend_error.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.55f, 0.25f, 1.0f));
        ImGui::TextWrapped("Blend space rejected: %s",
                           state.motion_space_blend_error.c_str());
        ImGui::PopStyleColor();
    }

    if (state.motion_space_ready) {
        ImGui::SeparatorText("Evaluation parameter");
        const int dim = std::max(1, state.motion_space_dimension);
        for (int axis = 0; axis < dim; ++axis) {
            ImGui::PushID(axis);
            const std::string label =
                "Blend parameter [" + std::to_string(axis) + "]";
            ParameterSliderWithTicks(state, label.c_str(), axis, commands);
            ImGui::PopID();
        }
        if (dim > 1) {
            int view_axis = state.motion_space_view_axis;
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputInt("View axis (1-D plots)", &view_axis)) {
                Push(commands, ViewerUiCommandType::SetMotionSpaceViewAxis,
                     std::clamp(view_axis, 0, dim - 1));
            }
        }
        DrawParameterSpace(state);

        bool preview_in_place = state.motion_space_preview_in_place;
        if (ImGui::Checkbox("In-place preview", &preview_in_place)) {
            Push(commands, ViewerUiCommandType::SetMotionSpacePreviewInPlace,
                 preview_in_place ? 1 : 0);
        }
        ImGui::SameLine();
        ImGui::TextDisabled(preview_in_place
                                ? "(root locked: cycle in place)"
                                : "(trajectory: integrated root path)");

        bool foot_lock = state.motion_space_foot_lock;
        if (ImGui::Checkbox("Foot-lock (IK)", &foot_lock)) {
            Push(commands, ViewerUiCommandType::SetMotionSpaceFootLock,
                 foot_lock ? 1 : 0);
        }
        ImGui::SameLine();
        ImGui::TextDisabled(foot_lock
                                ? "(stance feet pinned: skate removed)"
                                : "(raw blend: feet may slide)");

        DrawSteeringPlots(state);
        DrawPhaseTimeline(state);

        if (state.playback_mode != ViewerPlaybackMode::ParametricBlend) {
            ImGui::TextDisabled(
                "Switch mode to Parametric blend to render this evaluation parameter.");
        }
    } else {
        ImGui::TextDisabled("Add motion samples from Inputs.");
    }

    if (!state.motion_samples.empty()) {
        ImGui::SeparatorText("Samples (edit / remove)");
        for (int i = 0; i < static_cast<int>(state.motion_samples.size()); ++i) {
            ImGui::PushID(i);
            std::vector<float> parameter = state.motion_samples[i].parameter;
            parameter.resize(static_cast<std::size_t>(std::max(1, state.motion_space_dimension)),
                             0.0f);
            ImGui::SetNextItemWidth(60.0f * state.motion_space_dimension);
            if (ImGui::InputScalarN("##param", ImGuiDataType_Float, parameter.data(),
                                    state.motion_space_dimension, nullptr, nullptr,
                                    "%.3f", ImGuiInputTextFlags_EnterReturnsTrue)) {
                ViewerUiCommand command{ViewerUiCommandType::SetMotionSampleParameter};
                command.index = i;
                command.values = std::move(parameter);
                commands.push_back(std::move(command));
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("x")) {
                Push(commands, ViewerUiCommandType::RemoveMotionSample, i);
            }
            ImGui::SameLine();
            ImGui::TextUnformatted(state.motion_samples[i].label.c_str());
            ImGui::PopID();
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Clear motion space")) {
        Push(commands, ViewerUiCommandType::ClearMotionSpace);
    }
}
}  // namespace

std::vector<ViewerUiCommand> ImGuiViewerUi::Build(
    const ViewerUiState& state,
    const std::function<void()>& build_legacy_tabs) {
    std::vector<ViewerUiCommand> commands;
    ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520.0f, 780.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("PMG Viewer");

    ImGui::TextWrapped("%s", state.status_message.c_str());
    ImGui::SeparatorText("Mode");
    int mode = static_cast<int>(state.playback_mode);
    ImGui::RadioButton("Clip playback", &mode, 0);
    ImGui::BeginDisabled(!state.motion_space_ready);
    ImGui::RadioButton("Parametric blend", &mode, 1);
    ImGui::EndDisabled();
    ImGui::BeginDisabled(!state.graph_runtime_ready);
    ImGui::RadioButton("Graph runtime", &mode, 2);
    ImGui::EndDisabled();
    if (mode != static_cast<int>(state.playback_mode)) {
        Push(commands, ViewerUiCommandType::SetPlaybackMode, mode);
    }

    ImGui::SeparatorText("Transport");
    if (ImGui::Button(state.playing ? "Pause" : "Play "))
        Push(commands, ViewerUiCommandType::TogglePlayback);
    ImGui::SameLine();
    ImGui::BeginDisabled(state.graph_runtime_active);
    if (ImGui::Button("< Frame")) Push(commands, ViewerUiCommandType::StepFrame, -1);
    ImGui::SameLine();
    if (ImGui::Button("Frame >")) Push(commands, ViewerUiCommandType::StepFrame, 1);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Reset")) Push(commands, ViewerUiCommandType::ResetPlayback);

    float speed = state.playback_speed;
    if (ImGui::SliderFloat("Speed", &speed, 0.1f, 3.0f, "%.2fx"))
        Push(commands, ViewerUiCommandType::SetPlaybackSpeed, -1, speed);
    if (!state.graph_runtime_active) {
        float phase = state.phase;
        if (ImGui::SliderFloat("Phase", &phase, 0.0f, 1.0f, "%.3f"))
            Push(commands, ViewerUiCommandType::SetPhase, -1, phase);

        // Path preview: ghost skeletons across the active clip (clip & blend modes).
        bool path_preview = state.path_preview_enabled;
        if (ImGui::Checkbox("Path preview", &path_preview))
            Push(commands, ViewerUiCommandType::SetPathPreviewEnabled,
                 path_preview ? 1 : 0);
        if (state.path_preview_enabled) {
            ImGui::SameLine();
            int ghosts = state.path_preview_count;
            ImGui::SetNextItemWidth(160.0f);
            if (ImGui::SliderInt("Ghosts", &ghosts, 2, 12))
                Push(commands, ViewerUiCommandType::SetPathPreviewCount, ghosts);
            ImGui::SameLine();
            ImGui::TextDisabled("(translucent trail, dim=start -> bright=end)");
        }

        // Parameter-sweep overlay: how the travel path changes across the blend
        // axis (blend mode only).
        if (state.motion_space_ready) {
            bool sweep_paths = state.param_sweep_paths_enabled;
            if (ImGui::Checkbox("Param-sweep paths", &sweep_paths))
                Push(commands, ViewerUiCommandType::SetParamSweepPaths,
                     sweep_paths ? 1 : 0);
            ImGui::SameLine();
            ImGui::TextDisabled("(blue=slow -> red=fast)");
        }
    }

    if (ImGui::BeginTabBar("##viewer_tabs")) {
        if (ImGui::BeginTabItem("Inputs")) {
            BuildInputs(state, clip_filter_, commands);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Motion Space")) {
            BuildMotionSpace(state, commands);
            ImGui::EndTabItem();
        }
        build_legacy_tabs();  // ponytail: migrate remaining tabs incrementally.
        ImGui::EndTabBar();
    }
    ImGui::SeparatorText("Display");
    float skeleton_scale = state.skeleton_scale;
    if (ImGui::SliderFloat("Skeleton scale", &skeleton_scale, 0.1f, 5.0f, "%.2fx"))
        Push(commands, ViewerUiCommandType::SetSkeletonScale, -1, skeleton_scale);
    float display_scale = state.display_scale;
    if (ImGui::SliderFloat("Display scale", &display_scale, 1.0f, 40.0f, "%.1fx"))
        Push(commands, ViewerUiCommandType::SetDisplayScale, -1, display_scale);
    ImGui::End();
    return commands;
}

}  // namespace pmgviewer
