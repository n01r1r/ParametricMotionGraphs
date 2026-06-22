#include "ImGuiViewerUi.h"

#include <imgui.h>

namespace pmgviewer {
namespace {
void Push(std::vector<ViewerUiCommand>& commands, ViewerUiCommandType type,
          int index = -1, float x = 0.0f) {
    commands.push_back({type, index, x});
}
}  // namespace

std::vector<ViewerUiCommand> ImGuiViewerUi::Build(
    const ViewerUiState& state, const std::function<void()>& build_legacy_tabs) {
    std::vector<ViewerUiCommand> commands;
    ImGui::SetNextWindowPos(ImVec2(12.0f, 12.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(520.0f, 780.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("PMG Viewer");

    ImGui::TextWrapped("%s", state.status_message.c_str());
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

    ImGui::Separator();
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
    }

    build_legacy_tabs();  // ponytail: migrate remaining complex tabs incrementally.
    ImGui::Separator();
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
