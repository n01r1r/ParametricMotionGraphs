#include "PmgViewerWorkspace.h"

#include <cassert>
#include <stdexcept>
#include <vector>

namespace {

pmgviewer::ViewerUiCommand Command(
    pmgviewer::ViewerUiCommandType type, int index = -1, float x = 0.0f) {
    pmgviewer::ViewerUiCommand command{};
    command.type = type;
    command.index = index;
    command.x = x;
    return command;
}

void TestRepresentativeCommandsUpdateUiState() {
    pmgviewer::PmgViewerWorkspace workspace;

    workspace.ApplyUiCommands({
        Command(pmgviewer::ViewerUiCommandType::SetPlaybackMode, 2),
        Command(pmgviewer::ViewerUiCommandType::SetPhase, -1, 2.0f),
        Command(pmgviewer::ViewerUiCommandType::SetPlaybackSpeed, -1, 9.0f),
        Command(pmgviewer::ViewerUiCommandType::SetPathPreviewEnabled, 1),
        Command(pmgviewer::ViewerUiCommandType::SetPathPreviewCount, 99),
        Command(pmgviewer::ViewerUiCommandType::SetSkeletonScale, -1, 0.05f),
        Command(pmgviewer::ViewerUiCommandType::SetDisplayScale, -1, 99.0f),
    });

    const pmgviewer::ViewerUiState state = workspace.MakeUiState();
    assert(state.playback_mode == pmgviewer::ViewerPlaybackMode::GraphRuntime);
    assert(state.playback_speed == 3.0f);
    assert(state.phase == 1.0f);
    assert(state.path_preview_enabled);
    assert(state.path_preview_count == 12);
    assert(state.skeleton_scale == 0.1f);
    assert(state.display_scale == 40.0f);
}

void TestTogglePlaybackUpdatesUiState() {
    pmgviewer::PmgViewerWorkspace workspace;
    const bool before = workspace.MakeUiState().playing;
    workspace.ApplyUiCommand(Command(pmgviewer::ViewerUiCommandType::TogglePlayback));
    assert(workspace.MakeUiState().playing != before);
}

void TestInvalidPlaybackModeRejected() {
    pmgviewer::PmgViewerWorkspace workspace;

    bool threw = false;
    try {
        workspace.ApplyUiCommand(
            Command(pmgviewer::ViewerUiCommandType::SetPlaybackMode, 99));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    assert(threw);
}

}  // namespace

int main() {
    TestRepresentativeCommandsUpdateUiState();
    TestTogglePlaybackUpdatesUiState();
    TestInvalidPlaybackModeRejected();
    return 0;
}
