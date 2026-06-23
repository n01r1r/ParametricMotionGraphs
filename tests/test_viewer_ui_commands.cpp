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
        Command(pmgviewer::ViewerUiCommandType::SetShowJointNames, 1),
        Command(pmgviewer::ViewerUiCommandType::SetShowEndSitesSeparately, 1),
        Command(pmgviewer::ViewerUiCommandType::SetHideEndSites, 1),
        Command(pmgviewer::ViewerUiCommandType::SetDrawLocalJointAxes, 1),
        Command(pmgviewer::ViewerUiCommandType::SetMarkLikelyFootJoints, 1),
        Command(pmgviewer::ViewerUiCommandType::SetSkeletonPoseMode, 2),
    });

    const pmgviewer::ViewerUiState state = workspace.MakeUiState();
    assert(state.playback_mode == pmgviewer::ViewerPlaybackMode::GraphRuntime);
    assert(state.playback_speed == 3.0f);
    assert(state.phase == 1.0f);
    assert(state.path_preview_enabled);
    assert(state.path_preview_count == 12);
    assert(state.skeleton_scale == 0.1f);
    assert(state.display_scale == 40.0f);
    assert(state.show_joint_names);
    assert(!state.show_end_sites_separately);
    assert(state.hide_end_sites);
    assert(state.draw_local_joint_axes);
    assert(state.mark_likely_foot_joints);
    assert(state.skeleton_pose_mode == pmgviewer::ViewerSkeletonPoseMode::Rest);
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

void TestInputCommandsUpdateSnapshot() {
    pmgviewer::PmgViewerWorkspace workspace;
    workspace.ApplyUiCommand(
        Command(pmgviewer::ViewerUiCommandType::SetMotionSpaceDimension, 3));
    pmgviewer::ViewerUiCommand parameter =
        Command(pmgviewer::ViewerUiCommandType::SetNextSampleParameter);
    parameter.values = {1.0f, 2.0f, 3.0f};
    workspace.ApplyUiCommand(parameter);

    const pmgviewer::ViewerUiState state = workspace.MakeUiState();
    assert(state.motion_space_dimension == 3);
    assert(state.next_sample_parameter == parameter.values);
}

void TestMotionSpaceViewCommandsUpdateSnapshot() {
    pmgviewer::PmgViewerWorkspace workspace;
    workspace.ApplyUiCommands({
        Command(pmgviewer::ViewerUiCommandType::SetMotionSpaceDimension, 3),
        Command(pmgviewer::ViewerUiCommandType::SetMotionSpaceViewAxis, 99),
        Command(pmgviewer::ViewerUiCommandType::SetMotionSpacePreviewInPlace, 1),
    });

    const pmgviewer::ViewerUiState state = workspace.MakeUiState();
    assert(state.motion_space_view_axis == 2);
    assert(state.motion_space_preview_in_place);
}

void TestInvalidMotionSampleCommandRejected() {
    pmgviewer::PmgViewerWorkspace workspace;
    pmgviewer::ViewerUiCommand command =
        Command(pmgviewer::ViewerUiCommandType::SetMotionSampleParameter, 0);
    command.values = {1.0f};

    bool threw = false;
    try {
        workspace.ApplyUiCommand(command);
    } catch (const std::out_of_range&) {
        threw = true;
    }
    assert(threw);
}

}  // namespace

int main() {
    TestRepresentativeCommandsUpdateUiState();
    TestTogglePlaybackUpdatesUiState();
    TestInvalidPlaybackModeRejected();
    TestInputCommandsUpdateSnapshot();
    TestMotionSpaceViewCommandsUpdateSnapshot();
    TestInvalidMotionSampleCommandRejected();
    return 0;
}
