#pragma once

#include <string>
#include <vector>

namespace pmgviewer {

struct ViewerJointUiState {
    std::string name;
    int parent_index = -1;
    int channel_count = 0;
};

enum class ViewerPlaybackMode { ClipPlayback, ParametricBlend, GraphRuntime };

// UI-only snapshot. Values are copied; no mutable PMG object crosses this seam.
struct ViewerUiState {
    ViewerPlaybackMode playback_mode = ViewerPlaybackMode::ClipPlayback;
    bool playing = false;
    bool motion_space_ready = false;
    bool graph_runtime_ready = false;
    bool graph_runtime_active = false;
    bool path_preview_enabled = false;
    int path_preview_count = 5;
    float playback_speed = 1.0f;
    float phase = 0.0f;
    float skeleton_scale = 1.0f;
    float display_scale = 10.0f;
    int selected_clip_index = -1;
    int selected_spec_index = -1;
    std::string status_message;
    std::vector<std::string> clip_names;
    int loaded_frame_count = 0;
    float loaded_frames_per_second = 0.0f;
    std::vector<ViewerJointUiState> loaded_joints;
    std::string artifact_units;
    int motion_space_dimension = 1;
    bool has_motion_samples = false;
    std::vector<float> next_sample_parameter;
};

enum class ViewerUiCommandType {
    LoadClip, SelectClip, SetPlaybackMode, TogglePlayback, ResetPlayback,
    StepFrame, SetPlaybackSpeed, SetPhase, SetPathPreviewEnabled,
    SetPathPreviewCount, SetSkeletonScale, SetDisplayScale,
    SetScalarParameter, SetVectorParameter, RebuildMotionSpace,
    RecomputeHeatmap, SaveHeatmapCsv, BuildGraphRuntime, ResetGraphRuntime,
    SetDesiredRuntimeNode, SetDirectSteeringMode, SetGotoMode, SaveArtifact,
    LoadGraphArtifact, BuildArtifactFromSpec, SetMotionSpaceDimension,
    SetNextSampleParameter, AddMotionSample
};

struct ViewerUiCommand {
    ViewerUiCommandType type;
    int index = -1;
    float x = 0.0f;
    float y = 0.0f;
    std::string text;
    std::vector<float> values;
};

}  // namespace pmgviewer
