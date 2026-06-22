#pragma once

#include <string>
#include <vector>

namespace pmgviewer {

struct ViewerJointUiState {
    std::string name;
    int parent_index = -1;
    int channel_count = 0;
};

struct ViewerMotionSampleContactUiState {
    float strike_phase = 0.0f;
    float lift_phase = 0.0f;
};

struct ViewerMotionSampleUiState {
    std::string label;
    std::vector<float> parameter;
    std::vector<ViewerMotionSampleContactUiState> contacts;
    float registered_phase = 0.0f;
    std::vector<float> registration_anchor_phases;
};

enum class ViewerPlaybackMode { ClipPlayback, ParametricBlend, GraphRuntime };

enum class ViewerSkeletonPoseMode { Current, Frame0, Rest };

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
    std::string motion_space_name;
    std::string motion_space_blend_error;
    std::vector<float> motion_space_parameter;
    std::vector<float> motion_space_parameter_min;
    std::vector<float> motion_space_parameter_max;
    int motion_space_view_axis = 0;
    bool motion_space_preview_in_place = false;
    std::string motion_space_weight_error;
    std::vector<float> motion_space_local_blend_weights;
    std::vector<float> motion_space_turn_rate_curve;
    std::vector<float> motion_space_travel_speed_curve;
    bool motion_space_current_blend_metrics_valid = false;
    float motion_space_current_blend_turn_rate = 0.0f;
    float motion_space_current_blend_travel_speed = 0.0f;
    bool motion_space_has_parameter_calibration = false;
    int motion_space_calibration_metric_count = 0;
    int motion_space_calibration_sample_count = 0;
    int motion_space_calibration_samples_per_axis = 0;
    std::vector<ViewerMotionSampleUiState> motion_samples;
    bool show_joint_names = false;
    bool show_end_sites_separately = false;
    bool hide_end_sites = false;
    bool draw_local_joint_axes = false;
    bool mark_likely_foot_joints = false;
    ViewerSkeletonPoseMode skeleton_pose_mode = ViewerSkeletonPoseMode::Current;
};

enum class ViewerUiCommandType {
    LoadClip, SelectClip, SetPlaybackMode, TogglePlayback, ResetPlayback,
    StepFrame, SetPlaybackSpeed, SetPhase, SetPathPreviewEnabled,
    SetPathPreviewCount, SetSkeletonScale, SetDisplayScale,
    SetScalarParameter, SetVectorParameter, RebuildMotionSpace,
    RecomputeHeatmap, SaveHeatmapCsv, BuildGraphRuntime, ResetGraphRuntime,
    SetDesiredRuntimeNode, SetDirectSteeringMode, SetGotoMode, SaveArtifact,
    LoadGraphArtifact, BuildArtifactFromSpec, SetMotionSpaceDimension,
    SetNextSampleParameter, AddMotionSample, SetMotionSpaceViewAxis,
    SetMotionSpacePreviewInPlace, SetMotionSampleParameter,
    RemoveMotionSample, ClearMotionSpace, SetShowJointNames,
    SetShowEndSitesSeparately, SetHideEndSites, SetDrawLocalJointAxes,
    SetMarkLikelyFootJoints, SetSkeletonPoseMode
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
