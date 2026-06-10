#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/MotionClip.h"
#include "pmg/MotionDistance.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/ParametricMotionSpace.h"
#include "pmg/PmgBuilder.h"
#include "pmg/Pose.h"
#include "pmg/RuntimeController.h"
#include "pmg/Skeleton.h"

#include "Camera.h"
#include "SkeletonRenderer.h"

namespace pmgviewer {

// How the viewer sources the pose it renders each frame.
//   ClipPlayback    - sample the single loaded BVH clip.
//   ParametricBlend - evaluate the live ParametricMotionSpace blend.
//   GraphRuntime    - stream motion from a ParametricMotionGraph via
//                     RuntimeController (self-edge transitions, paper Sec 4-5).
enum class ViewerPlaybackMode {
    ClipPlayback,
    ParametricBlend,
    GraphRuntime,
};

// Top-level viewer state: owns the loaded BVH motion, playback clock, the live
// parametric-blend control, and produces a RenderScene each frame.
//
// Purpose: drive the skeleton renderer from pmg_core data and expose ImGui
// panels (playback, clip picker, camera, parametric blend).
// Assumptions: world is Y-up; a clip's lowest point over its whole duration is
// rested on y = 0 so vertical dynamics are preserved.
class ViewerApp {
public:
    void Initialize();

    void Update(float delta_seconds);
    void BuildUi();

    const RenderScene& Scene() const { return scene_; }
    OrbitCamera& Camera() { return camera_; }

private:
    struct PmgExample {
        std::string label;
        float parameter = 0.0f;
        pmg::MotionClip clip;
    };

    void DiscoverBvhFiles();
    void LoadClip(int file_index);
    pmg::Pose CurrentPose() const;
    const pmg::Skeleton& ActiveSkeleton() const;
    void RebuildScene(const pmg::Pose& pose);

    void AddCurrentClipToSpace(float parameter);
    void RebuildPmgSpace();
    static float ComputeGroundOffset(const pmg::Skeleton& skeleton, const pmg::MotionClip& clip);
    float ActiveReferenceDuration() const;

    void BuildWorkflowPanel();
    void BuildTransportPanel();
    void BuildClipsPanel();
    void BuildViewPanel();
    void BuildBlendPanel();
    void BuildDistanceGridPanel();
    void BuildGraphPanel();

    void RecomputeHeatmap();
    void SaveHeatmapCsv();
    void BuildGraphRuntime();

    void HandleShortcuts();
    void StepFrame(int direction);  // +1 / -1 frame; clip & blend modes only
    void ResetPlayback();
    bool ParametricBlendActive() const;
    bool GraphRuntimeActive() const;
    static const char* ModeName(ViewerPlaybackMode mode);

    std::vector<std::filesystem::path> bvh_files_;
    int selected_file_index_ = -1;
    char clip_filter_[64] = "";  // case-insensitive substring filter for the clip list
    std::string status_message_ = "No clip loaded.";

    pmg::Skeleton skeleton_;
    pmg::MotionClip clip_;
    float ground_offset_ = 0.0f;

    bool playing_ = true;
    float playback_speed_ = 1.0f;
    float current_time_seconds_ = 0.0f;
    bool follow_centroid_ = true;
    float skeleton_scale_ = 1.0f;
    // Render-only display scale: BVH is loaded in native units (small), so the
    // viewer scales geometry up for display. The metric/core stay native; this
    // never touches distances/thresholds (replaces the old loader x10 bake).
    float display_scale_ = 10.0f;

    ViewerPlaybackMode mode_ = ViewerPlaybackMode::ClipPlayback;
    std::vector<PmgExample> pmg_examples_;
    pmg::Skeleton pmg_skeleton_;
    pmg::ParametricMotionSpace pmg_space_;
    bool pmg_space_ready_ = false;
    float pmg_ground_offset_ = 0.0f;
    float pmg_parameter_ = 0.0f;
    float pmg_parameter_min_ = 0.0f;
    float pmg_parameter_max_ = 1.0f;
    float next_example_parameter_ = 0.0f;

    // --- Distance Grid heatmap (transition visualization, paper §3.1 Fig 3) ---
    // Source clip = the currently loaded clip_; target chosen below. Grid is
    // cached and rendered live; only Recompute / param changes rebuild it.
    int heatmap_target_index_ = -1;
    pmg::MotionClip heatmap_target_clip_;
    pmg::DistanceGridConfig heatmap_config_{5, 2, 2, 0.70f, 0.95f, 0.05f, 0.30f, {}};
    // GOOD/BAD transition thresholds. One source of truth shared by the heatmap
    // classification and the graph edge build (native units; paper 0.5 / 0.7).
    float tgood_ = 0.5f;
    float tbad_ = 0.7f;
    pmg::DistanceGrid heatmap_grid_;
    bool heatmap_ready_ = false;
    float heatmap_min_distance_ = 0.0f;
    float heatmap_max_distance_ = 1.0f;
    int heatmap_min_source_index_ = -1;
    int heatmap_min_target_index_ = -1;
    int heatmap_selected_source_index_ = -1;
    int heatmap_selected_target_index_ = -1;
    std::string heatmap_status_ = "Pick a target clip, then Recompute.";

    // --- Graph runtime (PMG streaming, paper Sec 4-5) ---------------------------
    // Builds a self-edge over the parametric space and drives a RuntimeController.
    pmg::ParametricMotionGraph graph_;
    // Owns the alignment strategy injected into the controller; must outlive it,
    // so it is declared first (members destroy in reverse order).
    std::optional<pmg::PointCloudAlignment> graph_alignment_;
    std::optional<pmg::RuntimeController> graph_controller_;
    bool graph_ready_ = false;
    float graph_desired_parameter_ = 0.0f;
    int graph_frame_count_ = 48;
    float graph_fps_ = 30.0f;
    std::string graph_status_ = "Build a parametric space, then Build Graph.";

    RenderScene scene_;
    OrbitCamera camera_;
};

}  // namespace pmgviewer
