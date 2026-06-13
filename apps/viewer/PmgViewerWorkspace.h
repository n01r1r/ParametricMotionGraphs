#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "pmg/AlignmentStrategy.h"
#include "pmg/BvhLoader.h"
#include "pmg/ContactDetection.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/GraphIo.h"
#include "pmg/MotionClip.h"
#include "pmg/MotionDistance.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/ParametricMotionSpace.h"
#include "pmg/PmgBuilder.h"
#include "pmg/Pose.h"
#include "pmg/RuntimeController.h"
#include "pmg/Skeleton.h"

#include "GraphAuthoringModel.h"
#include "ViewerWorkspace.h"

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

// PMG Adapter: owns loaded BVH motion, playback, graph diagnostics, and the
// conversion from pmg_core data into an algorithm-neutral RenderScene.
//
// Purpose: drive the viewer workspace from pmg_core data and expose one
// tabbed ImGui control window (workflow + transport on top; clips, blend,
// distance grid, graph, and PMG display settings as tabs).
// Assumptions: world is Y-up; a clip's lowest point over its whole duration is
// rested on y = 0 so vertical dynamics are preserved.
class PmgViewerWorkspace final : public ViewerWorkspace {
public:
    void Initialize(const std::string& artifact_path = {}) override;

    void Update(float delta_seconds) override;
    void BuildUi() override;

    // Ray pick (display-space, e.g. from a right-click unprojected by main):
    // intersects the ground plane and, in graph runtime, places the goto
    // target there. Returns true when the click was consumed.
    bool HandleGroundClick(
        const glm::vec3& ray_origin, const glm::vec3& ray_direction) override;

    const RenderScene& Scene() const override { return scene_; }

private:
    enum class GraphOrigin {
        None,
        LoadedArtifact,
        SpecBuild,
        QuickSandbox,
        AuthoredSandbox,
    };

    struct PmgExample {
        std::string label;
        pmg::ParameterVector parameter;  // one value per parameter axis
        pmg::MotionClip clip;
        std::vector<pmg::ContactInterval> contact_intervals;
    };

    void DiscoverBvhFiles();
    void LoadClip(int file_index);
    pmg::Pose CurrentPose() const;
    const pmg::Skeleton& ActiveSkeleton() const;
    void RebuildScene(const pmg::Pose& pose);

    void AddCurrentClipToSpace(const pmg::ParameterVector& parameter);
    void RebuildPmgSpace();
    // Resize the live parameter vectors (current/next) to pmg_dimension_ and
    // clamp the view axis. Call after the dimension changes.
    void ResizeParameterVectors();
    // (Re)generate the cached parametric preview clip for the current blend
    // parameter via ParametricMotionSpace::GenerateClip (paper root-delta path).
    void RegeneratePreviewClip();
    // Sample measured turn rate across the parameter axis for the steering
    // diagnostic plot. Cached; recomputed on space rebuild, not per frame.
    void RecomputeSteeringCurve();
    void RefreshExampleContacts(PmgExample& example);
    std::vector<int> ResolveContactJointIndices() const;
    static float ComputeGroundOffset(const pmg::Skeleton& skeleton, const pmg::MotionClip& clip);
    float ActiveReferenceDuration() const;

    void BuildWorkflowSection();
    void BuildTransportSection();
    void BuildInputsSection();
    void BuildDisplaySection();
    void BuildMotionSpaceSection();
    void BuildDistanceGridSection();
    void BuildGraphSection();
    void BuildGraphAuthorTab();
    void BuildGraphSpecTab();
    void BuildGraphQuickTab();
    void BuildGraphRuntimeTab();
    void DrawParameterSpace(int axis);
    void DrawPhaseTimeline(float canonical_phase);
    void DrawGraphCanvas();
    void DrawTransitionPipeline();

    void RecomputeHeatmap();
    void SaveHeatmapCsv();
    void BuildGraphRuntime();
    // Path B: in-GUI multi-node graph authoring. Snapshot the current motion
    // space as a node, then (re)build the whole immutable graph from the
    // viewer-side authoring model. Both build paths funnel through
    // InstallSandboxGraph, which owns the controller/alignment/flag wiring.
    void AddAuthoredNode();
    void AddAuthoredEdge(int source_node, int target_node);
    void BuildAuthoredGraph();
    void DrawAuthoredGraphCanvas();
    void InstallSandboxGraph(pmg::ParametricMotionGraph built, int blend_frames,
                             const std::string& status_label,
                             GraphOrigin origin);
    const char* GraphOriginLabel() const;
    const char* GraphPersistenceLabel() const;
    void DiscoverSpecFiles();
    void BuildArtifactFromSpec(const std::string& spec_path);
    void SaveArtifact(const std::string& name);
    void LoadGraphArtifact(const std::string& artifact_path);
    // Shared post-build/-load wiring: installs an artifact as the live runtime
    // (skeleton, graph, controller) and retains it for lossless re-save.
    void AdoptArtifact(pmg::BuiltPmgArtifact artifact,
                       const std::string& status_label,
                       GraphOrigin origin);

    // Build a runtime desired-parameter vector matching a node's dimension:
    // the 1-D control (graph_desired_parameter_) drives axis 0, remaining axes
    // hold the node's per-axis midpoint. Keeps multidimensional nodes from
    // crashing the controller while the runtime steering UI stays 1-D.
    pmg::ParameterVector DesiredParameterForNode(int node) const;

    // Goto steering delegates to the same core module as the CLI.
    void CalibrateSteering();
    void UpdateGotoSteering(const pmg::Pose& pose);
    void UpdateRootMotionDiagnostics(const pmg::Pose& pose, float delta_seconds);

    // SliderFloat plus tick marks at the example parameters, so the user can
    // see where the real clips sit on the blend axis.
    bool ParameterSliderWithTicks(const char* label, float* value,
                                  float min_value, float max_value, int axis);

    void HandleShortcuts();
    void StepFrame(int direction);  // +1 / -1 frame; clip & blend modes only
    void ResetPlayback();
    bool ParametricBlendActive() const;
    bool GraphRuntimeActive() const;

    std::vector<std::filesystem::path> bvh_files_;
    int selected_file_index_ = -1;
    char clip_filter_[64] = "";  // case-insensitive substring filter for the clip list
    std::string status_message_ = "No clip loaded.";

    // Spec library (multi-node graph authoring source) and the artifact most
    // recently built/loaded, kept whole so "Save artifact" is lossless.
    std::vector<std::filesystem::path> spec_files_;
    int selected_spec_index_ = -1;
    char save_artifact_name_[128] = "viewer_graph.pmg";
    std::optional<pmg::BuiltPmgArtifact> source_artifact_;

    pmg::Skeleton skeleton_;
    pmg::MotionClip clip_;
    float ground_offset_ = 0.0f;

    bool playing_ = true;
    float playback_speed_ = 1.0f;
    float current_time_seconds_ = 0.0f;
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
    // Multidimensional parameter authoring. The space has pmg_dimension_ axes;
    // every parameter vector below carries one value per axis. The 1-D
    // visualizations (slider ticks, parameter-space canvas, steering sweep)
    // operate on pmg_view_axis_, holding the other axes at the current value.
    int pmg_dimension_ = 1;
    int pmg_view_axis_ = 0;
    pmg::ParameterVector pmg_parameter_{0.0f};
    pmg::ParameterVector pmg_parameter_min_{0.0f};
    pmg::ParameterVector pmg_parameter_max_{1.0f};
    pmg::ParameterVector next_example_parameter_{0.0f};
    // Cached parametric preview. GenerateClip integrates blended per-frame root
    // deltas in each example's own heading frame (paper path), so a parameter
    // sweep follows an intermediate arc instead of the exaggerated root that
    // blending absolute positions (EvaluatePose) produced. Regenerated when the
    // blend parameter or the space changes; sampled by phase every frame.
    pmg::MotionClip pmg_preview_clip_;
    pmg::ParameterVector pmg_preview_parameter_;
    bool pmg_preview_dirty_ = true;
    // Render-only toggle: lock the horizontal root to the cycle start so the
    // character cycles in place (inspect the pose) instead of tracing its
    // integrated trajectory across the floor.
    bool pmg_preview_in_place_ = false;
    // Measured turn rate (rad/s) of GenerateClip across the blend-parameter
    // axis. A smooth steering response is monotone and spike-free; this is the
    // in-GUI counterpart to the dev/core steering-smoothness diagnostic.
    // Recomputed when the space changes (RecomputeSteeringCurve), not per frame.
    std::vector<float> steering_turn_rate_curve_;     // rad/s per axis sample
    std::vector<float> steering_travel_speed_curve_;  // native units/s per sample

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
    pmg::RuntimeControllerConfig graph_runtime_config_;
    // Owns the alignment strategy injected into the controller; must outlive it,
    // so it is declared first (members destroy in reverse order).
    std::optional<pmg::PointCloudAlignment> graph_alignment_;
    std::optional<pmg::RuntimeController> graph_controller_;
    bool graph_ready_ = false;
    GraphOrigin graph_origin_ = GraphOrigin::None;
    bool graph_open_runtime_tab_ = false;
    float graph_desired_parameter_ = 0.0f;
    // Full per-axis steering vector while a goto is active (empty otherwise);
    // lets multidimensional goal-directed control drive every axis, not just 0.
    pmg::ParameterVector goto_desired_parameter_;
    int graph_desired_node_ = 0;
    int selected_graph_node_ = 0;
    int selected_graph_edge_ = -1;
    // Canvas view-manipulation: per-node drag offsets (display space, added on
    // top of the auto-layout) and the node currently being dragged. Offsets are
    // (re)sized to the node count inside DrawGraphCanvas, so build/load paths
    // need no extra wiring. View-only; does not mutate the graph (Path B).
    std::vector<glm::vec2> graph_node_offsets_;
    int graph_drag_node_ = -1;

    // --- Path B authoring model (sandbox, not saveable) ------------------------
    // Viewer-side description of a multi-node graph. Nodes snapshot a 1-D motion
    // space (+ its skeleton, so a build can reject mismatched skeletons); edges
    // carry per-edge GOOD/BAD thresholds. BuildAuthoredGraph rebuilds the
    // immutable ParametricMotionGraph from these each time, so no remove/edit
    // API is needed on the core graph.
    struct AuthoredNode {
        std::string name;
        pmg::Skeleton skeleton;
        pmg::ParametricMotionSpace space;
    };
    std::vector<AuthoredNode> authored_nodes_;
    std::vector<AuthoredEdge> authored_edges_;
    char authored_node_name_[64] = "node";
    int authored_edge_source_ = 0;
    int authored_edge_target_ = 0;
    std::vector<glm::vec2> authored_node_offsets_;
    int authored_drag_node_ = -1;
    int authored_edge_drag_source_ = -1;

    float graph_fps_ = 30.0f;
    std::string graph_status_ = "Define a motion space, then build a PMG.";
    std::vector<std::string> contact_joint_names_;
    std::string artifact_units_ = "native motion units";

    // --- Goto steering (paper application B/C in the viewer) -------------------
    std::optional<pmg::GoalDirectedLocomotion> steering_;
    bool goto_active_ = false;
    glm::vec2 goto_target_{0.0f, 0.0f};  // native units, ground plane (x, z)
    float goto_tolerance_ = 2.0f;        // native units; arrival radius
    std::string goto_status_;

    float root_heading_radians_ = 0.0f;
    float actual_turn_rate_radians_per_second_ = 0.0f;
    bool root_heading_initialized_ = false;

    RenderScene scene_;
};

}  // namespace pmgviewer
