#pragma once

#include <glm/vec2.hpp>

#include <optional>
#include <vector>

#include "pmg/AlignmentStrategy.h"
#include "pmg/ParametricMotionGraph.h"
#include "pmg/RuntimeController.h"

namespace pmgviewer {

struct TransitionTraceMarker {
    glm::vec2 root_position{0.0f};
    int source_node = -1;
    int target_node = -1;
};

struct ViewerRuntimeRequest {
    int desired_node = -1;
    std::optional<pmg::ParameterVector> desired_parameter_raw;
    bool clear_trace = false;
};

enum class ViewerRuntimeStatus {
    kNotInstalled,
    kIdle,
    kNoOpSameParameter,
    kWaitingForPhaseGate,
    kTransitionActive,
    kNoFeasibleTransition,
    kInvalidRequest
};

struct ViewerRuntimeSnapshot {
    bool ready = false;
    int current_node = -1;
    int desired_node = -1;
    float current_phase = 0.0f;
    int completed_transitions = 0;

    ViewerRuntimeStatus status = ViewerRuntimeStatus::kNotInstalled;

    std::optional<pmg::Pose> current_pose;
    std::optional<pmg::ParameterVector> requested_raw;
    std::optional<pmg::ParameterVector> requested_projected;
    std::optional<pmg::ParameterVector> runtime_actual;
    std::optional<pmg::InterpolatedTransition> transition_preview;
    std::optional<pmg::RuntimeTransitionDiagnostics> active_transition;

    std::vector<glm::vec2> runtime_path_points;
    std::vector<TransitionTraceMarker> transition_markers;
};

class ViewerRuntimeModule {
public:
    void Install(
        pmg::ParametricMotionGraph graph,
        const pmg::Skeleton& skeleton,
        const pmg::RuntimeControllerConfig& config,
        float frames_per_second,
        const pmg::ParameterVector& initial_parameter);
    void Clear();
    void Reset();
    void Update(float delta_seconds, const ViewerRuntimeRequest& request);
    void ClearTrace();

    bool Ready() const { return controller_.has_value(); }
    const pmg::RuntimeControllerConfig& Config() const { return config_; }
    const pmg::ParametricMotionGraph& Graph() const { return graph_; }
    ViewerRuntimeSnapshot Snapshot() const;

private:
    void RecordTracePoint(const pmg::Pose& pose);
    void RefreshRequestPreview(const ViewerRuntimeRequest& request);

    pmg::ParametricMotionGraph graph_;
    pmg::RuntimeControllerConfig config_;
    float frames_per_second_ = 30.0f;
    std::optional<pmg::PointCloudAlignment> alignment_;
    std::optional<pmg::RuntimeController> controller_;
    std::vector<glm::vec2> path_points_;
    std::vector<TransitionTraceMarker> transition_markers_;
    std::optional<pmg::ParameterVector> requested_raw_;
    std::optional<pmg::ParameterVector> requested_projected_;
    std::optional<pmg::InterpolatedTransition> transition_preview_;
    int desired_node_ = -1;
    ViewerRuntimeStatus status_ = ViewerRuntimeStatus::kNotInstalled;
};

}  // namespace pmgviewer
