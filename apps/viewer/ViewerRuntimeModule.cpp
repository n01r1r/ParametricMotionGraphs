#include "ViewerRuntimeModule.h"

#include <stdexcept>
#include <utility>

namespace pmgviewer {
namespace {
constexpr std::size_t kMaxPathPoints = 600;
constexpr std::size_t kMaxTransitionMarkers = 128;
constexpr float kMinimumTracePointDistance = 0.01f;

bool HasOutgoingEdgeTo(const pmg::ParametricMotionGraph& graph, int source_node,
                       int target_node) {
    if (source_node < 0 || source_node >= graph.NumNodes() ||
        target_node < 0 || target_node >= graph.NumNodes()) {
        return false;
    }
    for (const int edge_index : graph.OutgoingEdgeIndices(source_node)) {
        if (graph.Edge(edge_index).target_node == target_node) {
            return true;
        }
    }
    return false;
}
}  // namespace

void ViewerRuntimeModule::Install(
    pmg::ParametricMotionGraph graph,
    const pmg::Skeleton& skeleton,
    const pmg::RuntimeControllerConfig& config,
    float frames_per_second,
    const pmg::ParameterVector& initial_parameter) {
    if (graph.NumNodes() == 0) {
        throw std::invalid_argument("ViewerRuntimeModule requires a non-empty graph");
    }
    graph_ = std::move(graph);
    config_ = config;
    frames_per_second_ = frames_per_second;
    alignment_.emplace(skeleton, config.transition_blend_frames);
    controller_.emplace(graph_, *alignment_, config);
    controller_->Start(0, initial_parameter, frames_per_second);
    desired_node_ = 0;
    status_ = ViewerRuntimeStatus::kIdle;
    ClearTrace();
}

void ViewerRuntimeModule::Clear() {
    controller_.reset();
    alignment_.reset();
    graph_ = {};
    path_points_.clear();
    transition_markers_.clear();
    requested_raw_.reset();
    requested_projected_.reset();
    transition_preview_.reset();
    desired_node_ = -1;
    status_ = ViewerRuntimeStatus::kNotInstalled;
}

void ViewerRuntimeModule::Reset() {
    if (!controller_.has_value()) {
        return;
    }
    controller_->Start(
        controller_->CurrentNode(),
        controller_->CurrentParameter(),
        frames_per_second_);
    ClearTrace();
}

void ViewerRuntimeModule::Update(
    float delta_seconds, const ViewerRuntimeRequest& request) {
    if (!controller_.has_value()) {
        throw std::logic_error("ViewerRuntimeModule must be installed before update");
    }
    if (request.clear_trace) {
        ClearTrace();
    }
    if (!request.desired_parameter_raw.has_value()) {
        requested_raw_.reset();
        requested_projected_.reset();
        transition_preview_.reset();
        status_ = ViewerRuntimeStatus::kInvalidRequest;
        return;
    }
    RefreshRequestPreview(request);

    pmg::RuntimeControlRequest control_request;
    control_request.desired_node = request.desired_node;
    control_request.desired_parameter = *request.desired_parameter_raw;
    desired_node_ = request.desired_node;

    const bool was_transitioning = controller_->IsTransitioning();
    controller_->Update(delta_seconds, control_request);
    status_ = controller_->IsTransitioning()
        ? ViewerRuntimeStatus::kTransitionActive
        : ViewerRuntimeStatus::kWaitingForPhaseGate;
    const pmg::ParameterVector& effective_desired_parameter =
        requested_projected_.value_or(control_request.desired_parameter);
    if (!controller_->IsTransitioning() &&
        control_request.desired_node == controller_->CurrentNode() &&
        effective_desired_parameter == controller_->CurrentParameter()) {
        status_ = ViewerRuntimeStatus::kNoOpSameParameter;
    } else if (!controller_->IsTransitioning() &&
               !HasOutgoingEdgeTo(graph_, controller_->CurrentNode(),
                                  control_request.desired_node)) {
        status_ = ViewerRuntimeStatus::kNoFeasibleTransition;
    }
    if (!was_transitioning && controller_->IsTransitioning()) {
        const auto transition = controller_->ActiveTransitionDiagnostics();
        if (transition.has_value()) {
            const pmg::Pose pose = controller_->CurrentPose();
            transition_markers_.push_back({
                {pose.root_position.x, pose.root_position.z},
                transition->source_node,
                transition->target_node,
            });
            if (transition_markers_.size() > kMaxTransitionMarkers) {
                transition_markers_.erase(transition_markers_.begin());
            }
        }
    }
    RecordTracePoint(controller_->CurrentPose());
}

void ViewerRuntimeModule::ClearTrace() {
    path_points_.clear();
    transition_markers_.clear();
    if (controller_.has_value()) {
        RecordTracePoint(controller_->CurrentPose());
    }
}

void ViewerRuntimeModule::RecordTracePoint(const pmg::Pose& pose) {
    const glm::vec2 root(pose.root_position.x, pose.root_position.z);
    if (!path_points_.empty()) {
        const glm::vec2 delta = root - path_points_.back();
        if (delta.x * delta.x + delta.y * delta.y <
            kMinimumTracePointDistance * kMinimumTracePointDistance) {
            return;
        }
    }
    path_points_.push_back(root);
    if (path_points_.size() > kMaxPathPoints) {
        path_points_.erase(path_points_.begin());
    }
}

void ViewerRuntimeModule::RefreshRequestPreview(
    const ViewerRuntimeRequest& request) {
    requested_raw_ = *request.desired_parameter_raw;
    requested_projected_.reset();
    transition_preview_.reset();

    if (request.desired_node < 0 || request.desired_node >= graph_.NumNodes()) {
        return;
    }

    const pmg::ParametricMotionSpace& target_space =
        graph_.Node(request.desired_node).motion_space;
    if (static_cast<int>(request.desired_parameter_raw->size()) !=
        target_space.ParameterDimension()) {
        return;
    }
    requested_projected_ =
        target_space.ProjectToSupport(*request.desired_parameter_raw);

    const int source_node = controller_->CurrentNode();
    for (const int edge_index : graph_.OutgoingEdgeIndices(source_node)) {
        const pmg::PmgEdge& edge = graph_.Edge(edge_index);
        if (edge.target_node != request.desired_node) {
            continue;
        }
        const pmg::ParameterVector& source_parameter =
            controller_->CurrentParameter();
        if (!edge.samples.empty() &&
            source_parameter.size() == edge.samples.front().source_parameter.size()) {
            transition_preview_ = edge.LookupInterpolated(
                source_parameter, *request.desired_parameter_raw);
        }
        return;
    }
}

ViewerRuntimeSnapshot ViewerRuntimeModule::Snapshot() const {
    ViewerRuntimeSnapshot snapshot;
    snapshot.ready = controller_.has_value();
    snapshot.runtime_path_points = path_points_;
    snapshot.transition_markers = transition_markers_;
    snapshot.status = status_;
    snapshot.desired_node = desired_node_;
    if (!controller_.has_value()) {
        return snapshot;
    }
    snapshot.current_node = controller_->CurrentNode();
    snapshot.current_phase = controller_->CurrentPhase();
    snapshot.completed_transitions = controller_->CompletedTransitions();
    snapshot.current_pose = controller_->CurrentPose();
    snapshot.requested_raw = requested_raw_;
    snapshot.requested_projected = requested_projected_;
    snapshot.transition_preview = transition_preview_;
    snapshot.active_transition = controller_->ActiveTransitionDiagnostics();
    snapshot.runtime_actual = snapshot.active_transition.has_value()
        ? snapshot.active_transition->actual_target_parameter
        : controller_->CurrentParameter();
    return snapshot;
}

}  // namespace pmgviewer
