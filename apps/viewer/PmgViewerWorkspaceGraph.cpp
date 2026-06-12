#include "PmgViewerWorkspace.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pmg/MathTypes.h"

namespace pmgviewer {

namespace {

constexpr float kEpsilon = 1.0e-6f;
constexpr float kGraphCanvasHeight = 300.0f;
constexpr float kGraphNodeRadius = 40.0f;

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

std::string ShortClipLabel(const std::string& name) {
    const std::size_t cut = name.find_last_of("/\\");
    return cut == std::string::npos ? name : name.substr(cut + 1);
}

ImVec2 Add(const ImVec2& left, const ImVec2& right) {
    return ImVec2(left.x + right.x, left.y + right.y);
}

ImVec2 Subtract(const ImVec2& left, const ImVec2& right) {
    return ImVec2(left.x - right.x, left.y - right.y);
}

ImVec2 Scale(const ImVec2& value, float scalar) {
    return ImVec2(value.x * scalar, value.y * scalar);
}

float Length(const ImVec2& value) {
    return std::sqrt(value.x * value.x + value.y * value.y);
}

ImVec2 Normalize(const ImVec2& value) {
    const float length = Length(value);
    return length > kEpsilon ? Scale(value, 1.0f / length) : ImVec2(1.0f, 0.0f);
}

float PointSegmentDistance(
    const ImVec2& point, const ImVec2& segment_start, const ImVec2& segment_end) {
    const ImVec2 segment = Subtract(segment_end, segment_start);
    const float length_squared =
        segment.x * segment.x + segment.y * segment.y;
    if (length_squared <= kEpsilon) {
        return Length(Subtract(point, segment_start));
    }
    const ImVec2 point_offset = Subtract(point, segment_start);
    const float projection = std::clamp(
        (point_offset.x * segment.x + point_offset.y * segment.y) /
            length_squared,
        0.0f, 1.0f);
    return Length(Subtract(
        point, Add(segment_start, Scale(segment, projection))));
}

ImVec2 CubicBezierPoint(
    const ImVec2& p0, const ImVec2& p1, const ImVec2& p2, const ImVec2& p3,
    float t) {
    const float one_minus_t = 1.0f - t;
    return Add(
        Add(
            Scale(p0, one_minus_t * one_minus_t * one_minus_t),
            Scale(p1, 3.0f * one_minus_t * one_minus_t * t)),
        Add(
            Scale(p2, 3.0f * one_minus_t * t * t),
            Scale(p3, t * t * t)));
}

void DrawArrowHead(
    ImDrawList* draw_list, const ImVec2& tip, const ImVec2& direction,
    ImU32 color, float size) {
    const ImVec2 unit = Normalize(direction);
    const ImVec2 perpendicular(-unit.y, unit.x);
    const ImVec2 base = Subtract(tip, Scale(unit, size));
    draw_list->AddTriangleFilled(
        tip,
        Add(base, Scale(perpendicular, 0.55f * size)),
        Subtract(base, Scale(perpendicular, 0.55f * size)),
        color);
}

}  // namespace

// --- Graph runtime (PMG streaming) -----------------------------------------

void PmgViewerWorkspace::LoadGraphArtifact(const std::string& artifact_path) {
    steering_.reset();
    graph_controller_.reset();
    graph_alignment_.reset();

    pmg::BuiltPmgArtifact artifact =
        pmg::LoadPmgArtifactText(artifact_path);
    if (artifact.skeleton.NumJoints() == 0) {
        throw std::runtime_error(
            "viewer runtime requires a complete artifact with a Skeleton");
    }
    if (artifact.graph.NumNodes() == 0 ||
        artifact.metadata.frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "viewer runtime artifact has incomplete graph/frame metadata");
    }

    artifact_units_ = artifact.metadata.units;
    contact_joint_names_.clear();
    const std::string first_node_name = artifact.graph.Node(0).name;
    for (const pmg::NodeRegistrationMetadata& registration :
         artifact.metadata.node_registrations) {
        if (registration.node_name == first_node_name) {
            contact_joint_names_ = registration.contact_joints;
            break;
        }
    }

    pmg_skeleton_ = std::move(artifact.skeleton);
    graph_ = std::move(artifact.graph);
    graph_fps_ = artifact.metadata.frames_per_second;
    pmg_space_ = graph_.Node(0).motion_space;
    if (pmg_space_.ParameterDimension() != 1) {
        throw std::runtime_error(
            "viewer runtime currently requires a one-dimensional first node");
    }
    pmg_examples_.clear();
    for (const pmg::ExampleMotion& example : pmg_space_.Examples()) {
        PmgExample viewer_example{
            ShortClipLabel(example.clip.name), example.parameter.front(),
            example.clip, {}};
        RefreshExampleContacts(viewer_example);
        pmg_examples_.push_back(std::move(viewer_example));
    }
    pmg_space_ready_ = true;
    pmg_parameter_min_ = pmg_space_.MinParameter().front();
    pmg_parameter_max_ = pmg_space_.MaxParameter().front();
    graph_desired_parameter_ =
        0.5f * (pmg_parameter_min_ + pmg_parameter_max_);
    graph_desired_node_ = 0;
    selected_graph_node_ = 0;
    selected_graph_edge_ = graph_.NumEdges() > 0 ? 0 : -1;

    graph_runtime_config_ =
        pmg::RuntimeControllerConfigFromArtifact(artifact);
    graph_alignment_.emplace(
        pmg_skeleton_, graph_runtime_config_.transition_blend_frames);
    graph_controller_.emplace(
        graph_, *graph_alignment_, graph_runtime_config_);
    graph_controller_->Start(0, {graph_desired_parameter_}, graph_fps_);
    graph_ready_ = true;
    mode_ = ViewerPlaybackMode::GraphRuntime;
    playing_ = true;
    graph_status_ = "Loaded artifact: " + artifact_path;
    status_message_ = graph_status_;
}

void PmgViewerWorkspace::BuildGraphRuntime() {
    graph_ready_ = false;
    graph_controller_.reset();
    // A new graph invalidates the achieved-turn-rate table and any active goto.
    steering_.reset();
    goto_active_ = false;
    goto_status_.clear();

    if (!pmg_space_ready_ || pmg_examples_.empty()) {
        graph_status_ = "Add at least one clip to the parametric space first.";
        return;
    }
    if (tbad_ < tgood_) {
        tbad_ = tgood_;
    }

    try {
        graph_ = pmg::ParametricMotionGraph{};
        const int node = graph_.AddNode("viewer_node", pmg_space_);

        pmg::PmgBuilderConfig config;
        config.source_sample_count = 8;     // small for interactive build
        config.target_sample_count = 32;
        config.generated_frames_per_second = graph_fps_;
        config.good_transition_threshold = tgood_;
        config.bad_transition_threshold = tbad_;

        pmg::PmgEdge edge = pmg::PmgBuilder::BuildEdge(
            pmg_skeleton_, node, node, pmg_space_, pmg_space_, config);
        if (edge.samples.empty()) {
            graph_status_ =
                "No valid self-edge (empty box). Raise TGOOD or use a looping clip.";
            return;
        }
        graph_.AddEdge(std::move(edge));

        // Runtime alignment uses the Kovar-derived joint point-cloud adapter.
        // It holds a reference to pmg_skeleton_, so build it first.
        graph_runtime_config_.transition_blend_frames =
            std::max(1, config.distance_grid.window_size);
        graph_alignment_.emplace(
            pmg_skeleton_, graph_runtime_config_.transition_blend_frames);
        graph_controller_.emplace(
            graph_, *graph_alignment_, graph_runtime_config_);
        graph_desired_parameter_ =
            std::clamp(graph_desired_parameter_, pmg_parameter_min_, pmg_parameter_max_);
        graph_desired_node_ = node;
        selected_graph_node_ = node;
        selected_graph_edge_ = 0;
        graph_controller_->Start(node, {graph_desired_parameter_}, graph_fps_);

        graph_ready_ = true;
        mode_ = ViewerPlaybackMode::GraphRuntime;
        graph_status_ = "Graph built: " + std::to_string(edge.samples.size()) +
                        " transition samples. Switched to Graph Runtime.";
    } catch (const std::exception& error) {
        graph_status_ = std::string("Graph build failed: ") + error.what();
    }
}

void PmgViewerWorkspace::DrawGraphCanvas() {
    if (!graph_ready_ || graph_.NumNodes() == 0) {
        return;
    }

    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float width = std::max(260.0f, ImGui::GetContentRegionAvail().x);
    const ImVec2 canvas_size(width, kGraphCanvasHeight);
    const ImVec2 canvas_center(
        origin.x + 0.5f * width,
        origin.y + 0.5f * kGraphCanvasHeight + 10.0f);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    draw_list->AddRectFilled(
        origin, Add(origin, canvas_size), IM_COL32(24, 27, 34, 255), 6.0f);
    draw_list->AddRect(
        origin, Add(origin, canvas_size), IM_COL32(65, 70, 82, 255), 6.0f);

    std::vector<ImVec2> node_positions(
        static_cast<std::size_t>(graph_.NumNodes()), canvas_center);
    if (graph_.NumNodes() == 2) {
        node_positions[0] = ImVec2(
            origin.x + 0.28f * width, canvas_center.y);
        node_positions[1] = ImVec2(
            origin.x + 0.72f * width, canvas_center.y);
    } else if (graph_.NumNodes() > 2) {
        const float radius = std::min(
            0.34f * width, 0.32f * kGraphCanvasHeight);
        for (int node_index = 0; node_index < graph_.NumNodes(); ++node_index) {
            const float angle =
                -0.5f * pmg::kPi +
                2.0f * pmg::kPi *
                    static_cast<float>(node_index) /
                    static_cast<float>(graph_.NumNodes());
            node_positions[node_index] = ImVec2(
                canvas_center.x + radius * std::cos(angle),
                canvas_center.y + radius * std::sin(angle));
        }
    }

    const std::optional<pmg::RuntimeTransitionDiagnostics> active_transition =
        graph_controller_.has_value()
            ? graph_controller_->ActiveTransitionDiagnostics()
            : std::nullopt;

    struct EdgeGeometry {
        bool self_edge = false;
        ImVec2 start;
        ImVec2 control_a;
        ImVec2 control_b;
        ImVec2 end;
    };
    std::vector<EdgeGeometry> edge_geometry(
        static_cast<std::size_t>(graph_.NumEdges()));

    for (int edge_index = 0; edge_index < graph_.NumEdges(); ++edge_index) {
        const pmg::PmgEdge& edge = graph_.Edge(edge_index);
        const ImVec2 source = node_positions[edge.source_node];
        const ImVec2 target = node_positions[edge.target_node];
        const bool is_active =
            active_transition.has_value() &&
            active_transition->source_node == edge.source_node &&
            active_transition->target_node == edge.target_node;
        const bool is_selected = edge_index == selected_graph_edge_;
        const ImU32 color =
            is_active ? IM_COL32(255, 145, 55, 255)
                      : is_selected ? IM_COL32(245, 245, 245, 255)
                                    : IM_COL32(115, 125, 145, 255);
        const float thickness = is_active ? 4.0f : is_selected ? 3.0f : 2.0f;

        if (edge.source_node == edge.target_node) {
            const ImVec2 start(
                source.x - 0.65f * kGraphNodeRadius,
                source.y - 0.75f * kGraphNodeRadius);
            const ImVec2 end(
                source.x + 0.65f * kGraphNodeRadius,
                source.y - 0.75f * kGraphNodeRadius);
            const ImVec2 control_a(
                source.x - 1.8f * kGraphNodeRadius,
                source.y - 2.6f * kGraphNodeRadius);
            const ImVec2 control_b(
                source.x + 1.8f * kGraphNodeRadius,
                source.y - 2.6f * kGraphNodeRadius);
            draw_list->AddBezierCubic(
                start, control_a, control_b, end, color, thickness);
            const ImVec2 before_end = CubicBezierPoint(
                start, control_a, control_b, end, 0.92f);
            DrawArrowHead(
                draw_list, end, Subtract(end, before_end), color, 10.0f);
            edge_geometry[edge_index] =
                {true, start, control_a, control_b, end};
            continue;
        }

        const ImVec2 direction = Normalize(Subtract(target, source));
        const ImVec2 perpendicular(-direction.y, direction.x);
        const float bend =
            edge.source_node < edge.target_node ? 7.0f : -7.0f;
        const ImVec2 offset = Scale(perpendicular, bend);
        const ImVec2 start = Add(
            Add(source, Scale(direction, kGraphNodeRadius + 2.0f)), offset);
        const ImVec2 end = Add(
            Subtract(target, Scale(direction, kGraphNodeRadius + 8.0f)), offset);
        draw_list->AddLine(start, end, color, thickness);
        DrawArrowHead(
            draw_list, end, Subtract(end, start), color, 10.0f);
        edge_geometry[edge_index] =
            {false, start, start, end, end};
    }

    const int active_node =
        graph_controller_.has_value() ? graph_controller_->CurrentNode() : -1;
    for (int node_index = 0; node_index < graph_.NumNodes(); ++node_index) {
        const bool is_active = node_index == active_node;
        const bool is_target = node_index == graph_desired_node_;
        const bool is_selected = node_index == selected_graph_node_;
        const ImVec2 center = node_positions[node_index];
        const ImU32 fill =
            is_active ? IM_COL32(30, 130, 165, 255)
                      : IM_COL32(55, 61, 73, 255);

        if (is_target) {
            draw_list->AddCircle(
                center, kGraphNodeRadius + 8.0f,
                IM_COL32(245, 180, 65, 255), 0, 4.0f);
        }
        if (is_selected) {
            draw_list->AddCircle(
                center, kGraphNodeRadius + 3.0f,
                IM_COL32(245, 245, 245, 255), 0, 2.0f);
        }
        draw_list->AddCircleFilled(center, kGraphNodeRadius, fill);
        draw_list->AddCircle(
            center, kGraphNodeRadius,
            IM_COL32(135, 145, 165, 255), 0, 2.0f);

        const pmg::PmgNode& node = graph_.Node(node_index);
        // The disc is a pure color indicator; name + subtitle stack below it on
        // the dark canvas, so labels of any length stay legible and never spill
        // over the fill.
        const ImVec2 title_size = ImGui::CalcTextSize(node.name.c_str());
        const float label_top = center.y + kGraphNodeRadius + 6.0f;
        draw_list->AddText(
            ImVec2(center.x - 0.5f * title_size.x, label_top),
            IM_COL32(238, 240, 245, 255), node.name.c_str());
        const std::string subtitle =
            std::to_string(node.motion_space.NumExamples()) + " samples / " +
            std::to_string(node.motion_space.ParameterDimension()) + "D";
        const ImVec2 subtitle_size = ImGui::CalcTextSize(subtitle.c_str());
        draw_list->AddText(
            ImVec2(center.x - 0.5f * subtitle_size.x,
                   label_top + title_size.y + 2.0f),
            IM_COL32(168, 174, 186, 255), subtitle.c_str());
    }

    ImGui::InvisibleButton("##pmg_graph_canvas", canvas_size);
    if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        int clicked_node = -1;
        for (int node_index = 0; node_index < graph_.NumNodes(); ++node_index) {
            if (Length(Subtract(mouse, node_positions[node_index])) <=
                kGraphNodeRadius + 8.0f) {
                clicked_node = node_index;
                break;
            }
        }

        if (clicked_node >= 0) {
            selected_graph_node_ = clicked_node;
            selected_graph_edge_ = -1;
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                graph_desired_node_ = clicked_node;
                const pmg::ParametricMotionSpace& target_space =
                    graph_.Node(clicked_node).motion_space;
                if (target_space.ParameterDimension() == 1) {
                    graph_desired_parameter_ = std::clamp(
                        graph_desired_parameter_,
                        target_space.MinParameter().front(),
                        target_space.MaxParameter().front());
                }
                goto_active_ = false;
            }
        } else {
            float best_distance = 12.0f;
            int clicked_edge = -1;
            for (int edge_index = 0; edge_index < graph_.NumEdges(); ++edge_index) {
                const EdgeGeometry& geometry = edge_geometry[edge_index];
                float distance = std::numeric_limits<float>::infinity();
                if (!geometry.self_edge) {
                    distance = PointSegmentDistance(
                        mouse, geometry.start, geometry.end);
                } else {
                    ImVec2 previous = geometry.start;
                    for (int sample = 1; sample <= 24; ++sample) {
                        const ImVec2 current = CubicBezierPoint(
                            geometry.start, geometry.control_a,
                            geometry.control_b, geometry.end,
                            static_cast<float>(sample) / 24.0f);
                        distance = std::min(
                            distance,
                            PointSegmentDistance(mouse, previous, current));
                        previous = current;
                    }
                }
                if (distance < best_distance) {
                    best_distance = distance;
                    clicked_edge = edge_index;
                }
            }
            if (clicked_edge >= 0) {
                selected_graph_edge_ = clicked_edge;
                selected_graph_node_ = -1;
            }
        }
    }

    ImGui::TextDisabled(
        "cyan active   gold runtime target   orange active transition   "
        "white selection");
    ImGui::TextDisabled(
        "click = inspect   double-click node = set runtime target");
}

void PmgViewerWorkspace::BuildGraphSection() {
    ImGui::TextWrapped(
        "ParametricMotionGraph selects an edge from the active motion space, "
        "interpolates its reachable target region and transition phases, aligns "
        "the target motion, then blends through the metric window.");
    ImGui::Text("Nodes: %d   edges: %d", graph_.NumNodes(), graph_.NumEdges());
    ImGui::TextDisabled(
        "Interactive build uses TGOOD/TBAD from Transition Grid.");
    if (ImGui::Button("Build single-node graph from Motion Space")) {
        BuildGraphRuntime();
    }
    ImGui::TextWrapped("%s", graph_status_.c_str());
    ImGui::Separator();

    if (graph_ready_ && graph_controller_.has_value()) {
        const int current_node = graph_controller_->CurrentNode();
        if (graph_desired_node_ < 0 || graph_desired_node_ >= graph_.NumNodes()) {
            graph_desired_node_ = current_node;
        }

        DrawGraphCanvas();
        ImGui::Separator();

        if (selected_graph_node_ >= 0 &&
            selected_graph_node_ < graph_.NumNodes()) {
            const pmg::PmgNode& selected_node =
                graph_.Node(selected_graph_node_);
            ImGui::Text(
                "Selected node: %s   %dD   %d samples",
                selected_node.name.c_str(),
                selected_node.motion_space.ParameterDimension(),
                selected_node.motion_space.NumExamples());
            if (selected_graph_node_ != graph_desired_node_) {
                ImGui::SameLine();
                if (ImGui::Button("Set as runtime target")) {
                    graph_desired_node_ = selected_graph_node_;
                    const pmg::ParametricMotionSpace& selected_space =
                        graph_.Node(graph_desired_node_).motion_space;
                    if (selected_space.ParameterDimension() == 1) {
                        graph_desired_parameter_ = std::clamp(
                            graph_desired_parameter_,
                            selected_space.MinParameter().front(),
                            selected_space.MaxParameter().front());
                    }
                    goto_active_ = false;
                }
            }
        } else if (selected_graph_edge_ >= 0 &&
                   selected_graph_edge_ < graph_.NumEdges()) {
            const pmg::PmgEdge& selected_edge =
                graph_.Edge(selected_graph_edge_);
            ImGui::Text(
                "Selected edge: %s -> %s   %zu source samples",
                graph_.Node(selected_edge.source_node).name.c_str(),
                graph_.Node(selected_edge.target_node).name.c_str(),
                selected_edge.samples.size());
        }

        const pmg::ParametricMotionSpace& target_space =
            graph_.Node(graph_desired_node_).motion_space;
        ImGui::Text(
            "Runtime request: %s -> %s",
            graph_.Node(current_node).name.c_str(),
            graph_.Node(graph_desired_node_).name.c_str());
        if (target_space.ParameterDimension() == 1) {
            const float target_min = target_space.MinParameter().front();
            const float target_max = target_space.MaxParameter().front();
            // While goto steering is active it owns the parameter; show it live
            // but do not let the slider fight the controller.
            ImGui::BeginDisabled(goto_active_);
            ImGui::SliderFloat(
                "Desired parameter", &graph_desired_parameter_,
                target_min, target_max, "%.3f");
            ImGui::EndDisabled();
        } else {
            ImGui::TextDisabled(
                "Viewer controls currently expose one-dimensional target spaces.");
        }

        ImGui::Text("Active: %s   phase %.3f   completed transitions %d",
                    graph_.Node(current_node).name.c_str(),
                    graph_controller_->CurrentPhase(),
                    graph_controller_->CompletedTransitions());
        const pmg::Pose current_pose = graph_controller_->CurrentPose();
        ImGui::Text(
            "Root: x %.2f  z %.2f  heading %.1f deg  actual turn %.3f rad/s",
            current_pose.root_position.x,
            current_pose.root_position.z,
            root_heading_radians_ * 180.0f / pmg::kPi,
            actual_turn_rate_radians_per_second_);
        DrawTransitionPipeline();

        ImGui::Separator();
        ImGui::TextDisabled("Goto steering");
        ImGui::TextWrapped("Right-click the floor (in Graph runtime mode) to set a "
                           "walk target. First click calibrates achieved turn rates "
                           "(a few seconds).");
        if (ImGui::Button(steering_.has_value() ? "Recalibrate" : "Calibrate steering")) {
            CalibrateSteering();
        }
        if (steering_.has_value()) {
            const pmg::SteeringCalibration& calibration =
                steering_->Calibration();
            ImGui::SameLine();
            ImGui::TextDisabled("rates %.2f .. %.2f rad/s",
                                calibration.lowest_rate,
                                calibration.highest_rate);
        }
        if (goto_active_) {
            const pmg::Pose pose = graph_controller_->CurrentPose();
            const float dx = goto_target_.x - pose.root_position.x;
            const float dz = goto_target_.y - pose.root_position.z;
            ImGui::Text("Target (%.1f, %.1f)   distance %.2f",
                        goto_target_.x, goto_target_.y,
                        std::sqrt(dx * dx + dz * dz));
            ImGui::SameLine();
            if (ImGui::Button("Clear target")) {
                goto_active_ = false;
                goto_status_ = "Target cleared.";
            }
        }
        if (!goto_status_.empty()) {
            ImGui::TextWrapped("%s", goto_status_.c_str());
        }
    } else {
        ImGui::TextDisabled(
            "No graph yet. Define motion-space samples and build a PMG.");
    }
}

void PmgViewerWorkspace::DrawTransitionPipeline() {
    if (!graph_ready_ || !graph_controller_.has_value()) {
        return;
    }

    const int source_node = graph_controller_->CurrentNode();
    const pmg::ParameterVector& source_parameter =
        graph_controller_->CurrentParameter();
    const pmg::ParameterVector requested_target{graph_desired_parameter_};

    const pmg::PmgEdge* selected_edge = nullptr;
    for (const int edge_index : graph_.OutgoingEdgeIndices(source_node)) {
        const pmg::PmgEdge& edge = graph_.Edge(edge_index);
        if (edge.target_node == graph_desired_node_) {
            selected_edge = &edge;
            break;
        }
    }

    std::optional<pmg::InterpolatedTransition> preview;
    if (selected_edge != nullptr && source_parameter.size() == 1 &&
        graph_.Node(graph_desired_node_).motion_space.ParameterDimension() == 1) {
        preview = selected_edge->LookupInterpolated(
            source_parameter, requested_target);
    }
    const std::optional<pmg::RuntimeTransitionDiagnostics> active =
        graph_controller_->ActiveTransitionDiagnostics();

    ImGui::Separator();
    ImGui::TextDisabled("PMG transition chain");

    ImGui::TextColored(
        ImVec4(0.35f, 0.78f, 1.0f, 1.0f),
        "1  SOURCE PARAMETER");
    ImGui::SameLine(190.0f);
    if (!source_parameter.empty()) {
        ImGui::Text(
            "%s  p=%.3f",
            graph_.Node(source_node).name.c_str(),
            source_parameter.front());
    }

    ImGui::TextColored(
        ImVec4(0.35f, 0.78f, 1.0f, 1.0f),
        "2  REACHABLE TARGET AABB");
    ImGui::SameLine(190.0f);
    const pmg::ParameterAabb* reachable_box = nullptr;
    if (active.has_value()) {
        reachable_box = &active->reachable_target_box;
    } else if (preview.has_value()) {
        reachable_box = &preview->target_parameter_box;
    }
    if (reachable_box != nullptr && !reachable_box->IsEmpty() &&
        reachable_box->min_corner.size() == 1) {
        const float actual_parameter =
            active.has_value()
                ? active->actual_target_parameter.front()
                : reachable_box->Clamp(requested_target).front();
        ImGui::Text(
            "[%.3f, %.3f]  desired %.3f -> actual %.3f",
            reachable_box->min_corner.front(),
            reachable_box->max_corner.front(),
            graph_desired_parameter_, actual_parameter);
    } else {
        ImGui::TextDisabled("no edge / empty region");
    }

    ImGui::TextColored(
        ImVec4(0.35f, 0.78f, 1.0f, 1.0f),
        "3  TRANSITION PHASES");
    ImGui::SameLine(190.0f);
    if (active.has_value()) {
        ImGui::Text(
            "source %.3f -> target %.3f",
            active->source_transition_phase,
            active->target_transition_phase);
    } else if (preview.has_value()) {
        ImGui::Text(
            "source %.3f -> target %.3f",
            preview->source_transition_phase,
            preview->target_transition_phase);
    } else {
        ImGui::TextDisabled("unavailable");
    }

    if (preview.has_value() || active.has_value()) {
        const float source_phase =
            active.has_value()
                ? active->source_transition_phase
                : preview->source_transition_phase;
        const float target_phase =
            active.has_value()
                ? active->target_transition_phase
                : preview->target_transition_phase;
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const float width = std::max(120.0f, ImGui::GetContentRegionAvail().x);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddLine(
            ImVec2(origin.x, origin.y + 8.0f),
            ImVec2(origin.x + width, origin.y + 8.0f),
            IM_COL32(100, 104, 112, 255), 2.0f);
        DrawPhaseMarker(
            draw_list, source_phase, origin.x, origin.y + 2.0f,
            width, 12.0f, IM_COL32(80, 205, 255, 255));
        DrawPhaseMarker(
            draw_list, target_phase, origin.x, origin.y + 2.0f,
            width, 12.0f, IM_COL32(245, 180, 65, 255));
        ImGui::InvisibleButton("##transition_phases", ImVec2(width, 20.0f));
    }

    ImGui::TextColored(
        ImVec4(0.35f, 0.78f, 1.0f, 1.0f),
        "4  ALIGNMENT TRANSFORM");
    ImGui::SameLine(190.0f);
    if (active.has_value()) {
        ImGui::Text(
            "yaw %.3f rad   dx %.3f   dz %.3f",
            active->alignment.yaw,
            active->alignment.dx,
            active->alignment.dz);
    } else {
        ImGui::TextDisabled("resolved when source enters phase gate");
    }

    ImGui::TextColored(
        ImVec4(0.35f, 0.78f, 1.0f, 1.0f),
        "5  RUNTIME BLEND WINDOW");
    ImGui::SameLine(190.0f);
    if (active.has_value()) {
        ImGui::Text(
            "%d frames   %.3f / %.3f s",
            graph_runtime_config_.transition_blend_frames,
            active->blend_elapsed_seconds,
            active->blend_duration_seconds);
        ImGui::ProgressBar(
            active->blend_progress, ImVec2(-1.0f, 0.0f), "transitioning");
    } else {
        ImGui::Text(
            "%d frames @ %.1f fps   waiting for phase gate",
            graph_runtime_config_.transition_blend_frames, graph_fps_);
    }
}

}  // namespace pmgviewer
