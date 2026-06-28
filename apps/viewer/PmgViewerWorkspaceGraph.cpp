#include "PmgViewerWorkspace.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pmg/GraphSpec.h"
#include "pmg/MathTypes.h"
#include "pmg/CyclicContinuity.h"
#include "pmg/RootCanonicalization.h"
#include "pmg/SkeletonCompatibility.h"

namespace pmgviewer {

namespace {

constexpr float kEpsilon = 1.0e-6f;
constexpr float kGraphCanvasHeight = 300.0f;
constexpr float kGraphNodeRadius = 40.0f;
constexpr float kAuthoringCanvasHeight = 240.0f;
constexpr float kAuthoringNodeRadius = 32.0f;
constexpr float kAuthoringConnectorRadius = 7.0f;

// Ring layout for >2 nodes. The old fixed radius was clamped by canvas height, so
// 3+ nodes (plus their self-loop arcs and labels) overlapped. Instead derive the
// radius from the node count so adjacent node disks always keep a readable gap,
// and grow the canvas height to fit that radius. Width is assumed ample (full
// content region), so only the height needs to grow.
struct RingLayout {
    float radius = 0.0f;
    float height = 0.0f;
};

RingLayout ComputeRingLayout(int node_count, float node_radius,
                             float base_height) {
    RingLayout layout;
    layout.height = base_height;
    if (node_count <= 2) {
        return layout;  // 2-node uses a fixed horizontal layout; no ring.
    }
    // Center-to-center gap that leaves ~0.7*radius of clear space between disks
    // for the edge labels that sit between them.
    const float min_center_distance = 2.7f * node_radius;
    const float chord =
        2.0f * std::sin(pmg::kPi / static_cast<float>(node_count));
    layout.radius =
        std::max(0.22f * base_height, min_center_distance / chord);
    layout.height =
        std::max(base_height, 2.0f * layout.radius + 3.0f * node_radius);
    return layout;
}

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

std::string FrameList(const std::vector<int>& frames) {
    std::ostringstream output;
    output << '[';
    for (std::size_t index = 0; index < frames.size(); ++index) {
        if (index > 0) {
            output << ',';
        }
        output << frames[index];
    }
    output << ']';
    return output.str();
}

std::string ParameterLabel(const pmg::ParameterVector& parameter) {
    std::ostringstream output;
    output << '[';
    for (std::size_t axis = 0; axis < parameter.size(); ++axis) {
        if (axis > 0) {
            output << ", ";
        }
        output << parameter[axis];
    }
    output << ']';
    return output.str();
}

const char* ViewerRuntimeStatusLabel(ViewerRuntimeStatus status) {
    switch (status) {
        case ViewerRuntimeStatus::kNotInstalled:
            return "not installed";
        case ViewerRuntimeStatus::kIdle:
            return "idle";
        case ViewerRuntimeStatus::kNoOpSameParameter:
            return "no-op (parameter unchanged)";
        case ViewerRuntimeStatus::kWaitingForPhaseGate:
            return "waiting for phase gate";
        case ViewerRuntimeStatus::kTransitionActive:
            return "transition active";
        case ViewerRuntimeStatus::kNoFeasibleTransition:
            return "no valid transition";
        case ViewerRuntimeStatus::kInvalidRequest:
            return "invalid request";
    }
    return "unknown";
}

std::string MotionSpaceClipSummary(
    const pmg::ParametricMotionSpace& motion_space) {
    constexpr int kMaxVisibleClipNames = 3;
    std::string summary;
    const std::vector<pmg::ExampleMotion>& examples = motion_space.Examples();
    const int visible_count = std::min(
        static_cast<int>(examples.size()), kMaxVisibleClipNames);
    for (int index = 0; index < visible_count; ++index) {
        if (!summary.empty()) {
            summary += ", ";
        }
        summary += ShortClipLabel(examples[index].clip.name);
    }
    if (static_cast<int>(examples.size()) > visible_count) {
        summary += ", +" +
                   std::to_string(
                       static_cast<int>(examples.size()) - visible_count);
    }
    return summary.empty() ? "none" : summary;
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

pmg::ParameterVector ResolveDesiredParameterForNode(
    const pmg::ParametricMotionSpace& space,
    float scalar_request,
    bool vector_valid,
    const pmg::ParameterVector& vector_request) {
    const int dim = std::max(1, space.ParameterDimension());
    const std::vector<float> lo = space.MinParameter();
    const std::vector<float> hi = space.MaxParameter();
    pmg::ParameterVector desired(static_cast<std::size_t>(dim), 0.0f);
    
    // Default to center of bounding box
    for (int axis = 0; axis < dim; ++axis) {
        const float axis_lo = axis < static_cast<int>(lo.size()) ? lo[axis] : 0.0f;
        const float axis_hi = axis < static_cast<int>(hi.size()) ? hi[axis] : 1.0f;
        desired[static_cast<std::size_t>(axis)] = 0.5f * (axis_lo + axis_hi);
    }

    if (vector_valid && vector_request.size() == desired.size()) {
        return vector_request;
    }

    if (space.HasExplicitParameterSupport()) {
        desired = space.ExplicitSupport()->Project(desired);
    }

    // Fallback: 1-D runtime control steers axis 0.
    desired[0] = scalar_request;

    if (space.HasExplicitParameterSupport()) {
        desired = space.ExplicitSupport()->Project(desired);
    }
    return desired;
}

pmg::ParameterVector PmgViewerWorkspace::DesiredParameterForNode(int node) const {
    if (node < 0 || node >= runtime_.Graph().NumNodes()) {
        return {graph_desired_parameter_};
    }
    const pmg::ParametricMotionSpace& space = runtime_.Graph().Node(node).motion_space;
    return ResolveDesiredParameterForNode(
        space,
        graph_desired_parameter_,
        graph_desired_parameter_vector_valid_,
        graph_desired_parameter_vector_);
}

void PmgViewerWorkspace::ResetGraphRuntimeSession(
    bool discard_source_artifact) {
    graph_origin_ = GraphOrigin::None;
    runtime_.Clear();
    graph_cyclic_summary_ = {};
    graph_cyclic_warning_.clear();
    goto_target_ = {0.0f, 0.0f};
    ResetSteeringState();
    if (discard_source_artifact) {
        source_artifact_.reset();
    }
}

void PmgViewerWorkspace::ResetGraphRuntimeSelection() {
    graph_desired_node_ = 0;
    selected_graph_node_ = 0;
    selected_graph_edge_ = runtime_.Graph().NumEdges() > 0 ? 0 : -1;
    // Fresh graph: discard drag offsets so an equal node count cannot inherit a
    // prior graph's positions (size == NumNodes skips the on-demand reset).
    graph_node_offsets_.clear();
    graph_drag_node_ = -1;
}

void PmgViewerWorkspace::StartGraphRuntimeController(
    pmg::ParametricMotionGraph graph,
    const pmg::RuntimeControllerConfig& config,
    const std::string& status_label, GraphOrigin origin) {
    const pmg::ParameterVector initial_parameter =
        ResolveDesiredParameterForNode(
            graph.Node(0).motion_space,
            graph_desired_parameter_,
            graph_desired_parameter_vector_valid_,
            graph_desired_parameter_vector_);
    runtime_.Install(
        std::move(graph), pmg_skeleton_, config, graph_fps_, initial_parameter);
    graph_origin_ = origin;
    graph_open_runtime_tab_ = true;
    mode_ = ViewerPlaybackMode::GraphRuntime;
    graph_status_ = status_label;
    ResetGraphRuntimeSelection();
}

void PmgViewerWorkspace::RebuildRootCanonicalizationMarkers() {
    root_canonicalization_markers_.clear();
    if (!source_artifact_.has_value()) {
        return;
    }
    const pmg::BuiltPmgArtifact& artifact = *source_artifact_;
    for (const std::string& bvh_path : artifact.metadata.source_bvh_paths) {
        try {
            const pmg::BvhData raw = pmg::BvhLoader::Load(bvh_path);
            const pmg::MotionClip canonical =
                pmg::CanonicalizeRootOrigin(raw.clip);
            RootCanonicalizationMarker marker;
            marker.label = std::filesystem::path(bvh_path).stem().string();
            marker.raw_start = raw.clip.frames.front().root_position;
            marker.normalized_trajectory.reserve(canonical.frames.size());
            for (const pmg::Pose& pose : canonical.frames) {
                marker.normalized_trajectory.push_back(pose.root_position);
            }
            root_canonicalization_markers_.push_back(std::move(marker));
        } catch (const std::exception&) {
            // Diagnostic-only overlay: unavailable raw BVH path should not
            // prevent artifact playback.
        }
    }
}

void PmgViewerWorkspace::AdoptArtifact(
    pmg::BuiltPmgArtifact artifact, const std::string& status_label,
    GraphOrigin origin) {
    // Validate the incoming artifact before disturbing any live runtime state,
    // so a rejected load/build leaves the current graph intact.
    if (artifact.skeleton.NumJoints() == 0) {
        throw std::runtime_error(
            "viewer runtime requires a complete artifact with a Skeleton");
    }
    if (artifact.graph.NumNodes() == 0 ||
        artifact.metadata.frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "viewer runtime artifact has incomplete graph/frame metadata");
    }
    // Multidimensional first nodes are allowed: the runtime steers axis 0 and
    // holds the other axes at their midpoint (see DesiredParameterForNode). The
    // 1-D steering UI degrades gracefully for dim > 1.

    ResetGraphRuntimeSession(false);

    // Retain the whole artifact so "Save artifact" is lossless; the live members
    // below are copies derived from it (load is rare and user-driven).
    source_artifact_ = std::move(artifact);
    const pmg::BuiltPmgArtifact& adopted = *source_artifact_;

    artifact_units_ = adopted.metadata.units;
    contact_joint_names_.clear();
    const std::string first_node_name = adopted.graph.Node(0).name;
    for (const pmg::NodeRegistrationMetadata& registration :
         adopted.metadata.node_registrations) {
        if (registration.node_name == first_node_name) {
            contact_joint_names_ = registration.contact_joints;
            break;
        }
    }

    pmg_skeleton_ = adopted.skeleton;
    RebuildRootCanonicalizationMarkers();
    graph_fps_ = adopted.metadata.frames_per_second;
    pmg_space_ = adopted.graph.Node(0).motion_space;
    pmg_examples_.clear();
    for (const pmg::ExampleMotion& example : pmg_space_.Examples()) {
        PmgExample viewer_example{
            ShortClipLabel(example.clip.name), example.parameter,
            example.clip, {}};
        RefreshExampleContacts(viewer_example);
        pmg_examples_.push_back(std::move(viewer_example));
    }
    pmg_space_ready_ = true;
    pmg_dimension_ = std::max(1, pmg_space_.ParameterDimension());
    ResizeParameterVectors();
    pmg_parameter_min_ = pmg_space_.MinParameter();
    pmg_parameter_max_ = pmg_space_.MaxParameter();
    const float gmin = pmg_parameter_min_.empty() ? 0.0f : pmg_parameter_min_.front();
    const float gmax = pmg_parameter_max_.empty() ? 1.0f : pmg_parameter_max_.front();
    graph_desired_parameter_ = 0.5f * (gmin + gmax);
    StartGraphRuntimeController(
        adopted.graph,
        pmg::RuntimeControllerConfigFromArtifact(adopted),
        status_label, origin);
    playing_ = true;
    graph_cyclic_summary_ =
        pmg::SummarizeArtifactCyclicContinuity(adopted);
    graph_cyclic_warning_ =
        pmg::FormatCyclicContinuityWarning(graph_cyclic_summary_);
    graph_status_ = status_label;
    if (!graph_cyclic_warning_.empty()) {
        graph_status_ += " | " + graph_cyclic_warning_;
    }
    status_message_ = graph_status_;
}

void PmgViewerWorkspace::LoadGraphArtifact(const std::string& artifact_path) {
    AdoptArtifact(pmg::LoadPmgArtifactText(artifact_path),
                  "Loaded artifact: " + artifact_path,
                  GraphOrigin::LoadedArtifact);
}

void PmgViewerWorkspace::BuildArtifactFromSpec(const std::string& spec_path) {
    // Same core path as the CLI --build-graph: the spec's per-edge config is
    // authoritative (the Transition Grid TGOOD/TBAD sliders do not override it).
    try {
        const pmg::GraphSpec spec = pmg::LoadGraphSpec(spec_path);
        AdoptArtifact(pmg::BuildPmgArtifactFromSpec(spec),
                      "Built graph from spec: " + spec_path,
                      GraphOrigin::SpecBuild);
    } catch (const std::exception& error) {
        graph_status_ = std::string("Spec build failed: ") + error.what();
    }
}

void PmgViewerWorkspace::SaveArtifact(const std::string& name) {
    if (!source_artifact_.has_value()) {
        graph_status_ = "No built graph to save.";
        return;
    }
    try {
        std::filesystem::path path(name);
        if (path.is_relative()) {
            path = std::filesystem::path(PMG_OUTPUT_DIRECTORY) / path;
        }
        if (path.extension().empty()) {
            path += ".pmg";
        }
        if (path.has_parent_path()) {
            std::filesystem::create_directories(path.parent_path());
        }
        pmg::SavePmgArtifactText(*source_artifact_, path.string());
        graph_status_ = "Saved artifact: " + path.string();
    } catch (const std::exception& error) {
        graph_status_ = std::string("Save failed: ") + error.what();
    }
}

const char* PmgViewerWorkspace::GraphOriginLabel() const {
    switch (graph_origin_) {
        case GraphOrigin::None:
            return "none";
        case GraphOrigin::LoadedArtifact:
            return "loaded artifact";
        case GraphOrigin::SpecBuild:
            return "spec build";
        case GraphOrigin::QuickSandbox:
            return "quick self-edge sandbox";
        case GraphOrigin::AuthoredSandbox:
            return "authored sandbox";
    }
    return "unknown";
}

const char* PmgViewerWorkspace::GraphPersistenceLabel() const {
    switch (graph_origin_) {
        case GraphOrigin::LoadedArtifact:
        case GraphOrigin::SpecBuild:
            return "saveable";
        case GraphOrigin::QuickSandbox:
        case GraphOrigin::AuthoredSandbox:
            return "not saveable";
        case GraphOrigin::None:
            return "no live graph";
    }
    return "unknown";
}

void PmgViewerWorkspace::BuildGraphRuntime() {
    // The sandbox self-edge graph has no backing artifact; drop any retained one
    // so "Save artifact" cannot write stale metadata from a prior build/load.
    ResetGraphRuntimeSession(true);

    if (!pmg_space_ready_ || pmg_examples_.empty()) {
        graph_status_ = "Add at least one clip to the parametric space first.";
        return;
    }
    if (tbad_ < tgood_) {
        tbad_ = tgood_;
    }

    try {
        pmg::ParametricMotionGraph built;
        const int node = built.AddNode("viewer_node", pmg_space_);

        pmg::PmgBuilderConfig config;
        config.source_sample_count = 8;     // small for interactive build
        config.target_sample_count = 32;
        config.generated_frames_per_second = graph_fps_;
        config.good_transition_threshold = tgood_;
        config.bad_transition_threshold = tbad_;
        config.distance_grid = heatmap_config_;

        pmg::PmgEdge edge = pmg::PmgBuilder::BuildEdge(
            pmg_skeleton_, node, node, pmg_space_, pmg_space_, config);
        if (edge.samples.empty()) {
            graph_status_ =
                "No valid self-edge (empty box). Raise TGOOD or use a looping clip.";
            return;
        }
        const std::size_t sample_count = edge.samples.size();
        built.AddEdge(std::move(edge));
        InstallSandboxGraph(
            std::move(built), std::max(1, config.distance_grid.window_size),
            "Graph built: " + std::to_string(sample_count) +
                " transition samples.",
            GraphOrigin::QuickSandbox);
    } catch (const std::exception& error) {
        graph_status_ = std::string("Graph build failed: ") + error.what();
    }
}

void PmgViewerWorkspace::InstallSandboxGraph(
    pmg::ParametricMotionGraph built, int blend_frames,
    const std::string& status_label, GraphOrigin origin) {
    // Shared install path for the single-node and authored sandbox builds.
    // Sandbox graphs have no backing artifact, so drop any retained one (no
    // stale "Save artifact") and clear the runtime overlays (steering/goto).
    ResetGraphRuntimeSession(true);

    // The viewer's parameter slider and canvas assume a one-dimensional first
    // node; derive its range when present.
    const pmg::ParametricMotionSpace& first_space = built.Node(0).motion_space;
    if (first_space.ParameterDimension() == 1) {
        pmg_parameter_min_ = first_space.MinParameter();
        pmg_parameter_max_ = first_space.MaxParameter();
        const float gmin =
            pmg_parameter_min_.empty() ? 0.0f : pmg_parameter_min_.front();
        const float gmax =
            pmg_parameter_max_.empty() ? 1.0f : pmg_parameter_max_.front();
        graph_desired_parameter_ = std::clamp(graph_desired_parameter_, gmin, gmax);
    }
    // Runtime alignment holds a reference to pmg_skeleton_, so build it first.
    pmg::RuntimeControllerConfig runtime_config = runtime_.Config();
    runtime_config.transition_blend_frames = std::max(1, blend_frames);
    StartGraphRuntimeController(
        std::move(built), runtime_config, status_label, origin);
}

void PmgViewerWorkspace::AddAuthoredNode() {
    if (!pmg_space_ready_ || pmg_examples_.empty()) {
        graph_status_ =
            "Author a motion space (add clips in Inputs) before adding a node.";
        return;
    }
    if (pmg_space_.ParameterDimension() != 1) {
        graph_status_ = "Motion space must be one-dimensional to add as a node.";
        return;
    }
    const std::string base_name =
        authored_node_name_[0] != '\0' ? authored_node_name_ : "node";
    std::string name = base_name;
    int suffix = 2;
    while (std::any_of(
        authored_nodes_.begin(), authored_nodes_.end(),
        [&name](const AuthoredNode& node) { return node.name == name; })) {
        name = base_name + " " + std::to_string(suffix);
        ++suffix;
    }
    authored_nodes_.push_back(AuthoredNode{name, pmg_skeleton_, pmg_space_});
    graph_status_ = "Added node '" + name + "' (" +
                    std::to_string(pmg_space_.NumExamples()) +
                    " samples). Authored nodes: " +
                    std::to_string(authored_nodes_.size());
}

void PmgViewerWorkspace::AddAuthoredEdge(
    int source_node, int target_node) {
    const AddAuthoredEdgeResult result = pmgviewer::AddAuthoredEdge(
        static_cast<int>(authored_nodes_.size()), source_node, target_node,
        tgood_, tbad_, authored_edges_);
    switch (result) {
        case AddAuthoredEdgeResult::Added:
            graph_status_ =
                "Added authored edge " + std::to_string(source_node) + "->" +
                std::to_string(target_node) +
                " from current TGOOD/TBAD.";
            return;
        case AddAuthoredEdgeResult::Duplicate:
            graph_status_ =
                "Edge " + std::to_string(source_node) + "->" +
                std::to_string(target_node) + " already exists.";
            return;
        case AddAuthoredEdgeResult::InvalidNode:
            graph_status_ = "Cannot add edge: source or target node is invalid.";
            return;
        case AddAuthoredEdgeResult::InvalidThreshold:
            graph_status_ =
                "Cannot add edge: require finite 0 <= TGOOD <= TBAD.";
            return;
    }
}

void PmgViewerWorkspace::BuildAuthoredGraph() {
    if (authored_nodes_.empty()) {
        graph_status_ = "Add at least one authored node first.";
        return;
    }
    // All nodes share one alignment skeleton; reject a mixed set up front.
    const pmg::Skeleton& first_skeleton = authored_nodes_.front().skeleton;
    for (int node_index = 0;
         node_index < static_cast<int>(authored_nodes_.size());
         ++node_index) {
        const pmg::SkeletonCompatibilityResult compatibility =
            pmg::CheckSkeletonCompatibility(
                first_skeleton, authored_nodes_[node_index].skeleton);
        if (!compatibility.compatible) {
            graph_status_ =
                "Authored node " + std::to_string(node_index) +
                " has an incompatible skeleton: " + compatibility.reason;
            return;
        }
    }
    if (authored_nodes_.front().space.ParameterDimension() != 1) {
        graph_status_ = "First authored node must be one-dimensional.";
        return;
    }

    try {
        pmg::ParametricMotionGraph built;
        for (const AuthoredNode& node : authored_nodes_) {
            built.AddNode(node.name, node.space);
        }

        const int node_count = static_cast<int>(authored_nodes_.size());
        int edges_built = 0;
        std::string skipped;
        for (const AuthoredEdge& authored : authored_edges_) {
            if (authored.source_node < 0 || authored.source_node >= node_count ||
                authored.target_node < 0 || authored.target_node >= node_count) {
                graph_status_ =
                    "Authored edge has an invalid source or target node.";
                return;
            }
            if (!std::isfinite(authored.tgood) ||
                !std::isfinite(authored.tbad) || authored.tgood < 0.0f ||
                authored.tbad < authored.tgood) {
                graph_status_ =
                    "Authored edge thresholds require finite "
                    "0 <= TGOOD <= TBAD.";
                return;
            }
            pmg::PmgBuilderConfig config;
            config.source_sample_count = 8;     // small for interactive build
            config.target_sample_count = 32;
            config.generated_frames_per_second = graph_fps_;
            config.good_transition_threshold = authored.tgood;
            config.bad_transition_threshold = authored.tbad;
            config.distance_grid = heatmap_config_;

            pmg::PmgEdge edge = pmg::PmgBuilder::BuildEdge(
                first_skeleton, authored.source_node, authored.target_node,
                authored_nodes_[authored.source_node].space,
                authored_nodes_[authored.target_node].space, config);
            if (edge.samples.empty()) {
                skipped += " " + std::to_string(authored.source_node) + "->" +
                           std::to_string(authored.target_node);
                continue;
            }
            built.AddEdge(std::move(edge));
            ++edges_built;
        }

        // Adopt the authored skeleton as the live alignment skeleton.
        pmg_skeleton_ = first_skeleton;
        std::string status = "Authored graph: " + std::to_string(node_count) +
                             " nodes, " + std::to_string(edges_built) + " edges.";
        if (!skipped.empty()) {
            status += " Skipped empty edges:" + skipped + " (raise TGOOD).";
        }
        InstallSandboxGraph(
            std::move(built), 5, status, GraphOrigin::AuthoredSandbox);
    } catch (const std::exception& error) {
        graph_status_ = std::string("Authored build failed: ") + error.what();
    }
}

void PmgViewerWorkspace::DrawAuthoredGraphCanvas() {
    const int node_count = static_cast<int>(authored_nodes_.size());
    if (node_count == 0) {
        return;
    }

    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float width = std::max(260.0f, ImGui::GetContentRegionAvail().x);
    const RingLayout layout = ComputeRingLayout(
        node_count, kAuthoringNodeRadius, kAuthoringCanvasHeight);
    const ImVec2 canvas_size(width, layout.height);
    const ImVec2 canvas_center(
        origin.x + 0.5f * width, origin.y + 0.5f * layout.height);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    draw_list->AddRectFilled(
        origin, Add(origin, canvas_size), IM_COL32(24, 27, 34, 255), 6.0f);
    draw_list->AddRect(
        origin, Add(origin, canvas_size), IM_COL32(65, 70, 82, 255), 6.0f);

    std::vector<ImVec2> node_positions(
        static_cast<std::size_t>(node_count), canvas_center);
    if (node_count == 2) {
        node_positions[0] =
            ImVec2(origin.x + 0.28f * width, canvas_center.y);
        node_positions[1] =
            ImVec2(origin.x + 0.72f * width, canvas_center.y);
    } else if (node_count > 2) {
        const float radius = std::min(0.42f * width, layout.radius);
        for (int node_index = 0; node_index < node_count; ++node_index) {
            const float angle =
                -0.5f * pmg::kPi +
                2.0f * pmg::kPi * static_cast<float>(node_index) /
                    static_cast<float>(node_count);
            node_positions[node_index] = ImVec2(
                canvas_center.x + radius * std::cos(angle),
                canvas_center.y + radius * std::sin(angle));
        }
    }

    // Preserve existing offsets when nodes are appended or removed.
    authored_node_offsets_.resize(
        static_cast<std::size_t>(node_count), glm::vec2(0.0f));
    for (int node_index = 0; node_index < node_count; ++node_index) {
        node_positions[node_index].x += authored_node_offsets_[node_index].x;
        node_positions[node_index].y += authored_node_offsets_[node_index].y;
    }

    for (const AuthoredEdge& edge : authored_edges_) {
        if (edge.source_node < 0 || edge.source_node >= node_count ||
            edge.target_node < 0 || edge.target_node >= node_count) {
            continue;
        }
        const ImVec2 source = node_positions[edge.source_node];
        const ImVec2 target = node_positions[edge.target_node];
        const ImU32 edge_color = IM_COL32(120, 135, 160, 255);
        if (edge.source_node == edge.target_node) {
            const float loop_side =
                source.x < canvas_center.x ? 1.0f : -1.0f;
            const ImVec2 start(
                source.x + loop_side * 0.65f * kAuthoringNodeRadius,
                source.y - 0.55f * kAuthoringNodeRadius);
            const ImVec2 end(
                source.x + loop_side * 0.65f * kAuthoringNodeRadius,
                source.y + 0.55f * kAuthoringNodeRadius);
            const ImVec2 control_a(
                source.x + loop_side * 1.8f * kAuthoringNodeRadius,
                source.y - 1.0f * kAuthoringNodeRadius);
            const ImVec2 control_b(
                source.x + loop_side * 1.8f * kAuthoringNodeRadius,
                source.y + 1.0f * kAuthoringNodeRadius);
            draw_list->AddBezierCubic(
                start, control_a, control_b, end, edge_color, 2.0f);
            const ImVec2 before_end = CubicBezierPoint(
                start, control_a, control_b, end, 0.92f);
            DrawArrowHead(
                draw_list, end, Subtract(end, before_end), edge_color, 8.0f);
            continue;
        }

        const ImVec2 direction = Normalize(Subtract(target, source));
        const ImVec2 start =
            Add(source, Scale(direction, kAuthoringNodeRadius + 7.0f));
        const ImVec2 end =
            Subtract(target, Scale(direction, kAuthoringNodeRadius + 7.0f));
        draw_list->AddLine(start, end, edge_color, 2.0f);
        DrawArrowHead(
            draw_list, end, Subtract(end, start), edge_color, 8.0f);
    }

    const ImVec2 mouse_position = ImGui::GetIO().MousePos;
    const bool canvas_hovered =
        ImGui::IsMouseHoveringRect(origin, Add(origin, canvas_size));
    int hovered_node = -1;
    int hovered_connector = -1;
    if (canvas_hovered) {
        for (int node_index = 0; node_index < node_count; ++node_index) {
            const ImVec2 connector = Add(
                node_positions[node_index],
                ImVec2(kAuthoringNodeRadius, 0.0f));
            if (Length(Subtract(mouse_position, connector)) <=
                kAuthoringConnectorRadius + 4.0f) {
                hovered_connector = node_index;
                break;
            }
        }
        for (int node_index = 0; node_index < node_count; ++node_index) {
            if (Length(Subtract(
                    mouse_position, node_positions[node_index])) <=
                kAuthoringNodeRadius) {
                hovered_node = node_index;
                break;
            }
        }
    }

    for (int node_index = 0; node_index < node_count; ++node_index) {
        const ImVec2 center = node_positions[node_index];
        const bool is_drag_source = node_index == authored_edge_drag_source_;
        const bool is_hovered = node_index == hovered_node;
        const ImU32 fill =
            is_drag_source ? IM_COL32(125, 95, 35, 255)
                           : is_hovered ? IM_COL32(80, 88, 104, 255)
                                        : IM_COL32(55, 61, 73, 255);
        draw_list->AddCircleFilled(center, kAuthoringNodeRadius, fill);
        draw_list->AddCircle(
            center, kAuthoringNodeRadius, IM_COL32(135, 145, 165, 255), 0,
            2.0f);

        const std::string& label = authored_nodes_[node_index].name;
        const ImVec2 label_size = ImGui::CalcTextSize(label.c_str());
        draw_list->AddText(
            ImVec2(
                center.x - 0.5f * label_size.x,
                center.y - label_size.y),
            IM_COL32(238, 240, 245, 255), label.c_str());
        const std::string subtitle =
            std::to_string(
                authored_nodes_[node_index].space.NumExamples()) +
            " samples";
        const ImVec2 subtitle_size = ImGui::CalcTextSize(subtitle.c_str());
        draw_list->AddText(
            ImVec2(
                center.x - 0.5f * subtitle_size.x,
                center.y + 2.0f),
            IM_COL32(168, 174, 186, 255), subtitle.c_str());

        const ImVec2 connector =
            Add(center, ImVec2(kAuthoringNodeRadius, 0.0f));
        const ImU32 connector_color =
            node_index == hovered_connector
                ? IM_COL32(255, 205, 90, 255)
                : IM_COL32(220, 165, 60, 255);
        draw_list->AddCircleFilled(
            connector, kAuthoringConnectorRadius, connector_color);
        draw_list->AddCircle(
            connector, kAuthoringConnectorRadius,
            IM_COL32(255, 230, 150, 255), 0, 1.5f);
    }

    if (authored_edge_drag_source_ >= 0 &&
        authored_edge_drag_source_ < node_count) {
        const ImVec2 source = Add(
            node_positions[authored_edge_drag_source_],
            ImVec2(kAuthoringNodeRadius, 0.0f));
        draw_list->AddLine(
            source, mouse_position, IM_COL32(255, 205, 90, 255), 3.0f);
        DrawArrowHead(
            draw_list, mouse_position, Subtract(mouse_position, source),
            IM_COL32(255, 205, 90, 255), 8.0f);
    }

    ImGui::InvisibleButton("##pmg_authored_graph_canvas", canvas_size);
    if (ImGui::IsItemActivated()) {
        if (hovered_connector >= 0) {
            authored_edge_drag_source_ = hovered_connector;
            authored_drag_node_ = -1;
        } else {
            authored_drag_node_ = hovered_node;
            authored_edge_drag_source_ = -1;
        }
    }
    if (ImGui::IsItemActive() && authored_drag_node_ >= 0 &&
        authored_drag_node_ < node_count) {
        const ImVec2 drag_delta = ImGui::GetIO().MouseDelta;
        authored_node_offsets_[authored_drag_node_].x += drag_delta.x;
        authored_node_offsets_[authored_drag_node_].y += drag_delta.y;
    }
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (authored_edge_drag_source_ >= 0) {
            if (hovered_node >= 0) {
                AddAuthoredEdge(authored_edge_drag_source_, hovered_node);
            } else {
                graph_status_ =
                    "Edge creation cancelled: release over a target node.";
            }
        }
        authored_edge_drag_source_ = -1;
        authored_drag_node_ = -1;
    }

    ImGui::TextDisabled(
        "Drag gold dot to target node. Drag node body to arrange.");
}

void PmgViewerWorkspace::DrawGraphCanvas() {
    if (!runtime_.Ready() || runtime_.Graph().NumNodes() == 0) {
        return;
    }

    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const float width = std::max(260.0f, ImGui::GetContentRegionAvail().x);
    const RingLayout layout = ComputeRingLayout(
        runtime_.Graph().NumNodes(), kGraphNodeRadius, kGraphCanvasHeight);
    const ImVec2 canvas_size(width, layout.height);
    const ImVec2 canvas_center(
        origin.x + 0.5f * width,
        origin.y + 0.47f * layout.height);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    draw_list->AddRectFilled(
        origin, Add(origin, canvas_size), IM_COL32(24, 27, 34, 255), 6.0f);
    draw_list->AddRect(
        origin, Add(origin, canvas_size), IM_COL32(65, 70, 82, 255), 6.0f);

    std::vector<ImVec2> node_positions(
        static_cast<std::size_t>(runtime_.Graph().NumNodes()), canvas_center);
    if (runtime_.Graph().NumNodes() == 2) {
        node_positions[0] = ImVec2(
            origin.x + 0.28f * width, canvas_center.y);
        node_positions[1] = ImVec2(
            origin.x + 0.72f * width, canvas_center.y);
    } else if (runtime_.Graph().NumNodes() > 2) {
        const float radius = std::min(0.42f * width, layout.radius);
        for (int node_index = 0; node_index < runtime_.Graph().NumNodes(); ++node_index) {
            const float angle =
                -0.5f * pmg::kPi +
                2.0f * pmg::kPi *
                    static_cast<float>(node_index) /
                    static_cast<float>(runtime_.Graph().NumNodes());
            node_positions[node_index] = ImVec2(
                canvas_center.x + radius * std::cos(angle),
                canvas_center.y + radius * std::sin(angle));
        }
    }

    // Drag offsets ride on top of the auto-layout. Resize on demand so build and
    // load paths need no extra wiring; a node-count change resets the layout.
    if (static_cast<int>(graph_node_offsets_.size()) != runtime_.Graph().NumNodes()) {
        graph_node_offsets_.assign(
            static_cast<std::size_t>(runtime_.Graph().NumNodes()), glm::vec2(0.0f));
        graph_drag_node_ = -1;
    }
    for (int node_index = 0; node_index < runtime_.Graph().NumNodes(); ++node_index) {
        node_positions[node_index].x += graph_node_offsets_[node_index].x;
        node_positions[node_index].y += graph_node_offsets_[node_index].y;
    }

    // Hover detection (before drawing, so the hovered node can be highlighted).
    const bool canvas_hovered =
        ImGui::IsMouseHoveringRect(origin, Add(origin, canvas_size));
    const ImVec2 mouse_position = ImGui::GetIO().MousePos;
    int hovered_node = -1;
    if (canvas_hovered) {
        for (int node_index = 0; node_index < runtime_.Graph().NumNodes(); ++node_index) {
            if (Length(Subtract(mouse_position, node_positions[node_index])) <=
                kGraphNodeRadius + 8.0f) {
                hovered_node = node_index;
                break;
            }
        }
    }

    const std::optional<pmg::RuntimeTransitionDiagnostics> active_transition =
        runtime_.Snapshot().active_transition;

    struct EdgeGeometry {
        bool self_edge = false;
        ImVec2 start;
        ImVec2 control_a;
        ImVec2 control_b;
        ImVec2 end;
    };
    std::vector<EdgeGeometry> edge_geometry(
        static_cast<std::size_t>(runtime_.Graph().NumEdges()));

    for (int edge_index = 0; edge_index < runtime_.Graph().NumEdges(); ++edge_index) {
        const pmg::PmgEdge& edge = runtime_.Graph().Edge(edge_index);
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
            const float loop_side =
                source.x < canvas_center.x ? 1.0f : -1.0f;
            const ImVec2 start(
                source.x + loop_side * 0.65f * kGraphNodeRadius,
                source.y - 0.55f * kGraphNodeRadius);
            const ImVec2 end(
                source.x + loop_side * 0.65f * kGraphNodeRadius,
                source.y + 0.55f * kGraphNodeRadius);
            const ImVec2 control_a(
                source.x + loop_side * 1.8f * kGraphNodeRadius,
                source.y - 1.0f * kGraphNodeRadius);
            const ImVec2 control_b(
                source.x + loop_side * 1.8f * kGraphNodeRadius,
                source.y + 1.0f * kGraphNodeRadius);
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

    const int active_node = runtime_.Snapshot().current_node;
    for (int node_index = 0; node_index < runtime_.Graph().NumNodes(); ++node_index) {
        const bool is_active = node_index == active_node;
        const bool is_target = node_index == graph_desired_node_;
        const bool is_selected = node_index == selected_graph_node_;
        const ImVec2 center = node_positions[node_index];
        const bool is_hovered = node_index == hovered_node;
        ImU32 fill =
            is_active ? IM_COL32(30, 130, 165, 255)
                      : IM_COL32(55, 61, 73, 255);
        if (is_hovered) {
            fill = is_active ? IM_COL32(40, 160, 200, 255)
                             : IM_COL32(80, 88, 104, 255);
        }

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

        const pmg::PmgNode& node = runtime_.Graph().Node(node_index);
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
    // Drag a node to reposition it (view declutter only; the graph is untouched).
    if (ImGui::IsItemActivated() && hovered_node >= 0) {
        graph_drag_node_ = hovered_node;
    }
    if (ImGui::IsItemActive() && graph_drag_node_ >= 0 &&
        graph_drag_node_ < static_cast<int>(graph_node_offsets_.size())) {
        const ImVec2 drag_delta = ImGui::GetIO().MouseDelta;
        graph_node_offsets_[graph_drag_node_].x += drag_delta.x;
        graph_node_offsets_[graph_drag_node_].y += drag_delta.y;
        selected_graph_node_ = graph_drag_node_;
        selected_graph_edge_ = -1;
    }
    if (!ImGui::IsItemActive()) {
        graph_drag_node_ = -1;
    }
    if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        int clicked_node = -1;
        for (int node_index = 0; node_index < runtime_.Graph().NumNodes(); ++node_index) {
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
                SetDesiredRuntimeNode(clicked_node);
            }
        } else {
            float best_distance = 12.0f;
            int clicked_edge = -1;
            for (int edge_index = 0; edge_index < runtime_.Graph().NumEdges(); ++edge_index) {
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
        "cyan active | gold target | orange transition | white selected | drag node");
}

void PmgViewerWorkspace::DrawParameterCoverage() {
    ImGui::TextUnformatted("Parameter-space coverage");
    ImGui::TextDisabled(
        "Example samples and the axis-extreme corners of the parameter box. A "
        "red ring marks a corner no example reaches (that quadrant is "
        "extrapolation, not interpolation).");
    if (!runtime_.Ready() || runtime_.Graph().NumNodes() == 0) {
        ImGui::TextDisabled("Load or build a graph to inspect coverage.");
        return;
    }

    selected_graph_node_ =
        std::clamp(selected_graph_node_, 0, runtime_.Graph().NumNodes() - 1);
    if (ImGui::BeginCombo("Node##coverage_node",
                          runtime_.Graph().Node(selected_graph_node_).name.c_str())) {
        for (int index = 0; index < runtime_.Graph().NumNodes(); ++index) {
            const bool selected = index == selected_graph_node_;
            if (ImGui::Selectable(runtime_.Graph().Node(index).name.c_str(), selected)) {
                selected_graph_node_ = index;
            }
        }
        ImGui::EndCombo();
    }

    const pmg::ParametricMotionSpace& space =
        runtime_.Graph().Node(selected_graph_node_).motion_space;
    const std::vector<pmg::ExampleMotion>& examples = space.Examples();
    const int dimension = space.ParameterDimension();
    if (examples.empty() || dimension <= 0) {
        ImGui::TextDisabled("Node has no examples.");
        return;
    }
    const pmg::ParameterVector min_parameter = space.MinParameter();
    const pmg::ParameterVector max_parameter = space.MaxParameter();

    // Spanned axes and full-dimension corner coverage, the same quantities the
    // spec `expect` validator checks offline.
    int spanned_axes = 0;
    for (int axis = 0; axis < dimension; ++axis) {
        const float first = examples.front().parameter[axis];
        for (const pmg::ExampleMotion& example : examples) {
            if (example.parameter[axis] != first) {
                ++spanned_axes;
                break;
            }
        }
    }
    constexpr int kMaxCornerDimension = 8;
    int sampled_corners = 0;
    int total_corners = 0;
    if (dimension <= kMaxCornerDimension) {
        total_corners = 1 << dimension;
        for (int mask = 0; mask < total_corners; ++mask) {
            bool found = false;
            for (const pmg::ExampleMotion& example : examples) {
                bool matches = true;
                for (int axis = 0; axis < dimension; ++axis) {
                    const float want = (mask & (1 << axis)) ? max_parameter[axis]
                                                            : min_parameter[axis];
                    if (example.parameter[axis] != want) {
                        matches = false;
                        break;
                    }
                }
                if (matches) {
                    found = true;
                    break;
                }
            }
            if (found) {
                ++sampled_corners;
            }
        }
    }
    if (total_corners > 0) {
        ImGui::Text("%d samples \xc2\xb7 spans %d of %d axes \xc2\xb7 "
                    "%d/%d box corners sampled",
                    static_cast<int>(examples.size()), spanned_axes, dimension,
                    sampled_corners, total_corners);
    } else {
        ImGui::Text("%d samples \xc2\xb7 spans %d of %d axes",
                    static_cast<int>(examples.size()), spanned_axes, dimension);
    }

    const int x_axis = 0;
    const int y_axis = dimension >= 2 ? 1 : -1;
    if (dimension > 2) {
        ImGui::TextDisabled("%d-D space; plotting axes 0 and 1.", dimension);
    } else if (dimension == 1) {
        ImGui::TextDisabled("1-D space; samples shown along the axis.");
    }

    const ImVec2 canvas_origin = ImGui::GetCursorScreenPos();
    const float canvas_width =
        std::max(ImGui::GetContentRegionAvail().x, 160.0f);
    const float canvas_height = 240.0f;
    ImGui::Dummy(ImVec2(canvas_width, canvas_height));
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(
        canvas_origin,
        ImVec2(canvas_origin.x + canvas_width, canvas_origin.y + canvas_height),
        IM_COL32(18, 22, 30, 255));

    const float pad = 40.0f;
    const float left = canvas_origin.x + pad;
    const float right = canvas_origin.x + canvas_width - pad;
    const float top = canvas_origin.y + 24.0f;
    const float bottom = canvas_origin.y + canvas_height - pad;

    auto axis_bounds = [&](int axis, float& lo, float& hi) {
        lo = (axis >= 0 && axis < static_cast<int>(min_parameter.size()))
                 ? min_parameter[axis]
                 : 0.0f;
        hi = (axis >= 0 && axis < static_cast<int>(max_parameter.size()))
                 ? max_parameter[axis]
                 : 1.0f;
    };
    float x_lo = 0.0f;
    float x_hi = 1.0f;
    float y_lo = 0.0f;
    float y_hi = 1.0f;
    axis_bounds(x_axis, x_lo, x_hi);
    if (y_axis >= 0) {
        axis_bounds(y_axis, y_lo, y_hi);
    }
    // Display ranges pad zero-width axes so a single value still renders.
    float x_view_lo = x_lo;
    float x_view_hi = x_hi;
    if (x_view_hi - x_view_lo < kEpsilon) {
        x_view_lo -= 0.5f;
        x_view_hi += 0.5f;
    }
    float y_view_lo = y_lo;
    float y_view_hi = y_hi;
    if (y_view_hi - y_view_lo < kEpsilon) {
        y_view_lo -= 0.5f;
        y_view_hi += 0.5f;
    }
    auto to_screen = [&](float value_x, float value_y) {
        const float tx = (value_x - x_view_lo) / (x_view_hi - x_view_lo);
        const float ty = (y_axis >= 0)
                             ? (value_y - y_view_lo) / (y_view_hi - y_view_lo)
                             : 0.5f;
        return ImVec2(left + tx * (right - left), bottom - ty * (bottom - top));
    };

    // Parameter box (the example AABB).
    draw_list->AddRect(to_screen(x_lo, y_lo), to_screen(x_hi, y_hi),
                       IM_COL32(70, 110, 160, 200));

    // Authored 2-D simplex support. This is only a visualization of the
    // sampled triangle; runtime still requests against the node's parameter
    // coordinates and transition boxes.
    if (dimension == 2 && examples.size() == 3) {
        for (int index = 0; index < 3; ++index) {
            const pmg::ParameterVector& start =
                examples[static_cast<std::size_t>(index)].parameter;
            const pmg::ParameterVector& end =
                examples[static_cast<std::size_t>((index + 1) % 3)].parameter;
            draw_list->AddLine(
                to_screen(start[0], start[1]),
                to_screen(end[0], end[1]),
                IM_COL32(80, 205, 255, 210), 2.0f);
        }
    }

    // Mark every box corner of the displayed axes; red ring where unsampled.
    if (dimension >= 1) {
        const float corner_x[2] = {x_lo, x_hi};
        const float corner_y[2] = {y_lo, y_hi};
        const int y_count = (y_axis >= 0) ? 2 : 1;
        for (int ix = 0; ix < 2; ++ix) {
            for (int iy = 0; iy < y_count; ++iy) {
                bool sampled = false;
                for (const pmg::ExampleMotion& example : examples) {
                    const bool x_ok = example.parameter[x_axis] == corner_x[ix];
                    const bool y_ok =
                        (y_axis < 0) || example.parameter[y_axis] == corner_y[iy];
                    if (x_ok && y_ok) {
                        sampled = true;
                        break;
                    }
                }
                if (!sampled) {
                    const ImVec2 p = to_screen(corner_x[ix], corner_y[iy]);
                    draw_list->AddCircle(p, 9.0f, IM_COL32(235, 90, 110, 255),
                                         0, 2.0f);
                    draw_list->AddText(ImVec2(p.x + 8.0f, p.y - 16.0f),
                                       IM_COL32(235, 90, 110, 255), "no sample");
                }
            }
        }
    }

    // Example samples.
    for (const pmg::ExampleMotion& example : examples) {
        const float vx = example.parameter[x_axis];
        const float vy = (y_axis >= 0) ? example.parameter[y_axis] : 0.0f;
        const ImVec2 p = to_screen(vx, vy);
        draw_list->AddCircleFilled(p, 5.0f, IM_COL32(120, 200, 255, 255));
        draw_list->AddText(ImVec2(p.x + 7.0f, p.y - 6.0f),
                           IM_COL32(200, 220, 240, 255),
                           ShortClipLabel(example.clip.name).c_str());
    }

    // Axis labels.
    draw_list->AddText(ImVec2(right - 36.0f, bottom + 6.0f),
                       IM_COL32(150, 160, 175, 255), "axis 0");
    if (y_axis >= 0) {
        draw_list->AddText(ImVec2(canvas_origin.x + 4.0f, top - 18.0f),
                           IM_COL32(150, 160, 175, 255), "axis 1");
    }
}

void PmgViewerWorkspace::BuildGraphSection() {
    ImGui::Text("Current graph: %s", GraphOriginLabel());
    ImGui::SameLine();
    if (runtime_.Ready()) {
        ImGui::TextDisabled(
            "| %d nodes, %d edges | %s",
            runtime_.Graph().NumNodes(), runtime_.Graph().NumEdges(), GraphPersistenceLabel());
    } else {
        ImGui::TextDisabled("| no live graph");
    }
    ImGui::TextWrapped("Last action: %s", graph_status_.c_str());

    if (!ImGui::BeginTabBar("##graph_workspace_tabs")) {
        return;
    }

    if (ImGui::BeginTabItem("Build")) {
        BuildGraphBuildTab();
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Coverage")) {
        DrawParameterCoverage();
        ImGui::EndTabItem();
    }

    const ImGuiTabItemFlags runtime_flags =
        graph_open_runtime_tab_ ? ImGuiTabItemFlags_SetSelected
                                : ImGuiTabItemFlags_None;
    if (ImGui::BeginTabItem("Runtime", nullptr, runtime_flags)) {
        BuildGraphRuntimeTab();
        ImGui::EndTabItem();
    }
    graph_open_runtime_tab_ = false;
    ImGui::EndTabBar();
}

void PmgViewerWorkspace::BuildGraphBuildTab() {
    // One Build tab, three sources: Author = manual snapshot + edges (saveable),
    // Spec = from .pmg_spec (saveable), Quick = scratch self-edge (not saveable).
    ImGui::SeparatorText("Source");
    ImGui::RadioButton("Author", &graph_build_source_, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Spec", &graph_build_source_, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Quick", &graph_build_source_, 2);
    ImGui::Separator();
    switch (graph_build_source_) {
    case 1: BuildGraphSpecTab(); break;
    case 2: BuildGraphQuickTab(); break;
    default: BuildGraphAuthorTab(); break;
    }
}

void PmgViewerWorkspace::BuildGraphSpecTab() {
    ImGui::TextUnformatted("Build saveable graph from .pmg_spec");
    ImGui::TextDisabled(
        "Spec owns nodes, edges, thresholds, phase ranges, registration, and "
        "metadata.");
    if (spec_files_.empty()) {
        ImGui::TextDisabled("No .pmg_spec files found in the spec directory.");
        return;
    }

    if (selected_spec_index_ < 0 ||
        selected_spec_index_ >= static_cast<int>(spec_files_.size())) {
        selected_spec_index_ = 0;
    }
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::BeginCombo(
            "Spec file##spec_select",
            spec_files_[selected_spec_index_].filename().string().c_str())) {
        for (int index = 0; index < static_cast<int>(spec_files_.size());
             ++index) {
            const bool selected = index == selected_spec_index_;
            if (ImGui::Selectable(
                    spec_files_[index].filename().string().c_str(), selected)) {
                selected_spec_index_ = index;
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::Button("Build and open Runtime")) {
        BuildArtifactFromSpec(spec_files_[selected_spec_index_].string());
    }
}

void PmgViewerWorkspace::BuildGraphQuickTab() {
    ImGui::TextUnformatted("Quick self-edge diagnostic");
    ImGui::TextDisabled(
        "Build one node from current motion space. Scratch only; not saveable.");
    ImGui::TextDisabled(
        "Uses current thresholds and Distance Grid phase range [%.2f, %.2f] "
        "-> [%.2f, %.2f].",
        heatmap_config_.source_phase_start, heatmap_config_.source_phase_end,
        heatmap_config_.target_phase_start, heatmap_config_.target_phase_end);

    ImGui::SetNextItemWidth(180.0f);
    ImGui::InputFloat("GOOD threshold##quick", &tgood_, 1.0f, 10.0f, "%.3f");
    ImGui::SetNextItemWidth(180.0f);
    ImGui::InputFloat("BAD threshold##quick", &tbad_, 1.0f, 10.0f, "%.3f");
    tgood_ = std::max(0.0f, tgood_);
    tbad_ = std::max(tgood_, tbad_);
    if (heatmap_ready_) {
        const bool likely_ok = heatmap_min_distance_ <= tgood_;
        ImGui::TextColored(
            likely_ok ? ImVec4(0.40f, 0.80f, 0.45f, 1.0f)
                      : ImVec4(0.90f, 0.55f, 0.30f, 1.0f),
            "Distance preview: min %.3f vs GOOD %.3f -> %s",
            heatmap_min_distance_, tgood_,
            likely_ok ? "likely buildable" : "likely empty");
    }
    if (ImGui::Button("Build and open Runtime")) {
        BuildGraphRuntime();
    }
}

void PmgViewerWorkspace::BuildGraphAuthorTab() {
    ImGui::TextUnformatted("1. Snapshot motion spaces as nodes");
    if (pmg_space_ready_ && pmg_space_.ParameterDimension() == 1) {
        ImGui::Text(
            "Current snapshot: %d samples | range [%.3f, %.3f]",
            pmg_space_.NumExamples(), pmg_space_.MinParameter().front(),
            pmg_space_.MaxParameter().front());
        ImGui::TextDisabled(
            "Clips: %s", MotionSpaceClipSummary(pmg_space_).c_str());
    } else {
        ImGui::TextDisabled(
            "Current motion space unavailable. Add clips on Motion Space tab.");
    }

    ImGui::SetNextItemWidth(150.0f);
    ImGui::InputTextWithHint(
        "##authored_node_name", "node name", authored_node_name_,
        sizeof(authored_node_name_));
    ImGui::SameLine();
    ImGui::BeginDisabled(
        !pmg_space_ready_ || pmg_space_.ParameterDimension() != 1);
    if (ImGui::Button("Add snapshot")) {
        AddAuthoredNode();
    }
    ImGui::EndDisabled();

    if (!authored_nodes_.empty()) {
        int remove_node = -1;
        ImGui::Text("Nodes (%d)", static_cast<int>(authored_nodes_.size()));
        for (int index = 0; index < static_cast<int>(authored_nodes_.size());
             ++index) {
            const AuthoredNode& node = authored_nodes_[index];
            ImGui::PushID(index);
            ImGui::Text(
                "%d. %s | %d samples | [%.3f, %.3f]",
                index + 1, node.name.c_str(), node.space.NumExamples(),
                node.space.MinParameter().front(),
                node.space.MaxParameter().front());
            ImGui::SameLine();
            if (ImGui::SmallButton("x##node")) {
                remove_node = index;
            }
            ImGui::TextDisabled(
                "   %s", MotionSpaceClipSummary(node.space).c_str());
            ImGui::PopID();
        }
        if (remove_node >= 0) {
            authored_nodes_.erase(authored_nodes_.begin() + remove_node);
            if (remove_node <
                static_cast<int>(authored_node_offsets_.size())) {
                authored_node_offsets_.erase(
                    authored_node_offsets_.begin() + remove_node);
            }
            authored_drag_node_ = -1;
            authored_edge_drag_source_ = -1;
            // Drop edges touching the removed node; renumber survivors so the
            // stored endpoints stay aligned with the node vector.
            std::vector<AuthoredEdge> kept;
            kept.reserve(authored_edges_.size());
            for (AuthoredEdge edge : authored_edges_) {
                if (edge.source_node == remove_node ||
                    edge.target_node == remove_node) {
                    continue;
                }
                if (edge.source_node > remove_node) --edge.source_node;
                if (edge.target_node > remove_node) --edge.target_node;
                kept.push_back(edge);
            }
            authored_edges_.swap(kept);
        }
    }

    ImGui::Separator();
    ImGui::TextUnformatted("2. Connect nodes");
    ImGui::TextDisabled(
        "Thresholds apply to each new edge; raw-sum units are corpus-specific.");
    ImGui::TextDisabled(
        "Build uses Distance Grid phase range [%.2f, %.2f] -> [%.2f, %.2f].",
        heatmap_config_.source_phase_start, heatmap_config_.source_phase_end,
        heatmap_config_.target_phase_start, heatmap_config_.target_phase_end);
    ImGui::SetNextItemWidth(180.0f);
    ImGui::InputFloat("GOOD threshold##author", &tgood_, 1.0f, 10.0f, "%.3f");
    ImGui::SetNextItemWidth(180.0f);
    ImGui::InputFloat("BAD threshold##author", &tbad_, 1.0f, 10.0f, "%.3f");
    tgood_ = std::max(0.0f, tgood_);
    tbad_ = std::max(tgood_, tbad_);

    const int node_count = static_cast<int>(authored_nodes_.size());
    if (node_count > 0) {
        DrawAuthoredGraphCanvas();

        authored_edge_source_ =
            std::clamp(authored_edge_source_, 0, node_count - 1);
        authored_edge_target_ =
            std::clamp(authored_edge_target_, 0, node_count - 1);
        if (ImGui::TreeNode("Advanced: add edge by list")) {
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::BeginCombo(
                    "Source##edge_source",
                    authored_nodes_[authored_edge_source_].name.c_str())) {
                for (int index = 0; index < node_count; ++index) {
                    const std::string item =
                        authored_nodes_[index].name + "##src" +
                        std::to_string(index);
                    if (ImGui::Selectable(
                            item.c_str(), index == authored_edge_source_)) {
                        authored_edge_source_ = index;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::BeginCombo(
                    "Target##edge_target",
                    authored_nodes_[authored_edge_target_].name.c_str())) {
                for (int index = 0; index < node_count; ++index) {
                    const std::string item =
                        authored_nodes_[index].name + "##tgt" +
                        std::to_string(index);
                    if (ImGui::Selectable(
                            item.c_str(), index == authored_edge_target_)) {
                        authored_edge_target_ = index;
                    }
                }
                ImGui::EndCombo();
            }
            if (ImGui::Button("Add directed edge")) {
                AddAuthoredEdge(
                    authored_edge_source_, authored_edge_target_);
            }
            ImGui::TreePop();
        }
    } else {
        ImGui::TextDisabled("Add at least one node snapshot.");
    }

    if (!authored_edges_.empty()) {
        ImGui::Text("Edges (%d)", static_cast<int>(authored_edges_.size()));
        int remove_edge = -1;
        for (int index = 0; index < static_cast<int>(authored_edges_.size());
             ++index) {
            AuthoredEdge& edge = authored_edges_[index];
            ImGui::PushID(1000 + index);
            ImGui::Text(
                "%d. %s -> %s",
                index + 1,
                authored_nodes_[edge.source_node].name.c_str(),
                authored_nodes_[edge.target_node].name.c_str());
            ImGui::SameLine();
            ImGui::TextUnformatted("GOOD");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(65.0f);
            ImGui::InputFloat("##aetg", &edge.tgood, 0.0f, 0.0f, "%.2f");
            ImGui::SameLine();
            ImGui::TextUnformatted("BAD");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(65.0f);
            ImGui::InputFloat("##aetb", &edge.tbad, 0.0f, 0.0f, "%.2f");
            ImGui::SameLine();
            if (ImGui::SmallButton("x##edge")) {
                remove_edge = index;
            }
            ImGui::PopID();
        }
        if (remove_edge >= 0) {
            authored_edges_.erase(authored_edges_.begin() + remove_edge);
        }
    }

    ImGui::Separator();
    ImGui::TextUnformatted("3. Build runtime graph");
    ImGui::TextDisabled(
        "%d nodes, %d edges | replaces current live graph | not saveable",
        static_cast<int>(authored_nodes_.size()),
        static_cast<int>(authored_edges_.size()));
    ImGui::BeginDisabled(authored_nodes_.empty());
    if (ImGui::Button("Build and open Runtime")) {
        BuildAuthoredGraph();
    }
    ImGui::EndDisabled();
}

void PmgViewerWorkspace::BuildGraphRuntimeTab() {
    if (!runtime_.Ready()) {
        ImGui::TextDisabled(
            "No live graph. Build one from the Build tab (Author / Spec / Quick).");
        return;
    }

    ImGui::Text(
        "%s | %d nodes, %d edges | %s",
        GraphOriginLabel(), runtime_.Graph().NumNodes(), runtime_.Graph().NumEdges(),
        GraphPersistenceLabel());
    if (!graph_cyclic_warning_.empty()) {
        ImGui::TextColored(
            ImVec4(1.0f, 0.78f, 0.35f, 1.0f),
            "%s",
            graph_cyclic_warning_.c_str());
    }
    if (!graph_cyclic_summary_.samples.empty() &&
        ImGui::CollapsingHeader("Cyclic continuity")) {
        ImGui::TextDisabled(
            "Strong %d / %d | pose %d | root %d | yaw %d | contact %d",
            graph_cyclic_summary_.strong_count,
            graph_cyclic_summary_.cyclic_sample_count,
            graph_cyclic_summary_.weak_pose_seam_count,
            graph_cyclic_summary_.weak_root_speed_count,
            graph_cyclic_summary_.weak_yaw_rate_count,
            graph_cyclic_summary_.weak_contact_count);
        if (ImGui::BeginTable(
                "cyclic_continuity_table", 7,
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                    ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Node");
            ImGui::TableSetupColumn("Param");
            ImGui::TableSetupColumn("Clip");
            ImGui::TableSetupColumn("Class");
            ImGui::TableSetupColumn("Seam");
            ImGui::TableSetupColumn("Root");
            ImGui::TableSetupColumn("Yaw");
            ImGui::TableHeadersRow();
            for (const pmg::CyclicContinuitySampleSummary& sample :
                 graph_cyclic_summary_.samples) {
                const pmg::CyclicContinuityRecord& record = sample.record;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(sample.node_name.c_str());
                ImGui::TableSetColumnIndex(1);
                const std::string parameter = ParameterLabel(sample.parameter);
                ImGui::TextUnformatted(parameter.c_str());
                ImGui::TableSetColumnIndex(2);
                const std::string clip = ShortClipLabel(sample.clip_name);
                ImGui::TextUnformatted(clip.c_str());
                ImGui::TableSetColumnIndex(3);
                ImGui::TextUnformatted(
                    pmg::CyclicContinuityClassificationName(
                        record.classification));
                ImGui::TableSetColumnIndex(4);
                ImGui::Text("%.2f", record.seam_step_ratio);
                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%.2f", record.root_speed_ratio);
                ImGui::TableSetColumnIndex(6);
                ImGui::Text("%.2f", record.yaw_rate_ratio);
            }
            ImGui::EndTable();
        }
    }
    if (source_artifact_.has_value()) {
        ImGui::SetNextItemWidth(200.0f);
        ImGui::InputTextWithHint(
            "##save_name", "artifact filename", save_artifact_name_,
            sizeof(save_artifact_name_));
        ImGui::SameLine();
        if (ImGui::Button("Save artifact") &&
            save_artifact_name_[0] != '\0') {
            SaveArtifact(save_artifact_name_);
        }
    }

    const ViewerRuntimeSnapshot runtime = runtime_.Snapshot();
    const int current_node = runtime.current_node;
    if (graph_desired_node_ < 0 || graph_desired_node_ >= runtime_.Graph().NumNodes()) {
        graph_desired_node_ = current_node;
    }

    DrawGraphCanvas();
    ImGui::Separator();

    if (selected_graph_node_ >= 0 &&
        selected_graph_node_ < runtime_.Graph().NumNodes()) {
        const pmg::PmgNode& selected_node =
            runtime_.Graph().Node(selected_graph_node_);
        ImGui::Text(
            "Selected node: %s | %dD | %d samples",
            selected_node.name.c_str(),
            selected_node.motion_space.ParameterDimension(),
            selected_node.motion_space.NumExamples());
        if (selected_graph_node_ != graph_desired_node_) {
            ImGui::SameLine();
            if (ImGui::Button("Set runtime target")) {
                SetDesiredRuntimeNode(selected_graph_node_);
            }
        }
    } else if (selected_graph_edge_ >= 0 &&
               selected_graph_edge_ < runtime_.Graph().NumEdges()) {
        const pmg::PmgEdge& selected_edge =
            runtime_.Graph().Edge(selected_graph_edge_);
        ImGui::Text(
            "Selected edge: %s -> %s | %zu transition samples",
            runtime_.Graph().Node(selected_edge.source_node).name.c_str(),
            runtime_.Graph().Node(selected_edge.target_node).name.c_str(),
            selected_edge.samples.size());
    }

    const pmg::ParametricMotionSpace& target_space =
        runtime_.Graph().Node(graph_desired_node_).motion_space;
    ImGui::Text(
        "Runtime request: %s -> %s",
        runtime_.Graph().Node(current_node).name.c_str(),
        runtime_.Graph().Node(graph_desired_node_).name.c_str());
    if (target_space.ParameterDimension() == 1) {
        const float target_min = target_space.MinParameter().front();
        const float target_max = target_space.MaxParameter().front();
        if (std::abs(target_max - target_min) <= pmg::kSmallEpsilon) {
            graph_desired_parameter_ = target_min;
            ImGui::TextDisabled(
                "Desired parameter fixed at %.3f (one authored example).",
                target_min);
        } else {
            ImGui::BeginDisabled(goto_active_);
            ImGui::SliderFloat(
                "Desired parameter", &graph_desired_parameter_,
                target_min, target_max, "%.3f");
            ImGui::EndDisabled();
        }
    } else if (target_space.ParameterDimension() == 2 && target_space.HasExplicitParameterSupport()) {
        const pmg::ParameterSupport& support = *target_space.ExplicitSupport();
        ImGui::TextDisabled("2D Parameter Control (click to request target)");

        const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        const float canvas_size = 200.0f;
        const ImVec2 canvas_max = ImVec2(canvas_pos.x + canvas_size, canvas_pos.y + canvas_size);

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->AddRectFilled(canvas_pos, canvas_max, IM_COL32(30, 30, 35, 255));
        draw_list->AddRect(canvas_pos, canvas_max, IM_COL32(100, 100, 100, 255));

        const std::vector<float> lo = target_space.MinParameter();
        const std::vector<float> hi = target_space.MaxParameter();
        const float p0_min = lo[0], p0_max = hi[0];
        const float p1_min = lo[1], p1_max = hi[1];
        
        const float p0_range = std::max(1e-5f, p0_max - p0_min);
        const float p1_range = std::max(1e-5f, p1_max - p1_min);

        auto param_to_canvas = [&](const pmg::ParameterVector& p) {
            float nx = (p[0] - p0_min) / p0_range;
            float ny = (p[1] - p1_min) / p1_range;
            return ImVec2(canvas_pos.x + nx * canvas_size, canvas_pos.y + (1.0f - ny) * canvas_size);
        };
        auto canvas_to_param = [&](const ImVec2& c) {
            float nx = (c.x - canvas_pos.x) / canvas_size;
            float ny = 1.0f - (c.y - canvas_pos.y) / canvas_size;
            return pmg::ParameterVector{
                std::clamp(p0_min + nx * p0_range, p0_min - p0_range * 0.5f, p0_max + p0_range * 0.5f),
                std::clamp(p1_min + ny * p1_range, p1_min - p1_range * 0.5f, p1_max + p1_range * 0.5f)};
        };

        if (support.GetType() == pmg::ParameterSupport::Type::kTriangulated2D) {
            for (const auto& tri : support.Triangles()) {
                ImVec2 v0 = param_to_canvas(support.Vertices()[tri[0]]);
                ImVec2 v1 = param_to_canvas(support.Vertices()[tri[1]]);
                ImVec2 v2 = param_to_canvas(support.Vertices()[tri[2]]);
                draw_list->AddTriangleFilled(v0, v1, v2, IM_COL32(60, 70, 80, 255));
                draw_list->AddTriangle(v0, v1, v2, IM_COL32(100, 120, 140, 255));
            }
        } else if (support.GetType() == pmg::ParameterSupport::Type::kSimplex && support.NumVertices() == 3) {
            ImVec2 v0 = param_to_canvas(support.Vertices()[0]);
            ImVec2 v1 = param_to_canvas(support.Vertices()[1]);
            ImVec2 v2 = param_to_canvas(support.Vertices()[2]);
            draw_list->AddTriangleFilled(v0, v1, v2, IM_COL32(60, 70, 80, 255));
            draw_list->AddTriangle(v0, v1, v2, IM_COL32(100, 120, 140, 255));
        }

        for (const auto& v : support.Vertices()) {
            draw_list->AddCircleFilled(param_to_canvas(v), 3.0f, IM_COL32(200, 200, 200, 255));
        }

        ImGui::InvisibleButton("##canvas", ImVec2(canvas_size, canvas_size));
        const bool is_hovered = ImGui::IsItemHovered();
        const bool is_active = ImGui::IsItemActive();
        if ((is_hovered && ImGui::IsMouseClicked(0)) || (is_active && ImGui::IsMouseDragging(0))) {
            graph_desired_parameter_vector_ = canvas_to_param(ImGui::GetMousePos());
            graph_desired_parameter_vector_valid_ = true;
        }

        if (runtime.requested_raw.has_value() &&
            runtime.requested_raw->size() == 2) {
            const ImVec2 requested = param_to_canvas(*runtime.requested_raw);
            draw_list->AddCircleFilled(
                requested, 4.0f, IM_COL32(255, 100, 100, 255));
            draw_list->AddCircle(
                requested, 5.0f, IM_COL32(255, 255, 255, 255));
        }
        if (runtime.requested_projected.has_value() &&
            runtime.requested_projected->size() == 2) {
            const ImVec2 projected = param_to_canvas(*runtime.requested_projected);
            draw_list->AddCircleFilled(
                projected, 4.0f, IM_COL32(255, 200, 100, 255));
        }
        if (runtime.runtime_actual.has_value() &&
            runtime.runtime_actual->size() == 2) {
            const ImVec2 actual = param_to_canvas(*runtime.runtime_actual);
            draw_list->AddCircleFilled(actual, 4.0f, IM_COL32(100, 255, 100, 255));
        }
        
        ImGui::TextDisabled("Requested parameter is updated immediately; actual transition\n"
                            "is scheduled when the current PMG edge reaches its valid transition phase.");
    } else {
        ImGui::TextDisabled(
            "Viewer controls currently expose one/two-dimensional target spaces.");
    }

    ImGui::Text(
        "Active: %s | phase %.3f | completed transitions %d",
        runtime_.Graph().Node(current_node).name.c_str(),
        runtime.current_phase,
        runtime.completed_transitions);
    const pmg::Pose current_pose = runtime.current_pose.value_or(pmg::Pose{});
    ImGui::Text(
        "Root: x %.2f | z %.2f | heading %.1f deg | turn %.3f rad/s",
        current_pose.root_position.x,
        current_pose.root_position.z,
        root_heading_radians_ * 180.0f / pmg::kPi,
        actual_turn_rate_radians_per_second_);
    ImGui::Checkbox("Path trail", &show_graph_path_trail_);
    ImGui::SameLine();
    ImGui::Checkbox(
        "Transition markers", &show_graph_transition_markers_);
    ImGui::Checkbox("Show root canonicalization markers",
                    &show_root_canonicalization_markers_);
    ImGui::SameLine();
    ImGui::Checkbox("Show anchor root trajectories",
                    &show_anchor_root_trajectories_);
    ImGui::SameLine();
    if (ImGui::Button("Clear trace")) {
        runtime_.ClearTrace();
    }
    ImGui::TextDisabled(
        "%zu trail points | %zu transitions",
        runtime.runtime_path_points.size(), runtime.transition_markers.size());
    DrawTransitionPipeline();

    ImGui::Separator();
    if (ImGui::Checkbox("Direct Steering Mode", &direct_steering_active_)) {
        ResetGotoState(direct_steering_active_
            ? "Arrow keys: left/right steer, up/down set speed."
            : "Direct steering disabled.");
        if (direct_steering_active_ && !steering_.has_value()) {
            CalibrateSteering();
        }
    }
    ImGui::TextDisabled("Arrow keys steer; no input holds last safe request.");
    if (ImGui::Checkbox("Path Following Mode", &path_following_active_)) {
        path_waypoints_.clear();
        path_waypoint_index_ = 0;
        ResetGotoState(path_following_active_
            ? "Right-click floor to append path waypoints."
            : "Path following disabled.");
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear Path")) {
        path_waypoints_.clear();
        path_waypoint_index_ = 0;
        ResetGotoState("Path cleared.");
    }
    ImGui::Text("Path: %zu waypoints", path_waypoints_.size());
    ImGui::TextDisabled("Steering: right-click floor to set target");
    if (ImGui::Button(
            steering_.has_value() ? "Recalibrate" : "Calibrate steering")) {
        CalibrateSteering();
    }
    if (steering_.has_value()) {
        const pmg::SteeringCalibration& calibration = steering_->Calibration();
        ImGui::SameLine();
        ImGui::TextDisabled(
            "rates %.2f .. %.2f rad/s",
            calibration.LowestRate(), calibration.HighestRate());
    }
    if (goto_active_) {
        const pmg::Pose pose = runtime.current_pose.value_or(pmg::Pose{});
        const float dx = goto_target_.x - pose.root_position.x;
        const float dz = goto_target_.y - pose.root_position.z;
        ImGui::Text(
            "Target (%.1f, %.1f) | distance %.2f",
            goto_target_.x, goto_target_.y, std::sqrt(dx * dx + dz * dz));
        ImGui::SameLine();
        if (ImGui::Button("Clear target")) {
            ResetGotoState("Target cleared.");
        }
    }
    if (!goto_status_.empty()) {
        ImGui::TextWrapped("%s", goto_status_.c_str());
    }
}

void PmgViewerWorkspace::DrawTransitionPipeline() {
    if (!runtime_.Ready()) {
        return;
    }

    const ViewerRuntimeSnapshot runtime = runtime_.Snapshot();
    const std::optional<pmg::RuntimeTransitionDiagnostics>& active =
        runtime.active_transition;
    const int source_node = active.has_value()
        ? active->source_node
        : runtime.current_node;
    const int target_node = active.has_value()
        ? active->target_node
        : runtime.desired_node;
    const pmg::ParameterVector source_parameter =
        active.has_value()
            ? active->source_parameter
            : runtime.runtime_actual.value_or(pmg::ParameterVector{});
    const auto node_label = [&](int node) -> const char* {
        if (node < 0 || node >= runtime_.Graph().NumNodes()) {
            return "none";
        }
        return runtime_.Graph().Node(node).name.c_str();
    };

    ImGui::Separator();
    ImGui::TextDisabled("Transition");

    constexpr const char* kSourceParameterLabel = "SOURCE";
    constexpr const char* kRequestedRawLabel = "REQUESTED RAW";
    constexpr const char* kProjectedSupportLabel = "REQUESTED PROJECTED";
    constexpr const char* kActualTargetLabel = "RUNTIME ACTUAL";
    constexpr const char* kSchedulingLabel = "SCHEDULING";
    constexpr const char* kCompletedTransitionsLabel = "COMPLETED";
    constexpr const char* kRuntimeSupportLabel = "RUNTIME SUPPORT";
    constexpr const char* kAlignmentTransformLabel = "ALIGNMENT";
    constexpr const char* kRuntimeBlendWindowLabel = "BLEND";

    const float widest_label_width = std::max({
        ImGui::CalcTextSize(kSourceParameterLabel).x,
        ImGui::CalcTextSize(kRequestedRawLabel).x,
        ImGui::CalcTextSize(kProjectedSupportLabel).x,
        ImGui::CalcTextSize(kActualTargetLabel).x,
        ImGui::CalcTextSize(kSchedulingLabel).x,
        ImGui::CalcTextSize(kCompletedTransitionsLabel).x,
    });
    const float value_column_x =
        ImGui::GetCursorPosX() + widest_label_width +
        ImGui::GetStyle().ItemSpacing.x;

    ImGui::TextColored(ImVec4(0.35f, 0.78f, 1.0f, 1.0f), kSourceParameterLabel);
    ImGui::SameLine(value_column_x);
    if (!source_parameter.empty()) {
        ImGui::Text("%s  p=%s",
                    node_label(source_node),
                    ParameterLabel(source_parameter).c_str());
    }

    ImGui::TextColored(ImVec4(0.35f, 0.78f, 1.0f, 1.0f), kRequestedRawLabel);
    ImGui::SameLine(value_column_x);
    ImGui::Text("%s  p=%s",
                node_label(target_node),
                ParameterLabel(
                    runtime.requested_raw.value_or(pmg::ParameterVector{})).c_str());

    ImGui::TextColored(ImVec4(0.35f, 0.78f, 1.0f, 1.0f), kProjectedSupportLabel);
    ImGui::SameLine(value_column_x);
    ImGui::Text("%s  p=%s",
                node_label(target_node),
                ParameterLabel(
                    runtime.requested_projected.value_or(pmg::ParameterVector{})).c_str());

    ImGui::TextColored(ImVec4(0.35f, 0.78f, 1.0f, 1.0f), kActualTargetLabel);
    ImGui::SameLine(value_column_x);
    if (runtime.runtime_actual.has_value()) {
        ImGui::Text("%s  p=%s",
                    node_label(target_node),
                    ParameterLabel(*runtime.runtime_actual).c_str());
    } else {
        ImGui::TextDisabled("pending");
    }

    ImGui::TextColored(ImVec4(0.35f, 0.78f, 1.0f, 1.0f), kSchedulingLabel);
    ImGui::SameLine(value_column_x);
    ImGui::TextDisabled("%s", ViewerRuntimeStatusLabel(runtime.status));

    ImGui::TextColored(ImVec4(0.35f, 0.78f, 1.0f, 1.0f), kCompletedTransitionsLabel);
    ImGui::SameLine(value_column_x);
    ImGui::Text("%d", runtime.completed_transitions);

    if (runtime.transition_preview.has_value() || active.has_value()) {
        const pmg::InterpolatedTransition* preview =
            runtime.transition_preview.has_value() ? &*runtime.transition_preview : nullptr;
        const float source_phase = active.has_value()
            ? active->source_transition_phase
            : preview->source_transition_phase;
        const float target_phase = active.has_value()
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
        kRuntimeSupportLabel);
    ImGui::SameLine(value_column_x);
    if (active.has_value()) {
        const std::string source_frames =
            FrameList(active->runtime_windows.source.sampled_frames);
        const std::string target_frames =
            FrameList(active->runtime_windows.target.sampled_frames);
        ImGui::Text(
            "%s  src %s  tgt %s",
            pmg::TransitionWindowConventionName(
                active->transition_window_convention),
            source_frames.c_str(), target_frames.c_str());
    } else {
        ImGui::TextDisabled("available during active transition");
    }

    ImGui::TextColored(
        ImVec4(0.35f, 0.78f, 1.0f, 1.0f),
        kAlignmentTransformLabel);
    ImGui::SameLine(value_column_x);
    if (active.has_value()) {
        ImGui::Text(
            "yaw %.3f rad   dx %.3f   dz %.3f",
            active->alignment.yaw,
            active->alignment.dx,
            active->alignment.dz);
    } else {
        ImGui::TextDisabled("waiting for phase gate");
    }

    ImGui::TextColored(
        ImVec4(0.35f, 0.78f, 1.0f, 1.0f),
        kRuntimeBlendWindowLabel);
    ImGui::SameLine(value_column_x);
    if (active.has_value()) {
        ImGui::Text(
            "%d frames   %.3f / %.3f s",
            runtime_.Config().transition_blend_frames,
            active->blend_elapsed_seconds,
            active->blend_duration_seconds);
        ImGui::ProgressBar(
            active->blend_progress, ImVec2(-1.0f, 0.0f), "transitioning");
    } else {
        ImGui::Text(
            "%d frames @ %.1f fps",
            runtime_.Config().transition_blend_frames, graph_fps_);
    }
}

}  // namespace pmgviewer
