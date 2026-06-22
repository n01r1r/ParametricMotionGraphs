#pragma once

#include "pmg/PmgArtifact.h"
#include "pmg/TransitionQuality.h"

#include <string>
#include <map>
#include <optional>
#include <vector>

namespace pmg {

// Exact, renderer-independent diagnostic request. Parameters use node-space
// axis units. frames_before/after are positive frame counts around transition.
struct TransitionProbeRequest {
    std::string source_node;
    std::string target_node;
    ParameterVector source_parameter;
    ParameterVector requested_target_parameter;
    int frames_before = 3;
    int frames_after = 3;
    TransitionQualityGateConfig quality_gate{
        true, 1.5f, 50.0f, 2.0f, false, 2.0f};
};

struct TransitionProbeResult {
    ParameterVector source_parameter;
    ParameterVector requested_target_parameter;
    ParameterVector effective_target_parameter;
    ParameterAabb target_box;
    bool accepted_by_box = false;
    OptimalTransition transition;
    std::string metric_class;
    float root_jump = 0.0f;
    float heading_jump = 0.0f;
    float velocity_jump = 0.0f;
    TransitionQualityRecord quality;
    TransitionQualityGateDecision quality_decision;
    bool final_accepted = false;
    std::string reject_reason;
};

struct TransitionAcceptanceAuditRow {
    ParameterVector source_parameter;
    ParameterVector requested_target_parameter;
    ParameterVector effective_target_parameter;
    ParameterAabb interpolated_target_box;
    bool accepted_by_box = false;
    float transition_distance = 0.0f;
    std::string metric_class;
    bool acceptance_violation = false;
    float distance_over_tbad = 0.0f;
    bool source_coverage_available = false;
    float source_coverage = 0.0f;
    float source_phase = 0.0f;
    int source_frame = 0;
    float target_phase = 0.0f;
    int target_frame = 0;
    float root_jump = 0.0f;
    float heading_jump = 0.0f;
    float velocity_jump = 0.0f;
    std::optional<TransitionQualityRecord> quality;
    bool quality_gate_accepts = false;
    std::string quality_reject_reason = "not_evaluated";
    std::string nearest_source_anchor_label;
    std::string nearest_target_anchor_label;
    std::string nearest_good_evidence;
    std::string nearest_bad_evidence;
    std::string overreach_origin;
    bool jog_walk_pair = false;
    std::string notes;
};

struct TransitionAcceptanceAuditSummary {
    std::string source_node;
    std::string target_node;
    PmgBuilderConfig config;
    std::vector<TransitionAcceptanceAuditRow> rows;
    int accepted_bad_count = 0;
    int near_threshold_accepted_count = 0;
    int accepted_bad_after_quality_gate = 0;
    int accepted_near_threshold_after_quality_gate = 0;
    int box_accepted_quality_rejected = 0;
    std::map<std::string, int> quality_reject_reasons;
    int jog_walk_row_count = 0;
    int shrinkage_row_count = 0;
    bool known_bad_case_found = false;
    std::string consistency_conclusion;
    std::string root_cause_conclusion;
};

// Resolves edge lookup and target projection, measures exact transition
// diagnostics, then applies the optional CLI-only quality gate. Does not alter
// RuntimeController or persisted graph data.
TransitionProbeResult ProbeTransition(
    const BuiltPmgArtifact& artifact,
    const TransitionProbeRequest& request);

TransitionQualityRecord MeasureExactTransitionQuality(
    const BuiltPmgArtifact& artifact,
    const PmgNode& source_node,
    const PmgNode& target_node,
    const PmgBuilderConfig& config,
    const ParameterVector& source_parameter,
    const ParameterVector& effective_target_parameter,
    const OptimalTransition& transition,
    int frames_before,
    int frames_after);

const char* TransitionMetricClass(
    float distance,
    const PmgBuilderConfig& config);

}  // namespace pmg
