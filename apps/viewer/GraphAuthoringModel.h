#pragma once

#include <cmath>
#include <vector>

namespace pmgviewer {

inline constexpr float kDefaultGoodTransitionThreshold = 80.0f;
inline constexpr float kDefaultBadTransitionThreshold = 110.0f;

// Directed edge description used by viewer-side graph authoring.
// Thresholds are raw weighted squared-sum units. Defaults preserve the prior
// mean-squared scale for the repository's 31-joint, 5-frame baseline.
struct AuthoredEdge {
    int source_node = 0;
    int target_node = 0;
    float tgood = kDefaultGoodTransitionThreshold;
    float tbad = kDefaultBadTransitionThreshold;
};

enum class AddAuthoredEdgeResult {
    Added,
    Duplicate,
    InvalidNode,
    InvalidThreshold,
};

// Adds one directed edge when both endpoints and thresholds are valid.
// One edge per ordered node pair keeps runtime target selection unambiguous.
inline AddAuthoredEdgeResult AddAuthoredEdge(
    int node_count, int source_node, int target_node, float tgood, float tbad,
    std::vector<AuthoredEdge>& edges) {
    if (source_node < 0 || source_node >= node_count || target_node < 0 ||
        target_node >= node_count) {
        return AddAuthoredEdgeResult::InvalidNode;
    }
    if (!std::isfinite(tgood) || !std::isfinite(tbad) || tgood < 0.0f ||
        tbad < tgood) {
        return AddAuthoredEdgeResult::InvalidThreshold;
    }
    for (const AuthoredEdge& edge : edges) {
        if (edge.source_node == source_node &&
            edge.target_node == target_node) {
            return AddAuthoredEdgeResult::Duplicate;
        }
    }
    edges.push_back(AuthoredEdge{source_node, target_node, tgood, tbad});
    return AddAuthoredEdgeResult::Added;
}

}  // namespace pmgviewer
