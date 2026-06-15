#pragma once

#include <cstddef>
#include <cmath>
#include <vector>

namespace pmgviewer {

// Authoring-boundary motion-family guard (mirrors core
// RequireBlendableMotionFamily, strict-(a)). A parametric blend space holds one
// motion family: structurally similar, registered examples. Folding an acyclic
// clip (e.g. a vault) in with cyclic locomotion blends raw phases and bleeds the
// odd clip's pose (the walk+vault arm-high jolt). Rule: in a multi-example space
// every example must be cyclic (>=2 foot contacts).
//
// Returns the index of the first example that breaks the premise, or -1 when the
// space is blendable. A single-example space (<2 examples) and the case where
// cyclicity cannot be judged (have_foot_joints == false) both return -1: a lone
// example never blends, and contact-detection failure must never block
// authoring. Structural-equality (equal anchor counts) is deliberately not
// checked: viewer examples are raw clips, not cycle-extracted, so same-family
// clips of differing length legitimately carry differing contact counts.
inline int FirstNonBlendableExample(
    const std::vector<int>& example_contact_counts, bool have_foot_joints) {
    if (example_contact_counts.size() < 2 || !have_foot_joints) {
        return -1;
    }
    for (std::size_t index = 0; index < example_contact_counts.size(); ++index) {
        if (example_contact_counts[index] < 2) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

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
