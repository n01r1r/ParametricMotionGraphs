#pragma once

#include "pmg/ParametricMotionSpace.h"
#include "pmg/TransitionTypes.h"

#include <optional>
#include <string>
#include <vector>

namespace pmg {

struct PmgNode {
    std::string name;
    ParametricMotionSpace motion_space;
};

// Result of interpolating an edge's stored samples at a query source parameter.
// Source phase identifies the first source support frame; target phase
// identifies the last target support frame. Contains no alignment transform:
// runtime alignment is recomputed from those directional references and the
// generated clips.
struct InterpolatedTransition {
    ParameterAabb target_parameter_box;
    float source_transition_phase = 0.0f;
    float target_transition_phase = 0.0f;
    // kNN-interpolated build-time transition distance D (proxy for the quality
    // of the scheduled transition). Carries TransitionSample::transition_distance
    // through the same source-parameter weighting used for the box and phases.
    float transition_distance = 0.0f;
};

struct PmgEdge {
    int source_node = -1;
    int target_node = -1;
    std::vector<TransitionSample> samples;

    // k-nearest interpolation of the target box and transition phases (PMG paper
    // §4, eqs. 1-3). Uses k = parameter_dimension + 1 nearest source samples
    // and cutoff = distance to the k-th nearest neighbor. Exact matches are
    // handled before reciprocal-weight evaluation. Returns nullopt only for an
    // edge with no sampled transitions.
    std::optional<InterpolatedTransition> LookupInterpolated(
        const ParameterVector& source_parameter,
        const ParameterVector& desired_target_parameter) const;
};

class ParametricMotionGraph {
public:
    int AddNode(std::string node_name, ParametricMotionSpace motion_space);
    int AddEdge(PmgEdge edge);

    const PmgNode& Node(int node_index) const;
    const PmgEdge& Edge(int edge_index) const;

    std::vector<int> OutgoingEdgeIndices(int source_node) const;

    int NumNodes() const;
    int NumEdges() const;

private:
    std::vector<PmgNode> nodes_;
    std::vector<PmgEdge> edges_;
};

}  // namespace pmg
