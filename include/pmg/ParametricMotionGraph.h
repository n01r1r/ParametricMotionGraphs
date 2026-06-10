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

// Result of interpolating an edge's stored samples at a query parameter.
struct InterpolatedTransition {
    ParameterAabb target_parameter_box;
    float source_transition_phase = 0.0f;
    float target_transition_phase = 0.0f;
    float alignment_yaw = 0.0f;  // target->source yaw about +Y (radians)
    float alignment_dx = 0.0f;
    float alignment_dz = 0.0f;
};

struct PmgEdge {
    int source_node = -1;
    int target_node = -1;
    std::vector<TransitionSample> samples;

    // k-nearest interpolation of the target box + transition phases (paper §4,
    // eqs 1-3). Uses k = ParameterDimension + 1 nearest source samples with
    // Allen 2002 weights w'_i = 1/d_i - 1/d_cutoff (cutoff = the (k+1)-th
    // nearest distance, or +inf when fewer samples exist), normalized to sum 1.
    // An exact parameter match returns that sample directly. Returns nullopt
    // only for an empty (no-edge) edge.
    std::optional<InterpolatedTransition> LookupInterpolated(
        const ParameterVector& source_parameter) const;
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
