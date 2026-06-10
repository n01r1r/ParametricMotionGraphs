#include "pmg/ParametricMotionGraph.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace pmg {

namespace {
constexpr float kExactMatchDistance = 1.0e-6f;
}  // namespace

std::optional<InterpolatedTransition> PmgEdge::LookupInterpolated(
    const ParameterVector& source_parameter) const {
    if (samples.empty()) {
        return std::nullopt;
    }

    // Order sample indices by parameter distance to the query.
    std::vector<float> distances(samples.size());
    std::vector<std::size_t> order(samples.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    for (std::size_t i = 0; i < samples.size(); ++i) {
        distances[i] = Distance(source_parameter, samples[i].source_parameter);
    }
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b) { return distances[a] < distances[b]; });

    auto as_result = [](const TransitionSample& sample) {
        InterpolatedTransition result;
        result.target_parameter_box = sample.target_parameter_box;
        result.source_transition_phase = sample.source_transition_phase;
        result.target_transition_phase = sample.target_transition_phase;
        result.alignment_yaw = sample.alignment_yaw;
        result.alignment_dx = sample.alignment_dx;
        result.alignment_dz = sample.alignment_dz;
        return result;
    };

    // Exact match: return that sample directly (weights would degenerate).
    if (distances[order.front()] <= kExactMatchDistance) {
        return as_result(samples[order.front()]);
    }

    const int dimension = static_cast<int>(source_parameter.size());
    const std::size_t neighbor_count =
        std::min<std::size_t>(static_cast<std::size_t>(dimension) + 1, samples.size());

    // Falloff cutoff = distance to the (k+1)-th nearest sample, if one exists.
    const float cutoff_distance = neighbor_count < samples.size()
                                      ? distances[order[neighbor_count]]
                                      : std::numeric_limits<float>::infinity();
    const float inverse_cutoff =
        std::isfinite(cutoff_distance) ? 1.0f / cutoff_distance : 0.0f;

    std::vector<float> weights(neighbor_count, 0.0f);
    float weight_sum = 0.0f;
    for (std::size_t n = 0; n < neighbor_count; ++n) {
        const float distance = distances[order[n]];
        const float weight = 1.0f / distance - inverse_cutoff;
        weights[n] = weight > 0.0f ? weight : 0.0f;
        weight_sum += weights[n];
    }

    // Degenerate weights (e.g. coincident neighbors): fall back to nearest.
    if (weight_sum <= kSmallEpsilon) {
        return as_result(samples[order.front()]);
    }

    InterpolatedTransition result;
    result.target_parameter_box.min_corner.assign(static_cast<std::size_t>(dimension), 0.0f);
    result.target_parameter_box.max_corner.assign(static_cast<std::size_t>(dimension), 0.0f);
    for (std::size_t n = 0; n < neighbor_count; ++n) {
        const float weight = weights[n] / weight_sum;
        const TransitionSample& sample = samples[order[n]];
        for (int d = 0; d < dimension; ++d) {
            result.target_parameter_box.min_corner[d] +=
                weight * sample.target_parameter_box.min_corner[d];
            result.target_parameter_box.max_corner[d] +=
                weight * sample.target_parameter_box.max_corner[d];
        }
        result.source_transition_phase += weight * sample.source_transition_phase;
        result.target_transition_phase += weight * sample.target_transition_phase;
        result.alignment_yaw += weight * sample.alignment_yaw;
        result.alignment_dx += weight * sample.alignment_dx;
        result.alignment_dz += weight * sample.alignment_dz;
    }
    return result;
}

int ParametricMotionGraph::AddNode(std::string node_name, ParametricMotionSpace motion_space) {
    const int node_index = static_cast<int>(nodes_.size());
    nodes_.push_back({std::move(node_name), std::move(motion_space)});
    return node_index;
}

int ParametricMotionGraph::AddEdge(PmgEdge edge) {
    if (edge.source_node < 0 || edge.source_node >= NumNodes()) {
        throw std::runtime_error("ParametricMotionGraph::AddEdge: invalid source node");
    }
    if (edge.target_node < 0 || edge.target_node >= NumNodes()) {
        throw std::runtime_error("ParametricMotionGraph::AddEdge: invalid target node");
    }

    const int edge_index = static_cast<int>(edges_.size());
    edges_.push_back(std::move(edge));
    return edge_index;
}

const PmgNode& ParametricMotionGraph::Node(int node_index) const {
    if (node_index < 0 || node_index >= NumNodes()) {
        throw std::runtime_error("ParametricMotionGraph::Node: invalid node index");
    }
    return nodes_[node_index];
}

const PmgEdge& ParametricMotionGraph::Edge(int edge_index) const {
    if (edge_index < 0 || edge_index >= NumEdges()) {
        throw std::runtime_error("ParametricMotionGraph::Edge: invalid edge index");
    }
    return edges_[edge_index];
}

std::vector<int> ParametricMotionGraph::OutgoingEdgeIndices(int source_node) const {
    if (source_node < 0 || source_node >= NumNodes()) {
        throw std::runtime_error("ParametricMotionGraph::OutgoingEdgeIndices: invalid source node");
    }

    std::vector<int> outgoing_edges;
    for (int edge_index = 0; edge_index < NumEdges(); ++edge_index) {
        if (edges_[edge_index].source_node == source_node) {
            outgoing_edges.push_back(edge_index);
        }
    }
    return outgoing_edges;
}

int ParametricMotionGraph::NumNodes() const {
    return static_cast<int>(nodes_.size());
}

int ParametricMotionGraph::NumEdges() const {
    return static_cast<int>(edges_.size());
}

}  // namespace pmg
