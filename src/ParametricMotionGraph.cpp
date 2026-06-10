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

InterpolatedTransition AverageSamples(
    const std::vector<const TransitionSample*>& selected_samples,
    const std::vector<float>& weights) {
    if (selected_samples.empty() || selected_samples.size() != weights.size()) {
        throw std::runtime_error("AverageSamples: invalid sample/weight count");
    }

    const int dimension = static_cast<int>(selected_samples.front()->source_parameter.size());
    InterpolatedTransition result;
    result.target_parameter_box.min_corner.assign(static_cast<std::size_t>(dimension), 0.0f);
    result.target_parameter_box.max_corner.assign(static_cast<std::size_t>(dimension), 0.0f);

    for (std::size_t sample_index = 0; sample_index < selected_samples.size(); ++sample_index) {
        const TransitionSample& sample = *selected_samples[sample_index];
        const float weight = weights[sample_index];
        for (int d = 0; d < dimension; ++d) {
            result.target_parameter_box.min_corner[d] +=
                weight * sample.target_parameter_box.min_corner[d];
            result.target_parameter_box.max_corner[d] +=
                weight * sample.target_parameter_box.max_corner[d];
        }
        result.source_transition_phase += weight * sample.source_transition_phase;
        result.target_transition_phase += weight * sample.target_transition_phase;
    }
    return result;
}

InterpolatedTransition AsResult(const TransitionSample& sample) {
    InterpolatedTransition result;
    result.target_parameter_box = sample.target_parameter_box;
    result.source_transition_phase = sample.source_transition_phase;
    result.target_transition_phase = sample.target_transition_phase;
    return result;
}
}  // namespace

std::optional<InterpolatedTransition> PmgEdge::LookupInterpolated(
    const ParameterVector& source_parameter) const {
    if (samples.empty()) {
        return std::nullopt;
    }

    std::vector<float> distances(samples.size());
    std::vector<std::size_t> order(samples.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    for (std::size_t i = 0; i < samples.size(); ++i) {
        distances[i] = Distance(source_parameter, samples[i].source_parameter);
    }
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b) { return distances[a] < distances[b]; });

    // Exact matches: if duplicated source samples exist, average all exact
    // matches uniformly rather than arbitrarily picking one.
    std::vector<const TransitionSample*> exact_samples;
    for (const std::size_t sample_index : order) {
        if (distances[sample_index] > kExactMatchDistance) {
            break;
        }
        exact_samples.push_back(&samples[sample_index]);
    }
    if (!exact_samples.empty()) {
        if (exact_samples.size() == 1) {
            return AsResult(*exact_samples.front());
        }
        const float inv_count = 1.0f / static_cast<float>(exact_samples.size());
        return AverageSamples(exact_samples, std::vector<float>(exact_samples.size(), inv_count));
    }

    const int dimension = static_cast<int>(source_parameter.size());
    const std::size_t neighbor_count =
        std::min<std::size_t>(static_cast<std::size_t>(dimension) + 1, samples.size());

    // PMG Eq. (2): cutoff is the k-th nearest neighbor, not k+1. The farthest
    // selected neighbor therefore receives zero unnormalized weight.
    const float cutoff_distance = distances[order[neighbor_count - 1]];
    if (cutoff_distance <= kExactMatchDistance || !std::isfinite(cutoff_distance)) {
        return AsResult(samples[order.front()]);
    }
    const float inverse_cutoff = 1.0f / cutoff_distance;

    std::vector<float> weights(neighbor_count, 0.0f);
    float weight_sum = 0.0f;
    for (std::size_t n = 0; n < neighbor_count; ++n) {
        const float distance = distances[order[n]];
        if (distance <= kExactMatchDistance) {
            // Should have been caught above, but keep the singularity guarded.
            return AsResult(samples[order[n]]);
        }
        const float weight = 1.0f / distance - inverse_cutoff;
        weights[n] = weight > 0.0f ? weight : 0.0f;
        weight_sum += weights[n];
    }

    if (weight_sum <= kSmallEpsilon) {
        return AsResult(samples[order.front()]);
    }

    std::vector<const TransitionSample*> selected_samples;
    selected_samples.reserve(neighbor_count);
    for (std::size_t n = 0; n < neighbor_count; ++n) {
        selected_samples.push_back(&samples[order[n]]);
        weights[n] /= weight_sum;
    }
    return AverageSamples(selected_samples, weights);
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
    if (edge.samples.empty()) {
        throw std::runtime_error("ParametricMotionGraph::AddEdge: empty edge is not a valid PMG edge");
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
