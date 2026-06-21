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

struct TransitionPhases {
    float source = 0.0f;
    float target = 0.0f;
};

TransitionPhases ResolveTargetPhases(
    const TransitionSample& sample,
    const ParameterVector& desired_target_parameter) {
    if (sample.target_phase_samples.empty()) {
        return {
            sample.source_transition_phase,
            sample.target_transition_phase,
        };
    }

    const ParameterVector target_parameter =
        sample.target_parameter_box.Clamp(desired_target_parameter);
    std::vector<float> distances(sample.target_phase_samples.size());
    std::vector<std::size_t> order(sample.target_phase_samples.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    for (std::size_t index = 0;
         index < sample.target_phase_samples.size(); ++index) {
        distances[index] = Distance(
            target_parameter,
            sample.target_phase_samples[index].target_parameter);
    }
    std::sort(order.begin(), order.end(),
              [&](std::size_t left, std::size_t right) {
                  return distances[left] < distances[right];
              });

    std::vector<const TargetTransitionPhaseSample*> exact_samples;
    for (const std::size_t sample_index : order) {
        if (distances[sample_index] > kExactMatchDistance) {
            break;
        }
        exact_samples.push_back(
            &sample.target_phase_samples[sample_index]);
    }
    if (!exact_samples.empty()) {
        TransitionPhases phases;
        const float weight =
            1.0f / static_cast<float>(exact_samples.size());
        for (const TargetTransitionPhaseSample* phase_sample :
             exact_samples) {
            phases.source +=
                weight * phase_sample->source_transition_phase;
            phases.target +=
                weight * phase_sample->target_transition_phase;
        }
        return phases;
    }

    const std::size_t neighbor_count = std::min<std::size_t>(
        target_parameter.size() + 1, sample.target_phase_samples.size());
    std::vector<float> weights(neighbor_count, 0.0f);
    float weight_sum = 0.0f;
    for (std::size_t neighbor = 0;
         neighbor < neighbor_count; ++neighbor) {
        const float distance = distances[order[neighbor]];
        if (!std::isfinite(distance)) {
            continue;
        }
        weights[neighbor] = 1.0f / distance;
        weight_sum += weights[neighbor];
    }
    if (weight_sum <= kSmallEpsilon) {
        const TargetTransitionPhaseSample& nearest =
            sample.target_phase_samples[order.front()];
        return {
            nearest.source_transition_phase,
            nearest.target_transition_phase,
        };
    }

    TransitionPhases phases;
    for (std::size_t neighbor = 0;
         neighbor < neighbor_count; ++neighbor) {
        const float weight = weights[neighbor] / weight_sum;
        const TargetTransitionPhaseSample& phase_sample =
            sample.target_phase_samples[order[neighbor]];
        phases.source +=
            weight * phase_sample.source_transition_phase;
        phases.target +=
            weight * phase_sample.target_transition_phase;
    }
    return phases;
}

std::optional<InterpolatedTransition> AverageSamples(
    const std::vector<const TransitionSample*>& selected_samples,
    const std::vector<float>& weights,
    const ParameterVector& desired_target_parameter) {
    if (selected_samples.empty() || selected_samples.size() != weights.size()) {
        throw std::runtime_error("AverageSamples: invalid sample/weight count");
    }

    const int target_dimension = static_cast<int>(
        selected_samples.front()->target_parameter_box.min_corner.size());
    InterpolatedTransition result;
    result.target_parameter_box = ParameterAabb::Empty(target_dimension);
    bool has_target_box = false;

    for (std::size_t sample_index = 0; sample_index < selected_samples.size(); ++sample_index) {
        const TransitionSample& sample = *selected_samples[sample_index];
        const float weight = weights[sample_index];
        if (weight <= 0.0f) {
            continue;
        }
        if (!has_target_box) {
            result.target_parameter_box = sample.target_parameter_box;
            has_target_box = true;
            continue;
        }
        for (int dimension = 0; dimension < target_dimension; ++dimension) {
            result.target_parameter_box.min_corner[dimension] = std::max(
                result.target_parameter_box.min_corner[dimension],
                sample.target_parameter_box.min_corner[dimension]);
            result.target_parameter_box.max_corner[dimension] = std::min(
                result.target_parameter_box.max_corner[dimension],
                sample.target_parameter_box.max_corner[dimension]);
        }
    }

    // ponytail: intersect existing sampled boxes; add a target-region hull only
    // if measured coverage loss makes this conservative policy inadequate.
    if (!has_target_box || result.target_parameter_box.IsEmpty()) {
        return std::nullopt;
    }

    const ParameterVector target_parameter =
        result.target_parameter_box.Clamp(desired_target_parameter);
    for (std::size_t sample_index = 0;
         sample_index < selected_samples.size(); ++sample_index) {
        const TransitionPhases phases = ResolveTargetPhases(
            *selected_samples[sample_index], target_parameter);
        result.source_transition_phase +=
            weights[sample_index] * phases.source;
        result.target_transition_phase +=
            weights[sample_index] * phases.target;
        result.transition_distance +=
            weights[sample_index] *
            selected_samples[sample_index]->transition_distance;
    }
    return result;
}

InterpolatedTransition AsResult(
    const TransitionSample& sample,
    const ParameterVector& desired_target_parameter) {
    InterpolatedTransition result;
    result.target_parameter_box = sample.target_parameter_box;
    const TransitionPhases phases =
        ResolveTargetPhases(sample, desired_target_parameter);
    result.source_transition_phase = phases.source;
    result.target_transition_phase = phases.target;
    result.transition_distance = sample.transition_distance;
    return result;
}
}  // namespace

std::optional<InterpolatedTransition> PmgEdge::LookupInterpolated(
    const ParameterVector& source_parameter,
    const ParameterVector& desired_target_parameter) const {
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
            return AsResult(
                *exact_samples.front(), desired_target_parameter);
        }
        const float inv_count = 1.0f / static_cast<float>(exact_samples.size());
        return AverageSamples(
            exact_samples,
            std::vector<float>(exact_samples.size(), inv_count),
            desired_target_parameter);
    }

    const int dimension = static_cast<int>(source_parameter.size());
    const std::size_t neighbor_count =
        std::min<std::size_t>(static_cast<std::size_t>(dimension) + 1, samples.size());

    // PMG Eq. (2): cutoff is the k-th nearest neighbor, not k+1. The farthest
    // selected neighbor therefore receives zero unnormalized weight.
    const float cutoff_distance = distances[order[neighbor_count - 1]];
    if (cutoff_distance <= kExactMatchDistance || !std::isfinite(cutoff_distance)) {
        return AsResult(
            samples[order.front()], desired_target_parameter);
    }
    const float inverse_cutoff = 1.0f / cutoff_distance;

    std::vector<float> weights(neighbor_count, 0.0f);
    float weight_sum = 0.0f;
    for (std::size_t n = 0; n < neighbor_count; ++n) {
        const float distance = distances[order[n]];
        if (distance <= kExactMatchDistance) {
            // Should have been caught above, but keep the singularity guarded.
            return AsResult(
                samples[order[n]], desired_target_parameter);
        }
        const float weight = 1.0f / distance - inverse_cutoff;
        weights[n] = weight > 0.0f ? weight : 0.0f;
        weight_sum += weights[n];
    }

    if (weight_sum <= kSmallEpsilon) {
        return AsResult(
            samples[order.front()], desired_target_parameter);
    }

    std::vector<const TransitionSample*> selected_samples;
    selected_samples.reserve(neighbor_count);
    for (std::size_t n = 0; n < neighbor_count; ++n) {
        selected_samples.push_back(&samples[order[n]]);
        weights[n] /= weight_sum;
    }
    return AverageSamples(
        selected_samples, weights, desired_target_parameter);
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
