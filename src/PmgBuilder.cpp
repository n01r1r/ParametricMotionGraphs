#include "pmg/PmgBuilder.h"

#include <random>
#include <stdexcept>
#include <vector>

namespace pmg {

namespace {

// Uniform random samples of a space's parameter axis-aligned range.
std::vector<ParameterVector> RandomParameterSamples(
    const ParametricMotionSpace& space,
    int sample_count,
    std::mt19937& rng) {
    const ParameterVector min_parameter = space.MinParameter();
    const ParameterVector max_parameter = space.MaxParameter();

    std::vector<std::uniform_real_distribution<float>> axis_distributions;
    axis_distributions.reserve(min_parameter.size());
    for (std::size_t dimension = 0; dimension < min_parameter.size(); ++dimension) {
        axis_distributions.emplace_back(min_parameter[dimension], max_parameter[dimension]);
    }

    std::vector<ParameterVector> samples;
    samples.reserve(static_cast<std::size_t>(sample_count));
    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
        ParameterVector parameter(min_parameter.size());
        for (std::size_t dimension = 0; dimension < parameter.size(); ++dimension) {
            parameter[dimension] = axis_distributions[dimension](rng);
        }
        samples.push_back(std::move(parameter));
    }
    return samples;
}

}  // namespace

PmgEdge PmgBuilder::BuildEdge(
    const Skeleton& skeleton,
    int source_node_index,
    int target_node_index,
    const ParametricMotionSpace& source_space,
    const ParametricMotionSpace& target_space,
    const PmgBuilderConfig& config) {
    if (config.source_sample_count <= 0 || config.target_sample_count <= 0) {
        throw std::runtime_error("PmgBuilder::BuildEdge: sample counts must be positive");
    }
    if (config.bad_transition_threshold < config.good_transition_threshold) {
        throw std::runtime_error("PmgBuilder::BuildEdge: bad threshold must be >= good threshold");
    }

    std::mt19937 rng(config.seed);
    const std::vector<ParameterVector> source_samples =
        RandomParameterSamples(source_space, config.source_sample_count, rng);
    const std::vector<ParameterVector> target_samples =
        RandomParameterSamples(target_space, config.target_sample_count, rng);

    // Generate each target motion once; reused across all source samples.
    std::vector<MotionClip> target_clips;
    target_clips.reserve(target_samples.size());
    for (const ParameterVector& target_parameter : target_samples) {
        target_clips.push_back(target_space.GenerateClip(
            target_parameter,
            config.generated_frame_count,
            config.generated_frames_per_second));
    }

    PmgEdge edge;
    edge.source_node = source_node_index;
    edge.target_node = target_node_index;

    // Per-GOOD record. The transition point/alignment must be averaged over only
    // the GOOD samples that remain inside the *adjusted* bounding box (paper
    // §3.2), so each GOOD hit's phases are kept until the box is finalized.
    struct GoodHit {
        ParameterVector parameter;
        float source_phase;
        float target_phase;
        float alignment_yaw;
        float alignment_dx;
        float alignment_dz;
    };

    for (const ParameterVector& source_parameter : source_samples) {
        const MotionClip source_clip = source_space.GenerateClip(
            source_parameter,
            config.generated_frame_count,
            config.generated_frames_per_second);

        std::vector<GoodHit> good_hits;
        std::vector<ParameterVector> bad_target_parameters;

        for (std::size_t target_index = 0; target_index < target_samples.size(); ++target_index) {
            const OptimalTransition transition = MotionDistance::FindOptimalTransition(
                skeleton, source_clip, target_clips[target_index], config.distance_grid);
            const float distance = transition.distance;

            if (distance <= config.good_transition_threshold) {
                good_hits.push_back({target_samples[target_index],
                                     transition.source_phase, transition.target_phase,
                                     transition.alignment.theta, transition.alignment.dx,
                                     transition.alignment.dz});
            } else if (distance >= config.bad_transition_threshold) {
                bad_target_parameters.push_back(target_samples[target_index]);
            }
            // NEUTRAL (between thresholds): not used to build the box; allowed
            // implicitly if it falls inside it (paper §3.2).
        }

        // Paper §3.2 / §6: an edge exists iff *every* source sample can reach some
        // target subspace. A source sample with no GOOD target => empty box =>
        // the whole edge cannot be created.
        if (good_hits.empty()) {
            return PmgEdge{source_node_index, target_node_index, {}};
        }

        ParameterAabb target_box;
        for (const GoodHit& hit : good_hits) {
            target_box.ExpandToInclude(hit.parameter);
        }
        // Conservative double-threshold shrink: push every BAD sample at least
        // epsilon outside the box (paper Fig 4c).
        for (const ParameterVector& bad_parameter : bad_target_parameters) {
            target_box.ShrinkToExclude(bad_parameter, config.box_shrink_epsilon);
        }

        // Average the normalized transition point (and alignment) over only the
        // GOOD samples still inside the adjusted box (paper §3.2).
        int in_box_count = 0;
        float sum_source_phase = 0.0f;
        float sum_target_phase = 0.0f;
        float sum_alignment_yaw = 0.0f;
        float sum_alignment_dx = 0.0f;
        float sum_alignment_dz = 0.0f;
        for (const GoodHit& hit : good_hits) {
            if (!target_box.Contains(hit.parameter)) {
                continue;
            }
            ++in_box_count;
            sum_source_phase += hit.source_phase;
            sum_target_phase += hit.target_phase;
            sum_alignment_yaw += hit.alignment_yaw;
            sum_alignment_dx += hit.alignment_dx;
            sum_alignment_dz += hit.alignment_dz;
        }

        // Adjusted box empty (collapsed by shrink, or no GOOD left inside) =>
        // this source sample cannot transition => no edge (paper §3.2 / §6).
        if (target_box.IsEmpty() || in_box_count == 0) {
            return PmgEdge{source_node_index, target_node_index, {}};
        }

        const float inv_count = 1.0f / static_cast<float>(in_box_count);
        edge.samples.push_back({
            source_parameter,
            target_box,
            sum_source_phase * inv_count,
            sum_target_phase * inv_count,
            sum_alignment_yaw * inv_count,
            sum_alignment_dx * inv_count,
            sum_alignment_dz * inv_count,
        });
    }

    return edge;
}

}  // namespace pmg
