#include "pmg/PmgBuilder.h"

#include <cmath>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace pmg {

namespace {

std::vector<ParameterVector> RandomParameterSamples(
    const ParametricMotionSpace& space,
    int sample_count,
    std::mt19937& rng) {
    std::vector<ParameterVector> samples;
    samples.reserve(static_cast<std::size_t>(sample_count));
    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
        samples.push_back(space.SampleSupportedParameter(rng));
    }
    return samples;
}

bool SameParameterVector(const ParameterVector& a, const ParameterVector& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > 1.0e-6f) {
            return false;
        }
    }
    return true;
}

void AppendUniqueParameter(std::vector<ParameterVector>& samples,
                           const ParameterVector& candidate) {
    for (const ParameterVector& existing : samples) {
        if (SameParameterVector(existing, candidate)) {
            return;
        }
    }
    samples.push_back(candidate);
}

std::vector<ParameterVector> BuildParameterSamples(
    const ParametricMotionSpace& space,
    int random_sample_count,
    bool include_example_parameters,
    std::mt19937& rng) {
    std::vector<ParameterVector> samples;
    if (include_example_parameters) {
        for (const ParameterVector& example_parameter : space.ExampleParameters()) {
            AppendUniqueParameter(samples, example_parameter);
        }
    }
    for (const ParameterVector& random_parameter :
         RandomParameterSamples(space, random_sample_count, rng)) {
        AppendUniqueParameter(samples, random_parameter);
    }
    return samples;
}

float PercentileSorted(const std::vector<float>& sorted, float percentile) {
    if (sorted.empty()) {
        return std::numeric_limits<float>::infinity();
    }
    const float clamped = std::clamp(percentile, 0.0f, 1.0f);
    const float position = clamped * static_cast<float>(sorted.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    if (lower == upper) {
        return sorted[lower];
    }
    const float alpha = position - static_cast<float>(lower);
    return (1.0f - alpha) * sorted[lower] + alpha * sorted[upper];
}

OptimalTransition FindConfiguredTransition(
    const Skeleton& skeleton,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const DistanceGridConfig& edge_grid,
    const PmgBuilderConfig& config) {
    switch (config.transition_metric_type) {
        case TransitionMetricType::kKovarDirectionalPointCloud:
            return MotionDistance::FindOptimalTransitionForConvention(
                skeleton, source_clip, target_clip, edge_grid,
                config.transition_convention);
        case TransitionMetricType::kDynamicsWindow:
            return MotionDistance::FindOptimalDynamicsTransitionForConvention(
                skeleton, source_clip, target_clip, edge_grid,
                config.transition_metric, config.transition_convention);
    }
    throw std::runtime_error("PmgBuilder::BuildEdge: unknown transition metric type");
}

}  // namespace

const char* TransitionMetricTypeName(TransitionMetricType type) {
    switch (type) {
        case TransitionMetricType::kKovarDirectionalPointCloud:
            return "kovar_directional_point_cloud";
        case TransitionMetricType::kDynamicsWindow:
            return "dynamics_window";
    }
    throw std::runtime_error("TransitionMetricTypeName: unknown transition metric type");
}

TransitionMetricType ParseTransitionMetricType(const std::string& name) {
    if (name == "kovar" || name == "kovar_directional_point_cloud") {
        return TransitionMetricType::kKovarDirectionalPointCloud;
    }
    if (name == "dynamics" || name == "dynamics_window") {
        return TransitionMetricType::kDynamicsWindow;
    }
    throw std::runtime_error(
        "ParseTransitionMetricType: unknown transition metric '" + name + "'");
}

PmgEdge PmgBuilder::BuildEdge(
    const Skeleton& skeleton,
    int source_node_index,
    int target_node_index,
    const ParametricMotionSpace& source_space,
    const ParametricMotionSpace& target_space,
    const PmgBuilderConfig& config) {
    return BuildEdgeWithReport(
        skeleton, source_node_index, target_node_index, source_space, target_space, config).edge;
}

EdgeBuildResult PmgBuilder::BuildEdgeWithReport(
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

    EdgeBuildResult result;
    result.edge.source_node = source_node_index;
    result.edge.target_node = target_node_index;
    result.report.source_node = source_node_index;
    result.report.target_node = target_node_index;

    // A self-edge concatenates a cyclic clip end onto its own start, so (when the
    // ablation is enabled) its metric windows wrap across the cycle boundary
    // instead of clamping at the endpoints. Cross-node edges keep clamped windows.
    DistanceGridConfig edge_grid = config.distance_grid;
    if (config.self_edge_cyclic_metric && source_node_index == target_node_index) {
        edge_grid.cyclic_wrap = true;
    }

    std::mt19937 rng(config.seed);
    const std::vector<ParameterVector> source_samples = BuildParameterSamples(
        source_space, config.source_sample_count, config.include_example_parameters, rng);
    const std::vector<ParameterVector> target_samples = BuildParameterSamples(
        target_space, config.target_sample_count, config.include_example_parameters, rng);

    std::vector<MotionClip> target_clips;
    target_clips.reserve(target_samples.size());
    for (const ParameterVector& target_parameter : target_samples) {
        target_clips.push_back(target_space.GenerateClip(
            target_parameter, config.generated_frames_per_second));
    }

    struct GoodHit {
        ParameterVector parameter;
        float source_phase;
        float target_phase;
        float distance;
    };

    for (const ParameterVector& source_parameter : source_samples) {
        SourceSampleBuildReport sample_report;
        sample_report.source_parameter = source_parameter;
        sample_report.target_box_before_shrink =
            ParameterAabb::Empty(target_space.ParameterDimension());
        sample_report.target_box_after_shrink =
            ParameterAabb::Empty(target_space.ParameterDimension());

        const MotionClip source_clip = source_space.GenerateClip(
            source_parameter, config.generated_frames_per_second);

        std::vector<GoodHit> good_hits;
        std::vector<ParameterVector> bad_target_parameters;
        std::vector<float> distances;
        distances.reserve(target_samples.size());

        for (std::size_t target_index = 0; target_index < target_samples.size(); ++target_index) {
            const OptimalTransition transition =
                FindConfiguredTransition(
                    skeleton, source_clip, target_clips[target_index],
                    edge_grid, config);
            const float distance = transition.distance;
            distances.push_back(distance);

            if (distance <= config.good_transition_threshold) {
                ++sample_report.good_count;
                good_hits.push_back({target_samples[target_index],
                                     transition.source_phase, transition.target_phase,
                                     distance});
            } else if (distance >= config.bad_transition_threshold) {
                ++sample_report.bad_count;
                bad_target_parameters.push_back(target_samples[target_index]);
            } else {
                ++sample_report.neutral_count;
            }
        }

        std::sort(distances.begin(), distances.end());
        if (!distances.empty()) {
            sample_report.min_distance = distances.front();
            sample_report.p25_distance = PercentileSorted(distances, 0.25f);
            sample_report.median_distance = PercentileSorted(distances, 0.50f);
            sample_report.max_distance = distances.back();
        }

        if (good_hits.empty()) {
            sample_report.accepted = false;
            sample_report.reject_reason = "no GOOD target samples";
            result.report.source_reports.push_back(sample_report);
            if (config.restrict_source_range) {
                // Paper Sec 6 (default): this source parameter cannot reach the
                // target, so drop it and keep building from the compatible source
                // samples instead of rejecting the whole edge.
                continue;
            }
            result.report.edge_created = false;
            // Legacy all-or-nothing ABLATION (restrict_source_range=false): an edge
            // requires every source sample to reach the target. Surface the
            // achievable distance vs threshold so the cause (incompatible spaces vs
            // a too-low threshold) is self-evident instead of the bare "no GOOD
            // target samples".
            result.report.reject_reason =
                "a source sample cannot transition to the target space (best "
                "distance " + std::to_string(sample_report.min_distance) +
                " > TGOOD " +
                std::to_string(config.good_transition_threshold) +
                "): spaces not transition-compatible over the whole source "
                "range (paper Sec 6) -- re-enable source-range restriction (on by "
                "default), pick compatible clips, or raise TGOOD";
            return result;
        }

        ParameterAabb target_box;
        for (const GoodHit& hit : good_hits) {
            target_box.ExpandToInclude(hit.parameter);
        }
        sample_report.target_box_before_shrink = target_box;

        for (const ParameterVector& bad_parameter : bad_target_parameters) {
            target_box.ShrinkToExclude(bad_parameter, config.box_shrink_epsilon);
        }
        sample_report.target_box_after_shrink = target_box;

        bool skip_source_sample = false;
        if (!target_box.IsEmpty()) {
            for (const ParameterVector& bad_parameter : bad_target_parameters) {
                if (target_box.Contains(bad_parameter)) {
                    sample_report.accepted = false;
                    sample_report.reject_reason = "BAD target sample remained inside adjusted box";
                    result.report.source_reports.push_back(sample_report);
                    if (config.restrict_source_range) {
                        // Paper Sec 6 (default): drop this source sample, keep rest.
                        skip_source_sample = true;
                        break;
                    }
                    result.report.edge_created = false;
                    result.report.reject_reason = "BAD target sample remained inside adjusted box";
                    return result;
                }
            }
        }
        if (skip_source_sample) {
            continue;
        }

        int in_box_count = 0;
        float sum_source_phase = 0.0f;
        float sum_target_phase = 0.0f;
        float sum_distance = 0.0f;
        for (const GoodHit& hit : good_hits) {
            if (!target_box.Contains(hit.parameter)) {
                continue;
            }
            ++in_box_count;
            sum_source_phase += hit.source_phase;
            sum_target_phase += hit.target_phase;
            sum_distance += hit.distance;
        }

        if (target_box.IsEmpty() || in_box_count == 0) {
            sample_report.accepted = false;
            sample_report.reject_reason = target_box.IsEmpty()
                                              ? "adjusted target box is empty"
                                              : "no GOOD samples remain inside adjusted box";
            result.report.source_reports.push_back(sample_report);
            if (config.restrict_source_range) {
                // Paper Sec 6 (default): drop this source sample, keep rest.
                continue;
            }
            result.report.edge_created = false;
            result.report.reject_reason = sample_report.reject_reason;
            return result;
        }

        const float inv_count = 1.0f / static_cast<float>(in_box_count);
        TransitionSample transition_sample;
        transition_sample.source_parameter = source_parameter;
        transition_sample.target_parameter_box = target_box;
        transition_sample.source_transition_phase =
            sum_source_phase * inv_count;
        transition_sample.target_transition_phase =
            sum_target_phase * inv_count;
        transition_sample.transition_distance = sum_distance * inv_count;
        transition_sample.target_phase_samples.reserve(
            static_cast<std::size_t>(in_box_count));
        for (const GoodHit& hit : good_hits) {
            if (!target_box.Contains(hit.parameter)) {
                continue;
            }
            transition_sample.target_phase_samples.push_back({
                hit.parameter,
                hit.source_phase,
                hit.target_phase,
            });
        }
        result.edge.samples.push_back(std::move(transition_sample));

        sample_report.accepted = true;
        result.report.source_reports.push_back(sample_report);
    }

    result.report.edge_created = !result.edge.samples.empty();
    if (!result.report.edge_created) {
        result.report.reject_reason = config.restrict_source_range
            ? "no source sample could transition to the target (paper Sec 6 "
              "source-range restriction left no compatible source samples)"
            : "edge has no transition samples";
    }
    return result;
}

}  // namespace pmg
