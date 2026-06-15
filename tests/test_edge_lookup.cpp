#include "pmg/ParametricMotionGraph.h"
#include "pmg/TransitionTypes.h"

#include <cassert>
#include <cmath>

namespace {

pmg::TransitionSample MakeSample(float source_param, float box_min, float box_max,
                                 float source_phase, float target_phase) {
    pmg::TransitionSample sample;
    sample.source_parameter = {source_param};
    sample.target_parameter_box.min_corner = {box_min};
    sample.target_parameter_box.max_corner = {box_max};
    sample.source_transition_phase = source_phase;
    sample.target_transition_phase = target_phase;
    return sample;
}

}  // namespace

int main() {
    pmg::PmgEdge edge;
    edge.source_node = 0;
    edge.target_node = 0;
    edge.samples.push_back(MakeSample(0.0f, 0.0f, 1.0f, 0.80f, 0.10f));
    edge.samples.push_back(MakeSample(1.0f, 1.0f, 2.0f, 0.90f, 0.20f));
    edge.samples.push_back(MakeSample(2.0f, 2.0f, 3.0f, 0.70f, 0.30f));
    edge.samples[1].target_phase_samples = {
        {{1.0f}, 0.82f, 0.12f},
        {{2.0f}, 0.94f, 0.28f},
    };
    edge.samples[0].transition_distance = 11.0f;
    edge.samples[1].transition_distance = 22.0f;
    edge.samples[2].transition_distance = 33.0f;

    // Exact match recovers that sample's box, phases, and build-time distance.
    {
        const auto result = edge.LookupInterpolated({1.0f}, {1.0f});
        assert(result.has_value());
        assert(std::abs(result->target_parameter_box.min_corner[0] - 1.0f) < 1.0e-6f);
        assert(std::abs(result->target_parameter_box.max_corner[0] - 2.0f) < 1.0e-6f);
        assert(std::abs(result->source_transition_phase - 0.82f) < 1.0e-6f);
        assert(std::abs(result->target_transition_phase - 0.12f) < 1.0e-6f);
        assert(std::abs(result->transition_distance - 22.0f) < 1.0e-6f);
    }

    // D5: same source parameter, different target parameter -> phase pair
    // measured at that target, not one in-box average.
    {
        const auto result = edge.LookupInterpolated({1.0f}, {2.0f});
        assert(result.has_value());
        assert(std::abs(result->source_transition_phase - 0.94f) < 1.0e-6f);
        assert(std::abs(result->target_transition_phase - 0.28f) < 1.0e-6f);
    }

    // Interior target queries interpolate continuously between retained GOOD
    // phase samples.
    {
        const auto result = edge.LookupInterpolated({1.0f}, {1.5f});
        assert(result.has_value());
        assert(std::abs(result->source_transition_phase - 0.88f) < 1.0e-6f);
        assert(std::abs(result->target_transition_phase - 0.20f) < 1.0e-6f);
    }

    // Paper Eq. (2) k-th cutoff: in 1D, k=2. The second nearest neighbor is the
    // cutoff and receives zero unnormalized weight. A query closer to sample 0
    // therefore reproduces sample 0 rather than using the former k+1 support.
    {
        const auto result = edge.LookupInterpolated({0.25f}, {0.5f});
        assert(result.has_value());
        assert(std::abs(result->target_parameter_box.min_corner[0] - 0.0f) < 1.0e-5f);
        assert(std::abs(result->target_parameter_box.max_corner[0] - 1.0f) < 1.0e-5f);
        assert(std::abs(result->source_transition_phase - 0.80f) < 1.0e-5f);
        assert(std::abs(result->transition_distance - 11.0f) < 1.0e-5f);
        assert(result->target_parameter_box.IsValid());
    }

    // Degenerate equidistant k-th weights fall back to nearest rather than using
    // a k+1 neighbor cutoff.
    {
        const auto result = edge.LookupInterpolated({0.5f}, {0.5f});
        assert(result.has_value());
        assert(std::abs(result->target_parameter_box.min_corner[0] - 0.0f) < 1.0e-5f);
        assert(std::abs(result->target_parameter_box.max_corner[0] - 1.0f) < 1.0e-5f);
    }

    // Duplicate exact samples are averaged explicitly.
    {
        pmg::PmgEdge duplicate_edge;
        duplicate_edge.source_node = 0;
        duplicate_edge.target_node = 0;
        duplicate_edge.samples.push_back(MakeSample(0.0f, 0.0f, 1.0f, 0.80f, 0.10f));
        duplicate_edge.samples.push_back(MakeSample(0.0f, 2.0f, 4.0f, 0.60f, 0.30f));
        duplicate_edge.samples[0].target_parameter_box.min_corner = {0.0f, 10.0f};
        duplicate_edge.samples[0].target_parameter_box.max_corner = {1.0f, 11.0f};
        duplicate_edge.samples[1].target_parameter_box.min_corner = {2.0f, 20.0f};
        duplicate_edge.samples[1].target_parameter_box.max_corner = {4.0f, 40.0f};
        const auto result =
            duplicate_edge.LookupInterpolated({0.0f}, {1.0f, 15.0f});
        assert(result.has_value());
        assert(std::abs(result->target_parameter_box.min_corner[0] - 1.0f) < 1.0e-5f);
        assert(std::abs(result->target_parameter_box.max_corner[0] - 2.5f) < 1.0e-5f);
        assert(std::abs(result->target_parameter_box.min_corner[1] - 15.0f) < 1.0e-5f);
        assert(std::abs(result->target_parameter_box.max_corner[1] - 25.5f) < 1.0e-5f);
        assert(std::abs(result->source_transition_phase - 0.70f) < 1.0e-5f);
    }

    // Empty edge -> no interpolation.
    {
        const pmg::PmgEdge empty_edge;
        assert(!empty_edge.LookupInterpolated({0.0f}, {0.0f}).has_value());
    }

    return 0;
}
