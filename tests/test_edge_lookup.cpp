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
    edge.samples.push_back(MakeSample(0.0f, 0.0f, 1.0f, 0.80f, 0.10f));
    edge.samples.push_back(MakeSample(1.0f, 1.0f, 2.0f, 0.90f, 0.20f));
    edge.samples.push_back(MakeSample(2.0f, 2.0f, 3.0f, 0.70f, 0.30f));

    // Exact match recovers that sample's box and phases.
    {
        const auto result = edge.LookupInterpolated({1.0f});
        assert(result.has_value());
        assert(std::abs(result->target_parameter_box.min_corner[0] - 1.0f) < 1.0e-6f);
        assert(std::abs(result->target_parameter_box.max_corner[0] - 2.0f) < 1.0e-6f);
        assert(std::abs(result->source_transition_phase - 0.90f) < 1.0e-6f);
    }

    // Midpoint query: equal weights on samples 0 and 1 (cutoff = sample 2).
    // Box = 0.5*[0,1] + 0.5*[1,2] = [0.5, 1.5]; phases = 0.5*(0.80,0.10)+0.5*(0.90,0.20).
    {
        const auto result = edge.LookupInterpolated({0.5f});
        assert(result.has_value());
        assert(std::abs(result->target_parameter_box.min_corner[0] - 0.5f) < 1.0e-5f);
        assert(std::abs(result->target_parameter_box.max_corner[0] - 1.5f) < 1.0e-5f);
        assert(std::abs(result->source_transition_phase - 0.85f) < 1.0e-5f);
        assert(std::abs(result->target_transition_phase - 0.15f) < 1.0e-5f);
        assert(result->target_parameter_box.IsValid());
    }

    // Interpolated box lies strictly between the two bracketing sample boxes.
    {
        const auto result = edge.LookupInterpolated({0.25f});
        assert(result.has_value());
        const float min_corner = result->target_parameter_box.min_corner[0];
        const float max_corner = result->target_parameter_box.max_corner[0];
        assert(min_corner > 0.0f && min_corner < 1.0f);
        assert(max_corner > 1.0f && max_corner < 2.0f);
        assert(result->target_parameter_box.IsValid());
    }

    // Empty edge -> no interpolation.
    {
        const pmg::PmgEdge empty_edge;
        assert(!empty_edge.LookupInterpolated({0.0f}).has_value());
    }

    return 0;
}
