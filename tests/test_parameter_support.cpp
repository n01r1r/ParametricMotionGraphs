#include "pmg/ParameterSupport.h"

#include <cassert>
#include <cmath>
#include <random>
#include <stdexcept>

namespace {

void AssertNear(float actual, float expected) {
    assert(std::abs(actual - expected) < 1.0e-5f);
}

void AssertVectorNear(
    const pmg::ParameterVector& actual,
    const pmg::ParameterVector& expected) {
    assert(actual.size() == expected.size());
    for (std::size_t index = 0; index < actual.size(); ++index) {
        AssertNear(actual[index], expected[index]);
    }
}

}  // namespace

int main() {
    const pmg::ParameterSupport triangle({
        {0.0f, 0.0f},
        {1.0f, 0.0f},
        {0.0f, 1.0f},
    });

    assert(triangle.Dimension() == 2);
    assert(triangle.NumVertices() == 3);

    const std::vector<float> weights =
        triangle.BarycentricWeights({0.25f, 0.5f});
    AssertNear(weights[0], 0.25f);
    AssertNear(weights[1], 0.25f);
    AssertNear(weights[2], 0.5f);
    assert(triangle.Contains({0.25f, 0.5f}));
    assert(!triangle.Contains({0.75f, 0.75f}));

    AssertVectorNear(triangle.Project({0.25f, 0.5f}), {0.25f, 0.5f});
    AssertVectorNear(triangle.Project({0.75f, 0.75f}), {0.5f, 0.5f});
    AssertVectorNear(triangle.Project({-0.2f, 0.4f}), {0.0f, 0.4f});

    std::mt19937 first_rng(7);
    std::mt19937 second_rng(7);
    const pmg::ParameterVector first_sample = triangle.SampleUniform(first_rng);
    const pmg::ParameterVector second_sample = triangle.SampleUniform(second_rng);
    AssertVectorNear(first_sample, second_sample);
    assert(triangle.Contains(first_sample, 1.0e-4f));

    bool invalid_vertex_count_threw = false;
    try {
        (void)pmg::ParameterSupport({{0.0f, 0.0f}, {1.0f, 0.0f}});
    } catch (const std::runtime_error&) {
        invalid_vertex_count_threw = true;
    }
    assert(invalid_vertex_count_threw);

    bool degenerate_threw = false;
    try {
        (void)pmg::ParameterSupport({
            {0.0f, 0.0f},
            {1.0f, 0.0f},
            {2.0f, 0.0f},
        });
    } catch (const std::runtime_error&) {
        degenerate_threw = true;
    }
    assert(degenerate_threw);

    return 0;
}
