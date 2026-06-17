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

    pmg::ParameterAabb upper_left_box;
    upper_left_box.min_corner = {0.0f, 0.7f};
    upper_left_box.max_corner = {0.4f, 1.0f};
    const pmg::ParameterVector constrained =
        triangle.ProjectInside({1.0f, 1.0f}, upper_left_box);
    assert(triangle.Contains(constrained));
    assert(upper_left_box.Contains(constrained));
    AssertVectorNear(constrained, {0.3f, 0.7f});

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

    bool invalid_triangle_index_threw = false;
    try {
        (void)pmg::ParameterSupport::CreateTriangulated2D(
            {{0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}},
            {{0, 1, 3}});
    } catch (const std::runtime_error&) {
        invalid_triangle_index_threw = true;
    }
    assert(invalid_triangle_index_threw);

    bool duplicate_triangle_threw = false;
    try {
        (void)pmg::ParameterSupport::CreateTriangulated2D(
            {{0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}},
            {{0, 1, 2}, {2, 1, 0}});
    } catch (const std::runtime_error&) {
        duplicate_triangle_threw = true;
    }
    assert(duplicate_triangle_threw);

    // Triangulated 2D tests
    const pmg::ParameterSupport triangulated = pmg::ParameterSupport::CreateTriangulated2D(
        {
            {-0.3f, 0.0f},
            {0.0f, 0.0f},
            {1.0f, 0.0f},
            {0.0f, 1.0f},
            {0.15f, 0.75f}
        },
        {
            {0, 1, 4},
            {1, 2, 4},
            {2, 3, 4},
            {3, 0, 4}
        }
    );

    // Triangulated2D_ContainsAndProjects
    assert(triangulated.Contains({0.0f, 0.5f})); // inside
    assert(!triangulated.Contains({1.0f, 1.0f})); // outside
    AssertVectorNear(triangulated.Project({0.0f, 0.5f}), {0.0f, 0.5f}); // unchanged
    AssertVectorNear(triangulated.Project({1.0f, 1.0f}), {0.5f, 0.5f}); // project into polygon

    // Triangulated2D_FindContainingTriangle / Triangulated2D_BarycentricWeights
    const std::vector<float> w_interior = triangulated.BarycentricWeights({0.15f, 0.75f});
    for (float w : w_interior) {
        assert(w >= 0.0f);
    }
    AssertNear(w_interior[0] + w_interior[1] + w_interior[2] + w_interior[3] + w_interior[4], 1.0f);
    assert(w_interior[0] == 0.0f && w_interior[1] == 0.0f && w_interior[2] == 0.0f && w_interior[3] == 0.0f);
    AssertNear(w_interior[4], 1.0f); // only interior anchor

    const std::vector<float> w_triangle1 = triangulated.BarycentricWeights({0.5f, 0.25f}); // Inside {1, 2, 4}
    AssertNear(w_triangle1[1] + w_triangle1[2] + w_triangle1[4], 1.0f);
    assert(w_triangle1[0] == 0.0f && w_triangle1[3] == 0.0f);

    // Triangulated2D_InteriorAnchorParticipates
    const std::vector<float> w_near_interior = triangulated.BarycentricWeights({0.1f, 0.5f});
    assert(w_near_interior[4] > 0.0f);

    return 0;
}
