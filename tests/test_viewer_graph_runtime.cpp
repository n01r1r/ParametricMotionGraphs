#include "PmgViewerWorkspace.h"

#include <cassert>
#include <iostream>
#include <vector>

namespace {

bool NearlyEqual(float left, float right) {
    return std::abs(left - right) <= 1.0e-5f;
}

void TestDesiredParameterUsesFull2DVector() {
    pmg::ParametricMotionSpace space("test_2d", 2);
    
    // Create a 2D triangulated explicit support node
    std::vector<pmg::ParameterVector> vertices = {
        {0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}
    };
    
    // Add examples to space so Domain/MinParameter does not assert
    for (const auto& v : vertices) {
        space.AddExample(v, pmg::MotionClip{"dummy", 30.0f, {pmg::Pose()}});
    }

    std::vector<std::array<int, 3>> triangles = {{0, 1, 2}};
    space.SetParameterSupport(pmg::ParameterSupport::CreateTriangulated2D(vertices, triangles));

    // Test with full 2D vector valid
    pmg::ParameterVector desired = pmgviewer::ResolveDesiredParameterForNode(
        space, 0.5f, true, {0.15f, 0.75f});
    std::cout << "desired size: " << desired.size() << "\n";
    if (desired.size() == 2) {
        std::cout << "desired: " << desired[0] << ", " << desired[1] << "\n";
    }
    assert(desired.size() == 2);
    assert(NearlyEqual(desired[0], 0.15f));
    assert(NearlyEqual(desired[1], 0.75f));

    std::cout << "Test 1 passed\n";

    // Test fallback behavior for 1D logic without full vector
    pmg::ParameterVector fallback = pmgviewer::ResolveDesiredParameterForNode(
        space, 0.5f, false, {});
    std::cout << "fallback size: " << fallback.size() << "\n";
    if (fallback.size() == 2) {
        std::cout << "fallback: " << fallback[0] << ", " << fallback[1] << "\n";
    }
    assert(fallback.size() == 2);
    assert(NearlyEqual(fallback[0], 0.5f)); // 1D slider
    
    std::cout << "Test 2 passed\n";
}

} // namespace

int main() {
    std::cout << "Starting test_viewer_graph_runtime\n";
    TestDesiredParameterUsesFull2DVector();
    std::cout << "test_viewer_graph_runtime passed\n";
    return 0;
}
