#include "GraphAuthoringModel.h"

#include <cassert>
#include <iostream>
#include <limits>
#include <vector>

int main() {
    using pmgviewer::AddAuthoredEdge;
    using pmgviewer::AddAuthoredEdgeResult;
    using pmgviewer::AuthoredEdge;

    const AuthoredEdge defaults;
    assert(defaults.tgood ==
           pmgviewer::kDefaultGoodTransitionThreshold);
    assert(defaults.tbad ==
           pmgviewer::kDefaultBadTransitionThreshold);

    std::vector<AuthoredEdge> edges;
    assert(AddAuthoredEdge(2, 0, 1, 0.5f, 0.7f, edges) ==
           AddAuthoredEdgeResult::Added);
    assert(edges.size() == 1);
    assert(edges.front().source_node == 0);
    assert(edges.front().target_node == 1);

    assert(AddAuthoredEdge(2, 0, 1, 0.6f, 0.8f, edges) ==
           AddAuthoredEdgeResult::Duplicate);
    assert(edges.size() == 1);

    assert(AddAuthoredEdge(2, -1, 1, 0.5f, 0.7f, edges) ==
           AddAuthoredEdgeResult::InvalidNode);
    assert(AddAuthoredEdge(2, 0, 2, 0.5f, 0.7f, edges) ==
           AddAuthoredEdgeResult::InvalidNode);
    assert(AddAuthoredEdge(
               2, 0, 0, std::numeric_limits<float>::quiet_NaN(), 0.7f,
               edges) == AddAuthoredEdgeResult::InvalidThreshold);
    assert(AddAuthoredEdge(2, 0, 0, -0.1f, 0.7f, edges) ==
           AddAuthoredEdgeResult::InvalidThreshold);
    assert(AddAuthoredEdge(2, 0, 0, 0.8f, 0.7f, edges) ==
           AddAuthoredEdgeResult::InvalidThreshold);

    assert(AddAuthoredEdge(2, 1, 0, 0.5f, 0.7f, edges) ==
           AddAuthoredEdgeResult::Added);
    assert(AddAuthoredEdge(2, 0, 0, 0.5f, 0.7f, edges) ==
           AddAuthoredEdgeResult::Added);
    assert(edges.size() == 3);

    std::cout << "test_viewer_graph_authoring passed\n";
    return 0;
}
