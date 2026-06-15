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

    using pmgviewer::FirstNonBlendableExample;
    // A lone example never blends -> always accepted.
    assert(FirstNonBlendableExample({3}, true) == -1);
    assert(FirstNonBlendableExample({}, true) == -1);
    // Same-family walks of differing length (differing contact counts) must NOT
    // be rejected -- structural-equality is deliberately not enforced here.
    assert(FirstNonBlendableExample({4, 6}, true) == -1);
    // Mixing an acyclic clip (vault: <2 foot contacts) is the cross-family jolt.
    assert(FirstNonBlendableExample({4, 1}, true) == 1);
    assert(FirstNonBlendableExample({1, 4}, true) == 0);
    assert(FirstNonBlendableExample({4, 0}, true) == 1);
    // Cyclicity cannot be judged without foot joints -> never block authoring.
    assert(FirstNonBlendableExample({4, 1}, false) == -1);

    std::cout << "test_viewer_graph_authoring passed\n";
    return 0;
}
