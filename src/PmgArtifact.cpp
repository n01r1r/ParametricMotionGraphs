#include "pmg/PmgArtifact.h"

#include <stdexcept>

namespace pmg {

RuntimeControllerConfig RuntimeControllerConfigFromArtifact(
    const BuiltPmgArtifact& artifact) {
    RuntimeControllerConfig runtime_config;

    // A node is cyclic when its registration declares a cycle_joint: that is the
    // locomotion/looping signal the offline build used for cyclic phase
    // normalization. The runtime reuses it so kWrapCyclicSelfEdges wraps only
    // genuinely cyclic self-edges (paper §5.2.1) instead of every self-edge.
    runtime_config.cyclic_nodes.assign(
        static_cast<std::size_t>(artifact.graph.NumNodes()), false);
    for (int node_index = 0; node_index < artifact.graph.NumNodes();
         ++node_index) {
        const std::string& node_name = artifact.graph.Node(node_index).name;
        for (const NodeRegistrationMetadata& registration :
             artifact.metadata.node_registrations) {
            if (registration.node_name == node_name) {
                runtime_config.cyclic_nodes[static_cast<std::size_t>(
                    node_index)] = !registration.cycle_joint.empty();
                break;
            }
        }
    }

    if (artifact.metadata.edge_builds.empty()) {
        return runtime_config;
    }

    const int transition_window_frames =
        artifact.metadata.edge_builds.front().config.distance_grid.window_size;
    if (transition_window_frames < 1) {
        throw std::runtime_error(
            "RuntimeControllerConfigFromArtifact: edge window_size must be positive");
    }

    for (const EdgeBuildMetadata& edge_build : artifact.metadata.edge_builds) {
        if (edge_build.config.distance_grid.window_size !=
            transition_window_frames) {
            throw std::runtime_error(
                "RuntimeControllerConfigFromArtifact: all edge builds must use "
                "the same window_size");
        }
    }

    runtime_config.transition_blend_frames = transition_window_frames;
    // Blend placement (RuntimeControllerConfig::convention) is intentionally
    // NOT taken from the artifact's metric convention: PMG locates transitions
    // with Kovar's directional metric but centers the runtime blend (§5.2.1).
    // It keeps the struct default (kPmgCentered).
    return runtime_config;
}

}  // namespace pmg
