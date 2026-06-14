#include "pmg/PmgArtifact.h"

#include <stdexcept>

namespace pmg {

RuntimeControllerConfig RuntimeControllerConfigFromArtifact(
    const BuiltPmgArtifact& artifact) {
    RuntimeControllerConfig runtime_config;
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
