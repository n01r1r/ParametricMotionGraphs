#include "pmg/PmgOfflinePipeline.h"

#include "pmg/MotionSpacePreparation.h"

#include <map>
#include <stdexcept>
#include <utility>

namespace pmg {

BuiltPmgArtifact BuildPmgOfflinePipeline(
    const GraphSpec& spec,
    const ArtifactBuildConfig& config) {
    BuiltPmgArtifact artifact;
    MotionSpacePreparationConfig preparation_config;
    preparation_config.default_cycle_joint = config.default_cycle_joint;
    preparation_config.default_contact_joints = config.default_contact_joints;
    preparation_config.default_min_contact_frames =
        config.default_min_contact_frames;
    preparation_config.default_dtw_refine = config.default_dtw_refine;
    preparation_config.calibration_frames_per_second =
        config.default_edge_config.generated_frames_per_second;
    preparation_config.skeleton_offset_tolerance =
        config.skeleton_offset_tolerance;
    const PreparedMotionSpaces prepared =
        PrepareMotionSpaces(spec, preparation_config);

    artifact.skeleton = prepared.skeleton;
    artifact.metadata.source_bvh_paths = prepared.source_bvh_paths;

    std::map<std::string, int> node_indices;
    for (const GraphSpecNode& node : spec.nodes) {
        const PreparedMotionSpace& prepared_node = prepared.Node(node.name);
        node_indices.emplace(
            node.name,
            artifact.graph.AddNode(node.name, prepared_node.production));
        artifact.metadata.node_registrations.push_back(
            prepared_node.registration);
    }

    artifact.metadata.generated_frame_count =
        config.default_edge_config.generated_frame_count;
    artifact.metadata.frames_per_second =
        config.default_edge_config.generated_frames_per_second;
    if (artifact.metadata.generated_frame_count <= 1 ||
        artifact.metadata.frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "BuildPmgOfflinePipeline: generated runtime frame settings are invalid");
    }

    for (const GraphSpecEdge& edge_spec : spec.edges) {
        PmgBuilderConfig edge_config =
            edge_spec.has_build_config ? edge_spec.build_config
                                       : config.default_edge_config;
        edge_config.generated_frame_count = artifact.metadata.generated_frame_count;
        edge_config.generated_frames_per_second = artifact.metadata.frames_per_second;

        const int source_index = node_indices.at(edge_spec.source_node);
        const int target_index = node_indices.at(edge_spec.target_node);
        EdgeBuildResult result = PmgBuilder::BuildEdgeWithReport(
            artifact.skeleton, source_index, target_index,
            artifact.graph.Node(source_index).motion_space,
            artifact.graph.Node(target_index).motion_space, edge_config);
        EdgeBuildMetadata metadata;
        metadata.source_node = edge_spec.source_node;
        metadata.target_node = edge_spec.target_node;
        metadata.config = edge_config;
        metadata.report = result.report;
        artifact.metadata.edge_builds.push_back(metadata);

        // Tolerant build: rejected edges stay visible in metadata, but not graph.
        if (result.report.edge_created) {
            artifact.graph.AddEdge(std::move(result.edge));
        }
    }

    if (!spec.edges.empty() && artifact.graph.NumEdges() == 0) {
        throw std::runtime_error(
            "BuildPmgOfflinePipeline: every declared edge was rejected; no "
            "transitions built");
    }
    return artifact;
}

}  // namespace pmg
