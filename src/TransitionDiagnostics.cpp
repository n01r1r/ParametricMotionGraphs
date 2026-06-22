#include "pmg/TransitionDiagnostics.h"

#include "pmg/ContactDetection.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/MotionDistance.h"
#include "pmg/PoseBlend.h"
#include "pmg/RootCanonicalization.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <stdexcept>
#include <vector>

namespace pmg {
namespace {

int FindNode(const ParametricMotionGraph& graph, const std::string& name) {
    for (int index = 0; index < graph.NumNodes(); ++index) {
        if (graph.Node(index).name == name) return index;
    }
    return -1;
}

int FindEdge(const BuiltPmgArtifact& artifact, int source, int target) {
    for (int index = 0; index < artifact.graph.NumEdges(); ++index) {
        const PmgEdge& edge = artifact.graph.Edge(index);
        if (edge.source_node == source && edge.target_node == target) return index;
    }
    return -1;
}

const EdgeBuildMetadata& FindMetadata(
    const BuiltPmgArtifact& artifact,
    const std::string& source,
    const std::string& target) {
    for (const EdgeBuildMetadata& metadata : artifact.metadata.edge_builds) {
        if (metadata.source_node == source && metadata.target_node == target) {
            return metadata;
        }
    }
    throw std::runtime_error("ProbeTransition: missing edge build metadata");
}

OptimalTransition EvaluateTransition(
    const Skeleton& skeleton,
    const MotionClip& source,
    const MotionClip& target,
    const PmgBuilderConfig& config) {
    if (config.transition_metric_type == TransitionMetricType::kDynamicsWindow) {
        return MotionDistance::FindOptimalDynamicsTransitionForConvention(
            skeleton, source, target, config.distance_grid,
            config.transition_metric, config.transition_convention);
    }
    return MotionDistance::FindOptimalTransitionForConvention(
        skeleton, source, target, config.distance_grid,
        config.transition_convention);
}

float HorizontalLength(const Vec3& value) {
    return std::sqrt(value.x * value.x + value.z * value.z);
}

float VelocityJump(
    const MotionClip& source,
    const MotionClip& target,
    const RigidTransform2D& alignment,
    int source_frame,
    int target_frame) {
    if (source_frame <= 0 || target_frame <= 0) return 0.0f;
    const Vec3 source_step = source.frames[source_frame].root_position -
                             source.frames[source_frame - 1].root_position;
    const Vec3 target_step = alignment.ApplyPoint(target.frames[target_frame].root_position) -
                             alignment.ApplyPoint(target.frames[target_frame - 1].root_position);
    return HorizontalLength((target_step - source_step) * source.frames_per_second);
}

int JointIndex(const Skeleton& skeleton, const std::string& name) {
    for (std::size_t index = 0; index < skeleton.joints.size(); ++index) {
        if (skeleton.joints[index].name == name) return static_cast<int>(index);
    }
    return -1;
}

struct ContactJoints { int left = -1; int right = -1; };

ContactJoints ResolveContactJoints(
    const BuiltPmgArtifact& artifact,
    const std::string& node_name,
    const PmgBuilderConfig& config) {
    ContactJoints joints;
    if (!config.transition_metric.contact_joint_indices.empty()) {
        joints.left = config.transition_metric.contact_joint_indices[0];
    }
    if (config.transition_metric.contact_joint_indices.size() > 1) {
        joints.right = config.transition_metric.contact_joint_indices[1];
    }
    for (const NodeRegistrationMetadata& registration :
         artifact.metadata.node_registrations) {
        if (registration.node_name != node_name) continue;
        for (const std::string& name : registration.contact_joints) {
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (lower.find("left") != std::string::npos) joints.left = JointIndex(artifact.skeleton, name);
            if (lower.find("right") != std::string::npos) joints.right = JointIndex(artifact.skeleton, name);
        }
    }
    if (joints.left < 0) joints.left = JointIndex(artifact.skeleton, "LeftAnkle");
    if (joints.right < 0) joints.right = JointIndex(artifact.skeleton, "RightAnkle");
    return joints;
}

TransitionQualityRecord MeasureQualityImpl(
    const BuiltPmgArtifact& artifact,
    const PmgNode& source_node,
    const PmgNode& target_node,
    const PmgBuilderConfig& config,
    const ParameterVector& source_parameter,
    const ParameterVector& target_parameter,
    const OptimalTransition& transition,
    int frames_before,
    int frames_after) {
    if (frames_before < 1 || frames_after < 1) {
        throw std::runtime_error("ProbeTransition: frames_before/after must be positive");
    }
    const MotionClip source = source_node.motion_space.GenerateClip(
        source_parameter, artifact.metadata.frames_per_second);
    const MotionClip target = target_node.motion_space.GenerateClip(
        target_parameter, artifact.metadata.frames_per_second);
    std::vector<Pose> poses;
    poses.reserve(frames_before + frames_after + 1);
    const bool continuous_self = &source_node == &target_node &&
                                 source_parameter == target_parameter;
    const int blend_frames = config.distance_grid.window_size;
    const int first_blend_offset = -blend_frames / 2;
    const int last_blend_offset = first_blend_offset + blend_frames - 1;
    for (int offset = -frames_before; offset <= frames_after; ++offset) {
        const int source_frame = std::clamp(
            transition.source_frame + offset, 0, source.NumFrames() - 1);
        if (continuous_self) {
            poses.push_back(source.frames[source_frame]);
            continue;
        }
        const int target_frame = std::clamp(
            transition.target_frame + offset, 0, target.NumFrames() - 1);
        const Pose aligned = transition.alignment.Apply(target.frames[target_frame]);
        if (offset < first_blend_offset) poses.push_back(source.frames[source_frame]);
        else if (offset > last_blend_offset) poses.push_back(aligned);
        else {
            const float linear = blend_frames == 1 ? 1.0f :
                static_cast<float>(offset - first_blend_offset) /
                static_cast<float>(blend_frames - 1);
            const float alpha = linear * linear * (3.0f - 2.0f * linear);
            poses.push_back(BlendPose(source.frames[source_frame], aligned, alpha));
        }
    }
    const ContactJoints joints = ResolveContactJoints(artifact, source_node.name, config);
    std::vector<int> contact_joints;
    if (joints.left >= 0) contact_joints.push_back(joints.left);
    if (joints.right >= 0 && joints.right != joints.left) contact_joints.push_back(joints.right);
    TransitionQualityContext context;
    context.frames_per_second = artifact.metadata.frames_per_second;
    context.transition_distance = transition.distance;
    context.left_foot_joint = joints.left;
    context.right_foot_joint = joints.right;
    if (!contact_joints.empty()) {
        MotionClip quality_clip;
        quality_clip.frames_per_second = artifact.metadata.frames_per_second;
        quality_clip.frames = poses;
        context.contact_settings = EstimateContactSettings(
            artifact.skeleton, quality_clip, contact_joints);
        context.contact_settings->min_contact_frames = 1;
    }
    TransitionQualityConfig quality_config;
    quality_config.frames_before = frames_before;
    quality_config.frames_after = frames_after;
    return MeasureTransitionQuality(
        artifact.skeleton, poses, frames_before, context, quality_config);
}

}  // namespace

TransitionQualityRecord MeasureExactTransitionQuality(
    const BuiltPmgArtifact& artifact,
    const PmgNode& source_node,
    const PmgNode& target_node,
    const PmgBuilderConfig& config,
    const ParameterVector& source_parameter,
    const ParameterVector& effective_target_parameter,
    const OptimalTransition& transition,
    int frames_before,
    int frames_after) {
    return MeasureQualityImpl(
        artifact, source_node, target_node, config, source_parameter,
        effective_target_parameter, transition, frames_before, frames_after);
}

const char* TransitionMetricClass(float distance, const PmgBuilderConfig& config) {
    if (!std::isfinite(distance)) return "EMPTY";
    if (distance <= config.good_transition_threshold) return "GOOD";
    if (distance >= config.bad_transition_threshold) return "BAD";
    return "NEUTRAL";
}

TransitionProbeResult ProbeTransition(
    const BuiltPmgArtifact& artifact,
    const TransitionProbeRequest& request) {
    std::string source_name = request.source_node;
    std::string target_name = request.target_node;
    if (source_name.empty() && target_name.empty()) {
        if (artifact.graph.NumEdges() != 1) {
            throw std::runtime_error(
                "ProbeTransition: source_node and target_node required for multiple edges");
        }
        const PmgEdge& only_edge = artifact.graph.Edge(0);
        source_name = artifact.graph.Node(only_edge.source_node).name;
        target_name = artifact.graph.Node(only_edge.target_node).name;
    } else if (source_name.empty() || target_name.empty()) {
        throw std::runtime_error(
            "ProbeTransition: source_node and target_node must be provided together");
    }
    const int source_index = FindNode(artifact.graph, source_name);
    const int target_index = FindNode(artifact.graph, target_name);
    if (source_index < 0 || target_index < 0) {
        throw std::runtime_error("ProbeTransition: unknown source or target node");
    }
    const int edge_index = FindEdge(artifact, source_index, target_index);
    if (edge_index < 0) throw std::runtime_error("ProbeTransition: requested edge not found");
    const PmgEdge& edge = artifact.graph.Edge(edge_index);
    const PmgNode& source_node = artifact.graph.Node(source_index);
    const PmgNode& target_node = artifact.graph.Node(target_index);
    const PmgBuilderConfig& config =
        FindMetadata(artifact, source_name, target_name).config;
    const auto lookup = edge.LookupInterpolated(
        request.source_parameter, request.requested_target_parameter);
    if (!lookup) throw std::runtime_error("ProbeTransition: edge lookup returned no transition");

    TransitionProbeResult result;
    result.source_parameter = request.source_parameter;
    result.requested_target_parameter = request.requested_target_parameter;
    result.target_box = lookup->target_parameter_box;
    result.accepted_by_box = result.target_box.Contains(request.requested_target_parameter);
    result.effective_target_parameter = result.target_box.Clamp(
        request.requested_target_parameter);
    if (target_node.motion_space.HasExplicitParameterSupport()) {
        result.effective_target_parameter =
            target_node.motion_space.ExplicitSupport()->ProjectInside(
                request.requested_target_parameter, result.target_box);
    }
    const MotionClip source = source_node.motion_space.GenerateClip(
        request.source_parameter, artifact.metadata.frames_per_second);
    const MotionClip target = target_node.motion_space.GenerateClip(
        result.effective_target_parameter, artifact.metadata.frames_per_second);
    result.transition = EvaluateTransition(artifact.skeleton, source, target, config);
    result.metric_class = TransitionMetricClass(result.transition.distance, config);
    const Pose& source_pose = source.frames[result.transition.source_frame];
    const Pose aligned = result.transition.alignment.Apply(
        target.frames[result.transition.target_frame]);
    result.root_jump = HorizontalLength(aligned.root_position - source_pose.root_position);
    result.heading_jump = std::abs(WrapAngleRadians(
        RootHeadingYaw(aligned) - RootHeadingYaw(source_pose)));
    result.velocity_jump = VelocityJump(
        source, target, result.transition.alignment,
        result.transition.source_frame, result.transition.target_frame);
    result.quality = MeasureExactTransitionQuality(
        artifact, source_node, target_node, config, request.source_parameter,
        result.effective_target_parameter, result.transition,
        request.frames_before, request.frames_after);
    result.quality_decision = EvaluateTransitionQualityGate(
        result.quality, request.quality_gate);
    if (source_index == target_index &&
        request.source_parameter == result.effective_target_parameter) {
        result.quality_decision = {};
    }
    result.final_accepted = result.accepted_by_box && result.quality_decision.accepted;
    result.reject_reason = !result.accepted_by_box
        ? "outside_target_box"
        : TransitionQualityGateReasonName(result.quality_decision.reason);
    return result;
}

}  // namespace pmg
