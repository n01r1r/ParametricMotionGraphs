#include "pmg/Diagnostics.h"

#include "pmg/AlignmentStrategy.h"
#include "pmg/ContactDetection.h"
#include "pmg/ForwardKinematics.h"
#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/MotionClip.h"
#include "pmg/PmgArtifact.h"
#include "pmg/RigidTransform2D.h"
#include "pmg/RootCanonicalization.h"
#include "pmg/TransitionQuality.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace pmg {
namespace {

constexpr float kLoopVelocityWeight = 0.50f;
constexpr float kLoopYawWeight = 0.25f;
constexpr float kLoopContactWeight = 0.75f;

std::string JsonEscape(const std::string& text) {
    std::ostringstream out;
    for (const char ch : text) {
        switch (ch) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << ch; break;
        }
    }
    return out.str();
}

std::string CsvEscape(const std::string& text) {
    if (text.find_first_of(",\"") == std::string::npos) {
        return text;
    }
    std::ostringstream out;
    out << '"';
    for (const char ch : text) {
        if (ch == '"') {
            out << "\"\"";
        } else {
            out << ch;
        }
    }
    out << '"';
    return out.str();
}

std::string JoinStrings(const std::vector<std::string>& items, const char* separator) {
    std::ostringstream out;
    for (std::size_t index = 0; index < items.size(); ++index) {
        if (index > 0) {
            out << separator;
        }
        out << items[index];
    }
    return out.str();
}

std::string JoinParameters(const ParameterVector& parameter) {
    std::ostringstream out;
    out << '[';
    for (std::size_t index = 0; index < parameter.size(); ++index) {
        if (index > 0) {
            out << ' ';
        }
        out << parameter[index];
    }
    out << ']';
    return out.str();
}

float HorizontalDistance(const Vec3& first, const Vec3& second) {
    const float dx = second.x - first.x;
    const float dz = second.z - first.z;
    return std::sqrt(dx * dx + dz * dz);
}

Vec3 MinVec(const Vec3& first, const Vec3& second) {
    return {
        std::min(first.x, second.x),
        std::min(first.y, second.y),
        std::min(first.z, second.z),
    };
}

Vec3 MaxVec(const Vec3& first, const Vec3& second) {
    return {
        std::max(first.x, second.x),
        std::max(first.y, second.y),
        std::max(first.z, second.z),
    };
}

std::string ToLower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

bool ContainsAny(const std::string& name, const std::vector<std::string>& needles) {
    const std::string lower = ToLower(name);
    for (const std::string& needle : needles) {
        if (lower.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool LooksLikeEndSite(const Joint& joint) {
    return joint.channels.empty() && joint.parent_index >= 0;
}

bool LooksLikeFootJoint(const Joint& joint) {
    if (LooksLikeEndSite(joint)) {
        return false;
    }
    return ContainsAny(joint.name, {"foot", "ankle", "toe", "ball"});
}

bool LooksLikeHeadJoint(const Joint& joint) {
    if (LooksLikeEndSite(joint)) {
        return false;
    }
    return ContainsAny(joint.name, {"head", "neck"});
}

float BoneLength(const Joint& joint) {
    return std::sqrt(
        joint.offset.x * joint.offset.x +
        joint.offset.y * joint.offset.y +
        joint.offset.z * joint.offset.z);
}

Pose MakeRestPose(const Skeleton& skeleton) {
    Pose pose;
    pose.root_position = {0.0f, 0.0f, 0.0f};
    pose.local_rotations.assign(
        static_cast<std::size_t>(skeleton.NumJoints()), Quaternion::Identity());
    return pose;
}

Vec3 RootNormalizedPoint(const Vec3& point, const Pose& pose) {
    const float yaw = RootHeadingYaw(pose);
    RigidTransform2D inverse;
    inverse.yaw = -yaw;
    inverse.RotateFloor(
        -pose.root_position.x, -pose.root_position.z, inverse.dx, inverse.dz);
    return inverse.ApplyPoint(point);
}

std::vector<Vec3> RootNormalizedPositions(const Skeleton& skeleton, const Pose& pose) {
    std::vector<Vec3> positions = ComputeJointWorldPositions(skeleton, pose);
    for (Vec3& position : positions) {
        position = RootNormalizedPoint(position, pose);
    }
    return positions;
}

float MeanJointDistance(
    const Skeleton& skeleton,
    const Pose& first,
    const Pose& second) {
    const std::vector<Vec3> first_positions =
        ComputeJointWorldPositions(skeleton, first);
    const std::vector<Vec3> second_positions =
        ComputeJointWorldPositions(skeleton, second);

    float distance_sum = 0.0f;
    for (std::size_t joint = 0; joint < first_positions.size(); ++joint) {
        distance_sum += (second_positions[joint] - first_positions[joint]).Norm();
    }
    return distance_sum / static_cast<float>(first_positions.size());
}

float MeanJointDistance(
    const std::vector<Vec3>& first_positions,
    const std::vector<Vec3>& second_positions) {
    if (first_positions.empty() || first_positions.size() != second_positions.size()) {
        return 0.0f;
    }
    float distance_sum = 0.0f;
    for (std::size_t joint = 0; joint < first_positions.size(); ++joint) {
        distance_sum += (second_positions[joint] - first_positions[joint]).Norm();
    }
    return distance_sum / static_cast<float>(first_positions.size());
}

float RootVelocity(const MotionClip& clip, int first_frame, int second_frame) {
    if (first_frame < 0 || second_frame < 0 ||
        first_frame >= clip.NumFrames() || second_frame >= clip.NumFrames()) {
        return 0.0f;
    }
    return HorizontalDistance(
        clip.frames[static_cast<std::size_t>(first_frame)].root_position,
        clip.frames[static_cast<std::size_t>(second_frame)].root_position) *
        clip.frames_per_second;
}

float RootYawDelta(const Pose& first, const Pose& second) {
    return WrapAngleRadians(RootHeadingYaw(second) - RootHeadingYaw(first));
}

Pose AlignPoseForTransition(
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const InterpolatedTransition& transition,
    const RuntimeControllerConfig& runtime_config,
    const RigidTransform2D& alignment,
    int target_frame) {
    float target_time_seconds =
        static_cast<float>(target_frame) / target_clip.frames_per_second;
    const float blend_seconds = TransitionWindowSpanSeconds(
        runtime_config.transition_blend_frames,
        source_clip.frames_per_second);
    const int half_window = runtime_config.transition_blend_frames / 2;
    const float target_lead_seconds =
        runtime_config.convention == TransitionWindowConvention::kPmgCentered
            ? static_cast<float>(half_window) / source_clip.frames_per_second
            : blend_seconds;
    const float raw_target_start =
        transition.target_transition_phase * target_clip.DurationSeconds() -
        target_lead_seconds;
    const float normalized_phase =
        target_clip.DurationSeconds() > 0.0f
            ? (target_time_seconds - raw_target_start) /
                  target_clip.DurationSeconds()
            : 0.0f;
    return alignment.Apply(target_clip.SampleNormalizedPhase(normalized_phase));
}

RigidTransform2D ResolveRuntimeAlignment(
    const BuiltPmgArtifact& artifact,
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const InterpolatedTransition& transition) {
    const RuntimeControllerConfig runtime_config =
        RuntimeControllerConfigFromArtifact(artifact);
    const int half_window = runtime_config.transition_blend_frames / 2;
    const float source_first_offset_phase =
        runtime_config.convention == TransitionWindowConvention::kPmgCentered
            ? static_cast<float>(half_window) /
                  static_cast<float>(std::max(1, source_clip.NumFrames() - 1))
            : 0.0f;
    const float live_source_phase =
        transition.source_transition_phase - source_first_offset_phase;
    const AlignmentContext context{
        source_clip,
        target_clip,
        live_source_phase,
        transition,
        runtime_config.transition_blend_frames,
        runtime_config.convention};
    const PointCloudAlignment alignment(
        artifact.skeleton,
        runtime_config.transition_blend_frames);
    return alignment.Resolve(context);
}

float TransitionVelocityJump(
    const MotionClip& source_clip,
    const MotionClip& target_clip,
    const RigidTransform2D& alignment,
    int source_frame,
    int target_frame) {
    if (source_frame <= 0 || target_frame <= 0) {
        return 0.0f;
    }
    const Vec3 source_step =
        source_clip.frames[static_cast<std::size_t>(source_frame)].root_position -
        source_clip.frames[static_cast<std::size_t>(source_frame - 1)].root_position;
    const Vec3 target_step =
        alignment.ApplyPoint(
            target_clip.frames[static_cast<std::size_t>(target_frame)].root_position) -
        alignment.ApplyPoint(
            target_clip.frames[static_cast<std::size_t>(target_frame - 1)].root_position);
    return HorizontalDistance(
        source_step * source_clip.frames_per_second,
        target_step * source_clip.frames_per_second);
}

TransitionPopAuditRow MakeTransitionPopRow(
    const BuiltPmgArtifact& artifact,
    const PmgNode& source_node,
    const PmgNode& target_node,
    const TransitionSample& source_sample,
    const TargetTransitionPhaseSample& target_sample,
    const TransitionProbeResult& probe) {
    TransitionPopAuditRow row;
    row.source_node = source_node.name;
    row.target_node = target_node.name;
    row.source_parameter = source_sample.source_parameter;
    row.target_parameter = target_sample.target_parameter;
    row.source_frame = probe.transition.source_frame;
    row.target_frame = probe.transition.target_frame;
    row.transition_distance = probe.transition.distance;
    row.local_pop_ratio = probe.quality.local_pop_ratio;
    row.root_speed_ratio = probe.quality.root_speed_ratio;
    row.yaw_rate_ratio = probe.quality.yaw_rate_ratio;
    row.classification =
        TransitionQualityClassificationName(probe.quality.classification);
    row.montage_reference = std::string("transition_montage_manifest.csv#") +
        source_node.name + "_" + target_node.name;

    const MotionClip source_clip = source_node.motion_space.GenerateClip(
        row.source_parameter, artifact.metadata.frames_per_second);
    const MotionClip target_clip = target_node.motion_space.GenerateClip(
        probe.effective_target_parameter, artifact.metadata.frames_per_second);
    const Pose source_pose =
        source_clip.frames[static_cast<std::size_t>(probe.transition.source_frame)];
    const Pose raw_target_pose =
        target_clip.frames[static_cast<std::size_t>(probe.transition.target_frame)];
    const InterpolatedTransition runtime_transition{
        source_sample.target_parameter_box,
        target_sample.source_transition_phase,
        target_sample.target_transition_phase,
        source_sample.transition_distance};

    row.raw_root_position_jump = HorizontalDistance(
        source_pose.root_position, raw_target_pose.root_position);
    row.raw_root_yaw_jump = std::abs(
        WrapAngleRadians(
            RootHeadingYaw(raw_target_pose) - RootHeadingYaw(source_pose)));
    row.raw_joint_pose_jump = MeanJointDistance(
        artifact.skeleton, source_pose, raw_target_pose);
    row.raw_velocity_jump = TransitionVelocityJump(
        source_clip, target_clip, RigidTransform2D{},
        probe.transition.source_frame, probe.transition.target_frame);

    const RigidTransform2D runtime_alignment = ResolveRuntimeAlignment(
        artifact, source_clip, target_clip, runtime_transition);
    const RuntimeControllerConfig runtime_config =
        RuntimeControllerConfigFromArtifact(artifact);
    const Pose aligned_target_pose = AlignPoseForTransition(
        source_clip, target_clip, runtime_transition, runtime_config,
        runtime_alignment, probe.transition.target_frame);
    row.aligned_root_position_jump = HorizontalDistance(
        source_pose.root_position, aligned_target_pose.root_position);
    row.aligned_root_yaw_jump = std::abs(
        WrapAngleRadians(
            RootHeadingYaw(aligned_target_pose) - RootHeadingYaw(source_pose)));
    row.aligned_joint_pose_jump = MeanJointDistance(
        artifact.skeleton, source_pose, aligned_target_pose);
    row.aligned_velocity_jump = TransitionVelocityJump(
        source_clip, target_clip, runtime_alignment,
        probe.transition.source_frame, probe.transition.target_frame);
    row.alignment_yaw = runtime_alignment.yaw;
    row.alignment_dx = runtime_alignment.dx;
    row.alignment_dz = runtime_alignment.dz;

    int contact_mismatch = 0;
    if (probe.quality.left_contact_before != TransitionContactState::kUnknown &&
        probe.quality.left_contact_after != TransitionContactState::kUnknown &&
        probe.quality.left_contact_before != probe.quality.left_contact_after) {
        ++contact_mismatch;
    }
    if (probe.quality.right_contact_before != TransitionContactState::kUnknown &&
        probe.quality.right_contact_after != TransitionContactState::kUnknown &&
        probe.quality.right_contact_before != probe.quality.right_contact_after) {
        ++contact_mismatch;
    }
    row.contact_mismatch = contact_mismatch;
    return row;
}

std::vector<int> FindLikelyJoints(
    const Skeleton& skeleton,
    bool (*predicate)(const Joint&)) {
    std::vector<int> joint_indices;
    for (int joint_index = 0; joint_index < skeleton.NumJoints(); ++joint_index) {
        if (predicate(skeleton.joints[static_cast<std::size_t>(joint_index)])) {
            joint_indices.push_back(joint_index);
        }
    }
    return joint_indices;
}

std::string JoinIndices(const std::vector<int>& indices) {
    std::vector<std::string> text;
    text.reserve(indices.size());
    for (int index : indices) {
        text.push_back(std::to_string(index));
    }
    return JoinStrings(text, ", ");
}

std::string UpAxisEstimate(const BvhData& data, bool* reliable) {
    if (reliable != nullptr) {
        *reliable = false;
    }
    if (data.skeleton.NumJoints() == 0 || data.clip.NumFrames() == 0) {
        return "unknown";
    }

    const std::vector<Vec3> first_positions =
        ComputeJointWorldPositions(data.skeleton, data.clip.frames.front());
    Vec3 min_position = first_positions.front();
    Vec3 max_position = first_positions.front();
    for (const Pose& pose : data.clip.frames) {
        const std::vector<Vec3> positions =
            ComputeJointWorldPositions(data.skeleton, pose);
        for (const Vec3& position : positions) {
            min_position = MinVec(min_position, position);
            max_position = MaxVec(max_position, position);
        }
    }

    const float range_x = max_position.x - min_position.x;
    const float range_y = max_position.y - min_position.y;
    const float range_z = max_position.z - min_position.z;
    const float vertical_range = std::max({range_x, range_y, range_z});
    if (vertical_range <= 0.0f) {
        return "unknown";
    }

    if (range_y >= range_x * 1.2f && range_y >= range_z * 1.2f) {
        if (reliable != nullptr) {
            *reliable = true;
        }
        return "Y";
    }
    if (range_x >= range_y * 1.2f && range_x >= range_z * 1.2f) {
        if (reliable != nullptr) {
            *reliable = true;
        }
        return "X";
    }
    if (range_z >= range_x * 1.2f && range_z >= range_y * 1.2f) {
        if (reliable != nullptr) {
            *reliable = true;
        }
        return "Z";
    }
    return "unknown";
}

float FloorEstimate(const BvhData& data) {
    if (data.skeleton.NumJoints() == 0 || data.clip.NumFrames() == 0) {
        return 0.0f;
    }

    float floor = std::numeric_limits<float>::infinity();
    for (const Pose& pose : data.clip.frames) {
        const std::vector<Vec3> positions =
            ComputeJointWorldPositions(data.skeleton, pose);
        for (const Vec3& position : positions) {
            floor = std::min(floor, position.y);
        }
    }
    return std::isfinite(floor) ? floor : 0.0f;
}

void WriteCsvHeader(std::ofstream& csv) {
    csv << "joint_index,joint_name,parent_index,is_end_site,is_likely_foot,is_likely_head,"
           "x,y,z\n";
}

}  // namespace

SkeletonInspectionReport InspectSkeleton(
    const BvhData& data,
    const std::string& source_path) {
    if (data.skeleton.NumJoints() == 0) {
        throw std::runtime_error("InspectSkeleton: skeleton must not be empty");
    }

    SkeletonInspectionReport report;
    report.source_path = source_path;
    report.joint_count = data.skeleton.NumJoints();
    report.frame_time = data.clip.frames_per_second > 0.0f
        ? 1.0f / data.clip.frames_per_second
        : 0.0f;

    report.likely_foot_joints = FindLikelyJoints(data.skeleton, LooksLikeFootJoint);
    report.likely_head_joints = FindLikelyJoints(data.skeleton, LooksLikeHeadJoint);
    report.end_site_count = 0;

    float bone_length_sum = 0.0f;
    int bone_length_count = 0;
    report.bbox_min = {0.0f, 0.0f, 0.0f};
    report.bbox_max = {0.0f, 0.0f, 0.0f};

    const Pose rest_pose = MakeRestPose(data.skeleton);
    const Pose frame0_pose = data.clip.NumFrames() > 0 ? data.clip.frames.front() : rest_pose;
    const std::vector<Vec3> rest_positions =
        ComputeJointWorldPositions(data.skeleton, rest_pose);
    const std::vector<Vec3> frame0_positions =
        ComputeJointWorldPositions(data.skeleton, frame0_pose);

    for (int joint_index = 0; joint_index < data.skeleton.NumJoints(); ++joint_index) {
        const Joint& joint = data.skeleton.joints[static_cast<std::size_t>(joint_index)];
        const bool is_end_site = LooksLikeEndSite(joint);
        const bool is_foot = LooksLikeFootJoint(joint);
        const bool is_head = LooksLikeHeadJoint(joint);
        report.end_site_count += is_end_site ? 1 : 0;
        if (!is_end_site && joint.parent_index >= 0) {
            const float length = BoneLength(joint);
            bone_length_sum += length;
            report.bone_length_min = bone_length_count == 0
                ? length
                : std::min(report.bone_length_min, length);
            report.bone_length_max = bone_length_count == 0
                ? length
                : std::max(report.bone_length_max, length);
            ++bone_length_count;
        }

        SkeletonInspectionJoint row;
        row.joint_index = joint_index;
        row.name = joint.name;
        row.parent_index = joint.parent_index;
        row.channel_count = static_cast<int>(joint.channels.size());
        row.is_end_site = is_end_site;
        row.is_likely_foot = is_foot;
        row.is_likely_head = is_head;
        row.rest_position = rest_positions[static_cast<std::size_t>(joint_index)];
        row.frame0_position = frame0_positions[static_cast<std::size_t>(joint_index)];
        report.joints.push_back(std::move(row));
        const Vec3& position = frame0_positions[static_cast<std::size_t>(joint_index)];
        if (report.joints.size() == 1) {
            report.bbox_min = position;
            report.bbox_max = position;
        } else {
            report.bbox_min = MinVec(report.bbox_min, position);
            report.bbox_max = MaxVec(report.bbox_max, position);
        }
    }

    report.bone_length_mean = bone_length_count > 0
        ? bone_length_sum / static_cast<float>(bone_length_count)
        : 0.0f;

    if (report.joint_count > 0) {
        for (const Vec3& position : frame0_positions) {
            report.bbox_min = MinVec(report.bbox_min, position);
            report.bbox_max = MaxVec(report.bbox_max, position);
        }
    }

    report.floor_estimate = FloorEstimate(data);
    bool up_axis_reliable = false;
    report.up_axis_estimate = UpAxisEstimate(data, &up_axis_reliable);
    report.up_axis_reliable = up_axis_reliable;
    report.root_channels.clear();
    if (!data.skeleton.joints.empty()) {
        for (const BvhChannelType channel : data.skeleton.joints.front().channels) {
            switch (channel) {
                case BvhChannelType::XPosition: report.root_channels.push_back("Xposition"); break;
                case BvhChannelType::YPosition: report.root_channels.push_back("Yposition"); break;
                case BvhChannelType::ZPosition: report.root_channels.push_back("Zposition"); break;
                case BvhChannelType::XRotation: report.root_channels.push_back("Xrotation"); break;
                case BvhChannelType::YRotation: report.root_channels.push_back("Yrotation"); break;
                case BvhChannelType::ZRotation: report.root_channels.push_back("Zrotation"); break;
            }
        }
    }
    for (const Joint& joint : data.skeleton.joints) {
        for (const BvhChannelType channel : joint.channels) {
            std::ostringstream entry;
            entry << joint.name << ':';
            switch (channel) {
                case BvhChannelType::XPosition: entry << "Xposition"; break;
                case BvhChannelType::YPosition: entry << "Yposition"; break;
                case BvhChannelType::ZPosition: entry << "Zposition"; break;
                case BvhChannelType::XRotation: entry << "Xrotation"; break;
                case BvhChannelType::YRotation: entry << "Yrotation"; break;
                case BvhChannelType::ZRotation: entry << "Zrotation"; break;
            }
            report.channel_order.push_back(entry.str());
        }
    }
    return report;
}

LoopAuditReport AuditLoop(
    const BvhData& data,
    int start_frame,
    int end_frame) {
    if (data.skeleton.NumJoints() == 0) {
        throw std::runtime_error("AuditLoop: skeleton must not be empty");
    }
    if (data.clip.NumFrames() < 2) {
        throw std::runtime_error("AuditLoop: clip must contain at least 2 frames");
    }

    LoopAuditReport report;
    report.frame_count = data.clip.NumFrames();
    report.frames_per_second = data.clip.frames_per_second;
    report.start_frame = start_frame >= 0 ? start_frame : 0;
    report.end_frame = end_frame >= 0 ? end_frame : data.clip.NumFrames() - 1;
    if (report.start_frame < 0 || report.end_frame >= data.clip.NumFrames() ||
        report.end_frame <= report.start_frame) {
        throw std::runtime_error("AuditLoop: invalid start/end frame range");
    }
    if (report.end_frame - report.start_frame < 1) {
        throw std::runtime_error("AuditLoop: start/end frame range must span at least 2 frames");
    }

    const Pose& start_pose = data.clip.frames[static_cast<std::size_t>(report.start_frame)];
    const Pose& end_pose = data.clip.frames[static_cast<std::size_t>(report.end_frame)];
    report.root_normalized_start_end_distance = MeanJointDistance(
        RootNormalizedPositions(data.skeleton, start_pose),
        RootNormalizedPositions(data.skeleton, end_pose));

    const int start_next = report.start_frame + 1;
    const int end_prev = report.end_frame - 1;
    const float start_velocity = RootVelocity(data.clip, report.start_frame, start_next);
    const float end_velocity = RootVelocity(data.clip, end_prev, report.end_frame);
    report.root_velocity_discontinuity = std::abs(end_velocity - start_velocity);

    const float start_yaw_delta = RootYawDelta(
        data.clip.frames[static_cast<std::size_t>(report.start_frame)],
        data.clip.frames[static_cast<std::size_t>(start_next)]);
    const float end_yaw_delta = RootYawDelta(
        data.clip.frames[static_cast<std::size_t>(end_prev)],
        data.clip.frames[static_cast<std::size_t>(report.end_frame)]);
    report.root_yaw_discontinuity = std::abs(WrapAngleRadians(end_yaw_delta - start_yaw_delta));

    const std::vector<int> foot_joints = FindLikelyJoints(data.skeleton, LooksLikeFootJoint);
    report.foot_contact_available = !foot_joints.empty();
    if (report.foot_contact_available) {
        const std::vector<ContactInterval> contacts = DetectContacts(
            data.skeleton, data.clip, foot_joints,
            EstimateContactSettings(data.skeleton, data.clip, foot_joints));
        const auto contact_state = [&contacts](int joint_index, int frame_index) {
            for (const ContactInterval& interval : contacts) {
                if (interval.joint_index == joint_index &&
                    frame_index >= interval.first_frame &&
                    frame_index <= interval.last_frame) {
                    return true;
                }
            }
            return false;
        };
        for (int joint_index : foot_joints) {
            if (contact_state(joint_index, report.start_frame) !=
                contact_state(joint_index, report.end_frame)) {
                ++report.foot_contact_mismatch;
            }
        }
    }

    report.cycle_score = report.root_normalized_start_end_distance +
        kLoopVelocityWeight * report.root_velocity_discontinuity +
        kLoopYawWeight * report.root_yaw_discontinuity +
        kLoopContactWeight * static_cast<float>(report.foot_contact_mismatch);

    report.suggested_notes.push_back(
        report.root_normalized_start_end_distance > 1.0f
            ? "start/end pose far apart after root normalization"
            : "start/end pose shape is reasonably close");
    report.suggested_notes.push_back(
        report.root_velocity_discontinuity > 1.0f
            ? "root speed changes sharply at seam"
            : "root speed seam looks mild");
    report.suggested_notes.push_back(
        report.root_yaw_discontinuity > 0.35f
            ? "heading jumps at seam"
            : "heading seam looks mild");
    if (report.foot_contact_available) {
        report.suggested_notes.push_back(
            report.foot_contact_mismatch > 0
                ? "foot contact state differs across seam"
                : "foot contact state matches across seam");
    } else {
        report.suggested_notes.push_back("foot contact heuristic unavailable");
    }
    return report;
}

TransitionPopAuditReport AuditTransitionPop(
    const BuiltPmgArtifact& artifact,
    int worst_k) {
    if (artifact.graph.NumEdges() == 0) {
        throw std::runtime_error("AuditTransitionPop: artifact contains no transitions");
    }
    if (worst_k < 1) {
        throw std::runtime_error("AuditTransitionPop: worst_k must be positive");
    }

    TransitionPopAuditReport report;
    report.montage_dir = "transition_montage_manifest.csv";
    bool first_edge = true;

    for (int edge_index = 0; edge_index < artifact.graph.NumEdges(); ++edge_index) {
        const PmgEdge& edge = artifact.graph.Edge(edge_index);
        const PmgNode& source_node = artifact.graph.Node(edge.source_node);
        const PmgNode& target_node = artifact.graph.Node(edge.target_node);
        if (first_edge) {
            report.source_node = source_node.name;
            report.target_node = target_node.name;
            first_edge = false;
        } else if (report.source_node != source_node.name ||
                   report.target_node != target_node.name) {
            report.source_node = "mixed";
            report.target_node = "mixed";
        }

        for (const TransitionSample& source_sample : edge.samples) {
            for (const TargetTransitionPhaseSample& target_sample :
                 source_sample.target_phase_samples) {
                TransitionProbeRequest request;
                request.source_node = source_node.name;
                request.target_node = target_node.name;
                request.source_parameter = source_sample.source_parameter;
                request.requested_target_parameter = target_sample.target_parameter;
                request.frames_before = 3;
                request.frames_after = 3;
                request.quality_gate.enabled = false;
                const TransitionProbeResult probe = ProbeTransition(artifact, request);
                if (!probe.final_accepted) {
                    continue;
                }
                TransitionPopAuditRow row = MakeTransitionPopRow(
                    artifact, source_node, target_node, source_sample,
                    target_sample, probe);
                ++report.transition_count;
                ++report.accepted_transition_count;
                report.worst_transitions.push_back(std::move(row));
            }
        }
    }

    if (report.transition_count == 0) {
        throw std::runtime_error(
            "AuditTransitionPop: artifact contains no accepted transitions");
    }

    std::sort(
        report.worst_transitions.begin(), report.worst_transitions.end(),
        [](const TransitionPopAuditRow& first, const TransitionPopAuditRow& second) {
            if (first.aligned_joint_pose_jump != second.aligned_joint_pose_jump) {
                return first.aligned_joint_pose_jump > second.aligned_joint_pose_jump;
            }
            if (first.aligned_root_position_jump != second.aligned_root_position_jump) {
                return first.aligned_root_position_jump > second.aligned_root_position_jump;
            }
            return first.aligned_velocity_jump > second.aligned_velocity_jump;
        });
    if (static_cast<int>(report.worst_transitions.size()) > worst_k) {
        report.worst_transitions.resize(static_cast<std::size_t>(worst_k));
    }

    report.suggested_notes.push_back(
        report.worst_transitions.empty()
            ? "no accepted transitions to audit"
            : "worst rows sorted by aligned joint jump, then aligned root jump");
    report.suggested_notes.push_back(
        "raw seam fields use target clip as-is; aligned seam fields recompute runtime point-cloud alignment");
    report.suggested_notes.push_back(
        "montage references reuse existing transition_montage_manifest.csv entries");
    return report;
}

void WriteSkeletonInspectionArtifacts(
    const SkeletonInspectionReport& report,
    const std::filesystem::path& out_dir) {
    std::filesystem::create_directories(out_dir);

    const std::filesystem::path md_path = out_dir / "skeleton_report.md";
    const std::filesystem::path json_path = out_dir / "skeleton_hierarchy.json";
    const std::filesystem::path rest_csv_path = out_dir / "joint_positions_rest.csv";
    const std::filesystem::path frame0_csv_path = out_dir / "joint_positions_frame0.csv";

    std::ofstream md(md_path);
    if (!md) {
        throw std::runtime_error("WriteSkeletonInspectionArtifacts: cannot write markdown");
    }
    md << "# Skeleton Inspection Report\n\n";
    md << "## Purpose\n\n";
    md << "Report-only BVH skeleton audit. No PMG algorithm changes.\n\n";
    md << "## Inputs\n\n";
    md << "- Source: `" << report.source_path << "`\n";
    md << "- Joint count: `" << report.joint_count << "`\n";
    md << "- End-site count: `" << report.end_site_count << "`\n";
    md << "- Frame time: `" << report.frame_time << "`\n";
    md << "- Root channels: `" << JoinStrings(report.root_channels, ", ") << "`\n";
    md << "- Channel order: `" << JoinStrings(report.channel_order, ", ") << "`\n";
    md << "- Bone length stats: min `" << report.bone_length_min << "` mean `"
       << report.bone_length_mean << "` max `" << report.bone_length_max << "`\n";
    md << "- Likely foot joints: `" << JoinIndices(report.likely_foot_joints) << "`\n";
    md << "- Likely head joints: `" << JoinIndices(report.likely_head_joints) << "`\n";
    md << "- Floor estimate: `" << report.floor_estimate << "`\n";
    md << "- Up-axis estimate: `" << report.up_axis_estimate
       << "` (reliable=" << (report.up_axis_reliable ? "yes" : "no") << ")\n";
    md << "- Bounding box: min `" << report.bbox_min.x << ' ' << report.bbox_min.y
       << ' ' << report.bbox_min.z << "` max `" << report.bbox_max.x << ' '
       << report.bbox_max.y << ' ' << report.bbox_max.z << "`\n\n";
    md << "## Artifacts\n\n";
    md << "- Markdown report: `" << md_path.string() << "`\n";
    md << "- Hierarchy JSON: `" << json_path.string() << "`\n";
    md << "- Rest pose CSV: `" << rest_csv_path.string() << "`\n";
    md << "- Frame 0 CSV: `" << frame0_csv_path.string() << "`\n";

    std::ofstream json(json_path);
    if (!json) {
        throw std::runtime_error("WriteSkeletonInspectionArtifacts: cannot write JSON");
    }
    json << "{\n";
    json << "  \"source_path\": \"" << JsonEscape(report.source_path) << "\",\n";
    json << "  \"joint_count\": " << report.joint_count << ",\n";
    json << "  \"end_site_count\": " << report.end_site_count << ",\n";
    json << "  \"frame_time\": " << report.frame_time << ",\n";
    json << "  \"root_channels\": [";
    for (std::size_t index = 0; index < report.root_channels.size(); ++index) {
        if (index > 0) json << ", ";
        json << "\"" << JsonEscape(report.root_channels[index]) << "\"";
    }
    json << "],\n";
    json << "  \"channel_order\": [";
    for (std::size_t index = 0; index < report.channel_order.size(); ++index) {
        if (index > 0) json << ", ";
        json << "\"" << JsonEscape(report.channel_order[index]) << "\"";
    }
    json << "],\n";
    json << "  \"bbox_min\": [" << report.bbox_min.x << ", " << report.bbox_min.y
         << ", " << report.bbox_min.z << "],\n";
    json << "  \"bbox_max\": [" << report.bbox_max.x << ", " << report.bbox_max.y
         << ", " << report.bbox_max.z << "],\n";
    json << "  \"bone_length_min\": " << report.bone_length_min << ",\n";
    json << "  \"bone_length_mean\": " << report.bone_length_mean << ",\n";
    json << "  \"bone_length_max\": " << report.bone_length_max << ",\n";
    json << "  \"likely_foot_joints\": [" << JoinIndices(report.likely_foot_joints) << "],\n";
    json << "  \"likely_head_joints\": [" << JoinIndices(report.likely_head_joints) << "],\n";
    json << "  \"floor_estimate\": " << report.floor_estimate << ",\n";
    json << "  \"up_axis_estimate\": \"" << JsonEscape(report.up_axis_estimate)
         << "\",\n";
    json << "  \"up_axis_reliable\": " << (report.up_axis_reliable ? "true" : "false")
         << ",\n";
    json << "  \"joints\": [\n";
    for (std::size_t index = 0; index < report.joints.size(); ++index) {
        const SkeletonInspectionJoint& joint = report.joints[index];
        json << "    {\"joint_index\": " << joint.joint_index
             << ", \"name\": \"" << JsonEscape(joint.name) << "\""
             << ", \"parent_index\": " << joint.parent_index
             << ", \"channel_count\": " << joint.channel_count
             << ", \"is_end_site\": " << (joint.is_end_site ? "true" : "false")
             << ", \"is_likely_foot\": " << (joint.is_likely_foot ? "true" : "false")
             << ", \"is_likely_head\": " << (joint.is_likely_head ? "true" : "false")
             << ", \"rest_position\": [" << joint.rest_position.x << ", "
             << joint.rest_position.y << ", " << joint.rest_position.z << "]"
             << ", \"frame0_position\": [" << joint.frame0_position.x << ", "
             << joint.frame0_position.y << ", " << joint.frame0_position.z << "]"
             << "}";
        if (index + 1 < report.joints.size()) {
            json << ",";
        }
        json << "\n";
    }
    json << "  ]\n";
    json << "}\n";

    std::ofstream rest_csv(rest_csv_path);
    if (!rest_csv) {
        throw std::runtime_error("WriteSkeletonInspectionArtifacts: cannot write rest CSV");
    }
    WriteCsvHeader(rest_csv);
    for (const SkeletonInspectionJoint& joint : report.joints) {
        rest_csv << joint.joint_index << ',' << CsvEscape(joint.name) << ','
                 << joint.parent_index << ',' << (joint.is_end_site ? "true" : "false")
                 << ',' << (joint.is_likely_foot ? "true" : "false")
                 << ',' << (joint.is_likely_head ? "true" : "false")
                 << ',' << joint.rest_position.x << ',' << joint.rest_position.y << ','
                 << joint.rest_position.z << '\n';
    }

    std::ofstream frame0_csv(frame0_csv_path);
    if (!frame0_csv) {
        throw std::runtime_error("WriteSkeletonInspectionArtifacts: cannot write frame0 CSV");
    }
    WriteCsvHeader(frame0_csv);
    for (const SkeletonInspectionJoint& joint : report.joints) {
        frame0_csv << joint.joint_index << ',' << CsvEscape(joint.name) << ','
                   << joint.parent_index << ',' << (joint.is_end_site ? "true" : "false")
                   << ',' << (joint.is_likely_foot ? "true" : "false")
                   << ',' << (joint.is_likely_head ? "true" : "false")
                   << ',' << joint.frame0_position.x << ',' << joint.frame0_position.y
                   << ',' << joint.frame0_position.z << '\n';
    }
}

void WriteLoopAuditReport(
    const LoopAuditReport& report,
    const std::filesystem::path& out_path) {
    const std::filesystem::path parent = out_path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    std::ofstream md(out_path);
    if (!md) {
        throw std::runtime_error("WriteLoopAuditReport: cannot write markdown");
    }
    md << "# Loop Audit\n\n";
    md << "- Source: `" << report.source_path << "`\n";
    md << "- Frame range: `" << report.start_frame << " .. " << report.end_frame << "`\n";
    md << "- Root-normalized start/end pose distance: `" << report.root_normalized_start_end_distance << "`\n";
    md << "- Root velocity discontinuity: `" << report.root_velocity_discontinuity << "`\n";
    md << "- Root yaw discontinuity: `" << report.root_yaw_discontinuity << "`\n";
    md << "- Foot contact mismatch: `" << report.foot_contact_mismatch << "`\n";
    md << "- Cycle score: `" << report.cycle_score << "`\n";
    md << "- Foot contact available: `" << (report.foot_contact_available ? "yes" : "no") << "`\n\n";
    md << "## Suggested notes\n\n";
    for (const std::string& note : report.suggested_notes) {
        md << "- " << note << "\n";
    }
}

void WriteTransitionPopAuditReport(
    const TransitionPopAuditReport& report,
    const std::filesystem::path& out_path) {
    const std::filesystem::path parent = out_path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    std::ofstream md(out_path);
    if (!md) {
        throw std::runtime_error("WriteTransitionPopAuditReport: cannot write markdown");
    }
    md << "# Transition Pop Audit\n\n";
    md << "- Source node: `" << report.source_node << "`\n";
    md << "- Target node: `" << report.target_node << "`\n";
    md << "- Transition count: `" << report.transition_count << "`\n";
    md << "- Accepted transition count: `" << report.accepted_transition_count << "`\n";
    md << "- Montage reference: `" << report.montage_dir << "`\n\n";
    md << "## Suggested notes\n\n";
    for (const std::string& note : report.suggested_notes) {
        md << "- " << note << "\n";
    }
    md << "\n## Worst transitions\n\n";
    md << "| Source p | Target p | Src frame | Tgt frame | D | Raw root | Aligned root | Raw joint | Aligned joint | Raw vel | Aligned vel | Contact mismatch | Align yaw | Align dx | Align dz | Montage |\n";
    md << "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|\n";
    for (const TransitionPopAuditRow& row : report.worst_transitions) {
        md << "| " << JoinParameters(row.source_parameter)
           << " | " << JoinParameters(row.target_parameter)
           << " | " << row.source_frame
           << " | " << row.target_frame
           << " | " << row.transition_distance
           << " | " << row.raw_root_position_jump
           << " | " << row.aligned_root_position_jump
           << " | " << row.raw_joint_pose_jump
           << " | " << row.aligned_joint_pose_jump
           << " | " << row.raw_velocity_jump
           << " | " << row.aligned_velocity_jump
           << " | " << row.contact_mismatch
           << " | " << row.alignment_yaw
           << " | " << row.alignment_dx
           << " | " << row.alignment_dz
           << " | `" << row.montage_reference << "` |\n";
    }
}

}  // namespace pmg
