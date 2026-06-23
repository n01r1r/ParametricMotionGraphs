#pragma once

#include "pmg/BvhLoader.h"
#include "pmg/PmgArtifact.h"
#include "pmg/TransitionDiagnostics.h"

#include <filesystem>
#include <string>
#include <vector>

namespace pmg {

struct SkeletonInspectionJoint {
    int joint_index = -1;
    std::string name;
    int parent_index = -1;
    int channel_count = 0;
    bool is_end_site = false;
    bool is_likely_foot = false;
    bool is_likely_head = false;
    Vec3 rest_position;
    Vec3 frame0_position;
};

struct SkeletonInspectionReport {
    std::string source_path;
    int joint_count = 0;
    int end_site_count = 0;
    std::vector<std::string> root_channels;
    std::vector<std::string> channel_order;
    float frame_time = 0.0f;
    Vec3 bbox_min{};
    Vec3 bbox_max{};
    float bone_length_min = 0.0f;
    float bone_length_mean = 0.0f;
    float bone_length_max = 0.0f;
    std::vector<int> likely_foot_joints;
    std::vector<int> likely_head_joints;
    float floor_estimate = 0.0f;
    std::string up_axis_estimate = "unknown";
    bool up_axis_reliable = false;
    std::vector<SkeletonInspectionJoint> joints;
};

struct LoopAuditReport {
    std::string source_path;
    int frame_count = 0;
    float frames_per_second = 0.0f;
    int start_frame = 0;
    int end_frame = 0;
    float root_normalized_start_end_distance = 0.0f;
    float root_velocity_discontinuity = 0.0f;
    float root_yaw_discontinuity = 0.0f;
    int foot_contact_mismatch = 0;
    bool foot_contact_available = false;
    float cycle_score = 0.0f;
    std::vector<std::string> suggested_notes;
};

struct TransitionPopAuditRow {
    std::string source_node;
    std::string target_node;
    ParameterVector source_parameter;
    ParameterVector target_parameter;
    int source_frame = -1;
    int target_frame = -1;
    float transition_distance = 0.0f;
    float raw_root_position_jump = 0.0f;
    float aligned_root_position_jump = 0.0f;
    float raw_root_yaw_jump = 0.0f;
    float aligned_root_yaw_jump = 0.0f;
    float raw_joint_pose_jump = 0.0f;
    float aligned_joint_pose_jump = 0.0f;
    float raw_velocity_jump = 0.0f;
    float aligned_velocity_jump = 0.0f;
    int contact_mismatch = 0;
    float alignment_yaw = 0.0f;
    float alignment_dx = 0.0f;
    float alignment_dz = 0.0f;
    float local_pop_ratio = 0.0f;
    float root_speed_ratio = 0.0f;
    float yaw_rate_ratio = 0.0f;
    std::string classification;
    std::string montage_reference;
};

struct TransitionPopAuditReport {
    std::string source_node;
    std::string target_node;
    std::string montage_dir;
    int transition_count = 0;
    int accepted_transition_count = 0;
    std::vector<TransitionPopAuditRow> worst_transitions;
    std::vector<std::string> suggested_notes;
};

SkeletonInspectionReport InspectSkeleton(
    const BvhData& data,
    const std::string& source_path = {});

LoopAuditReport AuditLoop(
    const BvhData& data,
    int start_frame = -1,
    int end_frame = -1);

TransitionPopAuditReport AuditTransitionPop(
    const BuiltPmgArtifact& artifact,
    int worst_k = 8);

void WriteSkeletonInspectionArtifacts(
    const SkeletonInspectionReport& report,
    const std::filesystem::path& out_dir);

void WriteLoopAuditReport(
    const LoopAuditReport& report,
    const std::filesystem::path& out_path);

void WriteTransitionPopAuditReport(
    const TransitionPopAuditReport& report,
    const std::filesystem::path& out_path);

}  // namespace pmg
