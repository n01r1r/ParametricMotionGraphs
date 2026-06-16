#include "pmg/CyclicContinuity.h"

#include "pmg/ForwardKinematics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace pmg {

namespace {

constexpr float kRatioEpsilon = 1.0e-6f;

float SymmetricPositiveRatio(float first, float second) {
    const float first_magnitude = std::abs(first);
    const float second_magnitude = std::abs(second);
    const float maximum = std::max(first_magnitude, second_magnitude);
    const float minimum = std::min(first_magnitude, second_magnitude);

    if (maximum <= kRatioEpsilon) {
        return 1.0f;
    }

    return maximum / std::max(minimum, kRatioEpsilon);
}

float SignedRateDiscontinuityRatio(
    float first,
    float second,
    float deadband) {
    const float first_magnitude = std::abs(first);
    const float second_magnitude = std::abs(second);

    if (first_magnitude <= deadband && second_magnitude <= deadband) {
        return 1.0f;
    }

    if (std::abs(first - second) <= kRatioEpsilon) {
        return 1.0f;
    }

    const float minimum_magnitude =
        std::min(first_magnitude, second_magnitude);

    return 1.0f + std::abs(first - second) /
                      std::max(minimum_magnitude, deadband);
}

float WrapPi(float angle_radians) {
    return angle_radians -
           2.0f * kPi * std::round(angle_radians / (2.0f * kPi));
}

float FloorDistance(const Vec3& first, const Vec3& second) {
    const float dx = second.x - first.x;
    const float dz = second.z - first.z;
    return std::sqrt(dx * dx + dz * dz);
}

float RootFacingYaw(const Pose& pose) {
    if (pose.local_rotations.empty()) {
        return 0.0f;
    }
    const Vec3 forward =
        Rotate(pose.local_rotations.front(), {0.0f, 0.0f, 1.0f});
    return std::atan2(forward.x, forward.z);
}

float WrappedYawDelta(float current, float previous) {
    return WrapPi(current - previous);
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
        distance_sum +=
            (second_positions[joint] - first_positions[joint]).Norm();
    }
    return distance_sum / static_cast<float>(first_positions.size());
}

float Median(std::vector<float> values) {
    if (values.empty()) {
        return 0.0f;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 1) {
        return values[middle];
    }
    return 0.5f * (values[middle - 1] + values[middle]);
}

float MeanRootSpeed(
    const MotionClip& clip,
    int first_frame,
    int last_frame) {
    float distance_sum = 0.0f;
    for (int frame = first_frame + 1; frame <= last_frame; ++frame) {
        distance_sum += FloorDistance(
            clip.frames[static_cast<std::size_t>(frame - 1)].root_position,
            clip.frames[static_cast<std::size_t>(frame)].root_position);
    }
    const int interval_count = last_frame - first_frame;
    return interval_count > 0
               ? distance_sum * clip.frames_per_second /
                     static_cast<float>(interval_count)
               : 0.0f;
}

float MeanYawRate(
    const MotionClip& clip,
    int first_frame,
    int last_frame) {
    float yaw_delta_sum = 0.0f;
    for (int frame = first_frame + 1; frame <= last_frame; ++frame) {
        yaw_delta_sum += WrappedYawDelta(
            RootFacingYaw(clip.frames[static_cast<std::size_t>(frame)]),
            RootFacingYaw(clip.frames[static_cast<std::size_t>(frame - 1)]));
    }
    const int interval_count = last_frame - first_frame;
    return interval_count > 0
               ? yaw_delta_sum * clip.frames_per_second /
                     static_cast<float>(interval_count)
               : 0.0f;
}

float JointFloorDrift(
    const Skeleton& skeleton,
    const Pose& first,
    const Pose& second,
    int joint_index) {
    if (joint_index < 0 || joint_index >= skeleton.NumJoints()) {
        return 0.0f;
    }
    const std::vector<Vec3> first_positions =
        ComputeJointWorldPositions(skeleton, first);
    const std::vector<Vec3> second_positions =
        ComputeJointWorldPositions(skeleton, second);
    return FloorDistance(
        first_positions[static_cast<std::size_t>(joint_index)],
        second_positions[static_cast<std::size_t>(joint_index)]);
}

bool ContactStateAtFrame(
    const Skeleton& skeleton,
    const MotionClip& clip,
    int joint_index,
    int frame_index,
    const ContactDetectionSettings& settings) {
    if (joint_index < 0 || joint_index >= skeleton.NumJoints()) {
        return false;
    }
    const int previous_frame = std::max(frame_index - 1, 0);
    const int next_frame = std::min(frame_index + 1, clip.NumFrames() - 1);
    const std::vector<Vec3> previous_positions = ComputeJointWorldPositions(
        skeleton, clip.frames[static_cast<std::size_t>(previous_frame)]);
    const std::vector<Vec3> current_positions = ComputeJointWorldPositions(
        skeleton, clip.frames[static_cast<std::size_t>(frame_index)]);
    const std::vector<Vec3> next_positions = ComputeJointWorldPositions(
        skeleton, clip.frames[static_cast<std::size_t>(next_frame)]);

    const float dt = static_cast<float>(next_frame - previous_frame) /
                     clip.frames_per_second;
    const float speed =
        dt > kRatioEpsilon
            ? (next_positions[static_cast<std::size_t>(joint_index)] -
               previous_positions[static_cast<std::size_t>(joint_index)])
                      .Norm() /
                  dt
            : 0.0f;
    return current_positions[static_cast<std::size_t>(joint_index)].y <=
               settings.height_threshold &&
           speed <= settings.speed_threshold;
}

void UpdateContactEvidence(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const CyclicContinuityContext& context,
    CyclicContinuityRecord& record) {
    std::vector<int> contact_joints;
    if (context.left_foot_joint >= 0) {
        contact_joints.push_back(context.left_foot_joint);
    }
    if (context.right_foot_joint >= 0 &&
        context.right_foot_joint != context.left_foot_joint) {
        contact_joints.push_back(context.right_foot_joint);
    }
    if (!context.contact_settings.has_value() || contact_joints.empty()) {
        return;
    }

    const int last_frame = clip.NumFrames() - 1;

    record.has_contact_evidence = true;
    record.contact_state_matches = true;
    if (context.left_foot_joint >= 0) {
        record.contact_state_matches &=
            ContactStateAtFrame(
                skeleton, clip, context.left_foot_joint, 0,
                *context.contact_settings) ==
            ContactStateAtFrame(
                skeleton, clip, context.left_foot_joint, last_frame,
                *context.contact_settings);
    }
    if (context.right_foot_joint >= 0) {
        record.contact_state_matches &=
            ContactStateAtFrame(
                skeleton, clip, context.right_foot_joint, 0,
                *context.contact_settings) ==
            ContactStateAtFrame(
                skeleton, clip, context.right_foot_joint, last_frame,
                *context.contact_settings);
    }
}

CyclicContinuityClassification Classify(
    const CyclicContinuityRecord& record,
    const CyclicContinuityConfig& config) {
    const bool weak_pose =
        record.seam_step_ratio > config.pose_seam_ratio_threshold;

    const bool weak_root =
        record.root_speed_ratio > config.root_speed_ratio_threshold;

    const bool weak_yaw =
        record.yaw_rate_ratio > config.yaw_rate_ratio_threshold;

    const bool weak_contact =
        record.has_contact_evidence &&
        (!record.contact_state_matches ||
         record.max_contact_drift > config.contact_drift_threshold);

    if (weak_pose) {
        return CyclicContinuityClassification::kWeakPoseSeam;
    }
    if (weak_root) {
        return CyclicContinuityClassification::kWeakRootSpeed;
    }
    if (weak_yaw) {
        return CyclicContinuityClassification::kWeakYawRate;
    }
    if (weak_contact) {
        return CyclicContinuityClassification::kWeakContact;
    }
    return CyclicContinuityClassification::kStrong;
}

void RequireInputs(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const CyclicContinuityConfig& config) {
    if (skeleton.NumJoints() <= 0) {
        throw std::runtime_error(
            "MeasureCyclicContinuity: skeleton must not be empty");
    }
    clip.RequireNotEmpty("MeasureCyclicContinuity");
    if (clip.frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "MeasureCyclicContinuity: frames_per_second must be positive");
    }
    if (config.speed_window_frames < 1) {
        throw std::runtime_error(
            "MeasureCyclicContinuity: speed_window_frames must be positive");
    }
    if (config.yaw_rate_deadband < 0.0f) {
        throw std::runtime_error(
            "MeasureCyclicContinuity: yaw_rate_deadband must be non-negative");
    }
    for (const Pose& pose : clip.frames) {
        pose.RequireJointCount(
            skeleton.NumJoints(), "MeasureCyclicContinuity");
    }
}

}  // namespace

RigidTransform2D ComputeCycleDelta(const MotionClip& clip) {
    clip.RequireNotEmpty("ComputeCycleDelta");
    const Pose& first = clip.frames.front();
    const Pose& last = clip.frames.back();

    RigidTransform2D delta;
    delta.yaw = WrapPi(RootFacingYaw(last) - RootFacingYaw(first));

    float rotated_x = 0.0f;
    float rotated_z = 0.0f;
    delta.RotateFloor(
        first.root_position.x, first.root_position.z, rotated_x, rotated_z);

    delta.dx = last.root_position.x - rotated_x;
    delta.dz = last.root_position.z - rotated_z;
    return delta;
}

CyclicContinuityRecord MeasureCyclicContinuity(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const CyclicContinuityContext& context,
    const CyclicContinuityConfig& config) {
    RequireInputs(skeleton, clip, config);

    CyclicContinuityRecord record;
    record.cycle_delta = ComputeCycleDelta(clip);

    if (clip.NumFrames() < 3) {
        record.classification =
            CyclicContinuityClassification::kInsufficientData;
        return record;
    }

    std::vector<float> in_clip_steps;
    in_clip_steps.reserve(static_cast<std::size_t>(clip.NumFrames() - 1));
    for (int frame = 1; frame < clip.NumFrames(); ++frame) {
        in_clip_steps.push_back(MeanJointDistance(
            skeleton,
            clip.frames[static_cast<std::size_t>(frame - 1)],
            clip.frames[static_cast<std::size_t>(frame)]));
    }
    record.median_step = Median(std::move(in_clip_steps));

    const Pose next_cycle_first = record.cycle_delta.Apply(clip.frames.front());
    const Pose& seam_last = clip.frames.back();
    record.seam_step =
        MeanJointDistance(skeleton, seam_last, next_cycle_first);
    record.seam_step_ratio =
        record.median_step > kRatioEpsilon
            ? record.seam_step / record.median_step
            : (record.seam_step <= kRatioEpsilon
                   ? 0.0f
                   : std::numeric_limits<float>::infinity());

    const int last_frame = clip.NumFrames() - 1;
    const int window = std::min(config.speed_window_frames, last_frame);
    record.pre_root_speed =
        MeanRootSpeed(clip, last_frame - window, last_frame);
    record.seam_root_speed =
        FloorDistance(seam_last.root_position, next_cycle_first.root_position) *
        clip.frames_per_second;
    record.post_root_speed = MeanRootSpeed(clip, 0, window);
    record.root_speed_ratio =
        SymmetricPositiveRatio(record.pre_root_speed, record.post_root_speed);

    record.pre_yaw_rate = MeanYawRate(clip, last_frame - window, last_frame);
    record.seam_yaw_rate = WrappedYawDelta(
                               RootFacingYaw(next_cycle_first),
                               RootFacingYaw(seam_last)) *
                           clip.frames_per_second;
    record.post_yaw_rate = MeanYawRate(clip, 0, window);
    record.yaw_rate_ratio = SignedRateDiscontinuityRatio(
        record.pre_yaw_rate, record.post_yaw_rate, config.yaw_rate_deadband);

    record.left_foot_drift =
        JointFloorDrift(skeleton, seam_last, next_cycle_first,
                        context.left_foot_joint);
    record.right_foot_drift =
        JointFloorDrift(skeleton, seam_last, next_cycle_first,
                        context.right_foot_joint);
    record.max_contact_drift =
        std::max(record.left_foot_drift, record.right_foot_drift);
    UpdateContactEvidence(skeleton, clip, context, record);

    record.classification = Classify(record, config);
    return record;
}

const char* CyclicContinuityClassificationName(
    CyclicContinuityClassification classification) {
    switch (classification) {
        case CyclicContinuityClassification::kStrong:
            return "strong";
        case CyclicContinuityClassification::kWeakPoseSeam:
            return "weak_pose_seam";
        case CyclicContinuityClassification::kWeakRootSpeed:
            return "weak_root_speed";
        case CyclicContinuityClassification::kWeakYawRate:
            return "weak_yaw_rate";
        case CyclicContinuityClassification::kWeakContact:
            return "weak_contact";
        case CyclicContinuityClassification::kInsufficientData:
            return "insufficient_data";
    }
    throw std::runtime_error(
        "CyclicContinuityClassificationName: unsupported classification");
}

}  // namespace pmg
