#include "pmg/TransitionQuality.h"

#include "pmg/ForwardKinematics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace pmg {

namespace {

constexpr float kRatioEpsilon = 1.0e-6f;

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
    float delta = current - previous;
    while (delta > kPi) {
        delta -= 2.0f * kPi;
    }
    while (delta < -kPi) {
        delta += 2.0f * kPi;
    }
    return delta;
}

float SymmetricMagnitudeRatio(float first, float second) {
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

    const float minimum_magnitude =
        std::min(first_magnitude, second_magnitude);
    if (std::abs(first - second) <= kRatioEpsilon) {
        return 1.0f;
    }
    return 1.0f + std::abs(first - second) /
                      std::max(minimum_magnitude, deadband);
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

float MeanRootSpeed(
    std::span<const Pose> poses,
    int first_pose,
    int last_pose,
    float frames_per_second) {
    float distance_sum = 0.0f;
    for (int pose = first_pose + 1; pose <= last_pose; ++pose) {
        distance_sum += FloorDistance(
            poses[static_cast<std::size_t>(pose - 1)].root_position,
            poses[static_cast<std::size_t>(pose)].root_position);
    }
    const int interval_count = last_pose - first_pose;
    return interval_count > 0
               ? distance_sum * frames_per_second /
                     static_cast<float>(interval_count)
               : 0.0f;
}

float MeanYawRate(
    std::span<const Pose> poses,
    int first_pose,
    int last_pose,
    float frames_per_second) {
    float yaw_delta_sum = 0.0f;
    for (int pose = first_pose + 1; pose <= last_pose; ++pose) {
        yaw_delta_sum += WrappedYawDelta(
            RootFacingYaw(poses[static_cast<std::size_t>(pose)]),
            RootFacingYaw(poses[static_cast<std::size_t>(pose - 1)]));
    }
    const int interval_count = last_pose - first_pose;
    return interval_count > 0
               ? yaw_delta_sum * frames_per_second /
                     static_cast<float>(interval_count)
               : 0.0f;
}

float JointFloorDrift(
    const Skeleton& skeleton,
    std::span<const Pose> poses,
    int first_pose,
    int last_pose,
    int joint_index) {
    if (joint_index < 0 || joint_index >= skeleton.NumJoints()) {
        return 0.0f;
    }

    float drift = 0.0f;
    Vec3 previous = ComputeJointWorldPositions(
        skeleton, poses[static_cast<std::size_t>(first_pose)])
                        [static_cast<std::size_t>(joint_index)];
    for (int pose = first_pose + 1; pose <= last_pose; ++pose) {
        const Vec3 current = ComputeJointWorldPositions(
            skeleton, poses[static_cast<std::size_t>(pose)])
                                 [static_cast<std::size_t>(joint_index)];
        drift += FloorDistance(previous, current);
        previous = current;
    }
    return drift;
}

TransitionContactState ContactStateAtFrame(
    const std::vector<ContactInterval>& contacts,
    int joint_index,
    int frame_index) {
    for (const ContactInterval& interval : contacts) {
        if (interval.joint_index == joint_index &&
            frame_index >= interval.first_frame &&
            frame_index <= interval.last_frame) {
            return TransitionContactState::kInContact;
        }
    }
    return TransitionContactState::kNotInContact;
}

bool IsInContact(TransitionContactState state) {
    return state == TransitionContactState::kInContact;
}

TransitionQualityClassification Classify(
    const TransitionQualityRecord& record,
    const TransitionQualityContext& context,
    const TransitionQualityConfig& config) {
    const bool left_contact_drift =
        IsInContact(record.left_contact_before) &&
        IsInContact(record.left_contact_after) &&
        record.left_foot_drift > config.contact_drift_threshold;
    const bool right_contact_drift =
        IsInContact(record.right_contact_before) &&
        IsInContact(record.right_contact_after) &&
        record.right_foot_drift > config.contact_drift_threshold;
    const bool contact_mismatch = left_contact_drift || right_contact_drift;

    const float pose_score =
        config.pose_pop_ratio_threshold > 0.0f
            ? record.local_pop_ratio / config.pose_pop_ratio_threshold
            : 0.0f;
    const float speed_score =
        config.root_speed_ratio_threshold > 0.0f
            ? record.root_speed_ratio /
                  config.root_speed_ratio_threshold
            : 0.0f;
    const float yaw_score =
        config.yaw_rate_ratio_threshold > 0.0f
            ? record.yaw_rate_ratio / config.yaw_rate_ratio_threshold
            : 0.0f;

    if (context.cyclic_seam &&
        (pose_score > 1.0f || speed_score > 1.0f ||
         yaw_score > 1.0f || contact_mismatch)) {
        return TransitionQualityClassification::kCyclicSeamMismatch;
    }

    if (contact_mismatch) {
        return TransitionQualityClassification::kContactMismatch;
    }

    if (pose_score <= 1.0f && speed_score <= 1.0f &&
        yaw_score <= 1.0f) {
        return TransitionQualityClassification::kSmooth;
    }

    if (pose_score >= speed_score && pose_score >= yaw_score) {
        return TransitionQualityClassification::kPosePop;
    }
    if (yaw_score >= speed_score) {
        return TransitionQualityClassification::kYawRateDiscontinuity;
    }
    return TransitionQualityClassification::kRootSpeedDiscontinuity;
}

void RequireInputs(
    const Skeleton& skeleton,
    std::span<const Pose> poses,
    int transition_pose_index,
    const TransitionQualityContext& context,
    const TransitionQualityConfig& config) {
    if (skeleton.NumJoints() <= 0) {
        throw std::runtime_error(
            "MeasureTransitionQuality: skeleton must not be empty");
    }
    if (poses.empty()) {
        throw std::runtime_error(
            "MeasureTransitionQuality: pose window must not be empty");
    }
    if (transition_pose_index < 0 ||
        transition_pose_index >= static_cast<int>(poses.size())) {
        throw std::runtime_error(
            "MeasureTransitionQuality: transition pose index is out of range");
    }
    if (context.frames_per_second <= 0.0f) {
        throw std::runtime_error(
            "MeasureTransitionQuality: frames_per_second must be positive");
    }
    if (config.frames_before < 1 || config.frames_after < 1) {
        throw std::runtime_error(
            "MeasureTransitionQuality: before/after frame counts must be positive");
    }
    if (config.yaw_rate_deadband < 0.0f) {
        throw std::runtime_error(
            "MeasureTransitionQuality: yaw_rate_deadband must be non-negative");
    }
    for (const Pose& pose : poses) {
        pose.RequireJointCount(
            skeleton.NumJoints(), "MeasureTransitionQuality");
    }
}

}  // namespace

TransitionQualityRecord MeasureTransitionQuality(
    const Skeleton& skeleton,
    std::span<const Pose> world_poses,
    int transition_pose_index,
    const TransitionQualityContext& context,
    const TransitionQualityConfig& config) {
    RequireInputs(
        skeleton, world_poses, transition_pose_index, context, config);

    TransitionQualityRecord record;
    record.transition_distance = context.transition_distance;

    const int first_pose =
        std::max(0, transition_pose_index - config.frames_before);
    const int last_pose = std::min(
        static_cast<int>(world_poses.size()) - 1,
        transition_pose_index + config.frames_after);

    if (transition_pose_index - first_pose < 1 ||
        last_pose - transition_pose_index < 1) {
        record.classification =
            TransitionQualityClassification::kInsufficientData;
        return record;
    }

    for (int pose = first_pose + 1; pose <= last_pose; ++pose) {
        record.local_max_step = std::max(
            record.local_max_step,
            MeanJointDistance(
                skeleton,
                world_poses[static_cast<std::size_t>(pose - 1)],
                world_poses[static_cast<std::size_t>(pose)]));
    }
    if (context.baseline_median_step > kRatioEpsilon) {
        record.local_pop_ratio =
            record.local_max_step / context.baseline_median_step;
    }

    record.pre_root_speed = MeanRootSpeed(
        world_poses, first_pose, transition_pose_index,
        context.frames_per_second);
    record.post_root_speed = MeanRootSpeed(
        world_poses, transition_pose_index, last_pose,
        context.frames_per_second);
    record.root_speed_ratio = SymmetricMagnitudeRatio(
        record.pre_root_speed, record.post_root_speed);

    record.pre_yaw_rate = MeanYawRate(
        world_poses, first_pose, transition_pose_index,
        context.frames_per_second);
    record.post_yaw_rate = MeanYawRate(
        world_poses, transition_pose_index, last_pose,
        context.frames_per_second);
    record.yaw_rate_ratio = SignedRateDiscontinuityRatio(
        record.pre_yaw_rate, record.post_yaw_rate,
        config.yaw_rate_deadband);

    record.left_foot_drift = JointFloorDrift(
        skeleton, world_poses, first_pose, last_pose,
        context.left_foot_joint);
    record.right_foot_drift = JointFloorDrift(
        skeleton, world_poses, first_pose, last_pose,
        context.right_foot_joint);
    record.max_contact_drift =
        std::max(record.left_foot_drift, record.right_foot_drift);

    if (context.contact_settings.has_value()) {
        MotionClip window_clip;
        window_clip.frames_per_second = context.frames_per_second;
        window_clip.frames.assign(
            world_poses.begin() + first_pose,
            world_poses.begin() + last_pose + 1);

        std::vector<int> contact_joints;
        if (context.left_foot_joint >= 0) {
            contact_joints.push_back(context.left_foot_joint);
        }
        if (context.right_foot_joint >= 0 &&
            context.right_foot_joint != context.left_foot_joint) {
            contact_joints.push_back(context.right_foot_joint);
        }

        if (!contact_joints.empty()) {
            const std::vector<ContactInterval> contacts = DetectContacts(
                skeleton, window_clip, contact_joints,
                *context.contact_settings);
            const int local_transition =
                transition_pose_index - first_pose;
            const int before_frame = std::max(0, local_transition - 1);
            const int after_frame = std::min(
                window_clip.NumFrames() - 1, local_transition + 1);

            if (context.left_foot_joint >= 0) {
                record.left_contact_before = ContactStateAtFrame(
                    contacts, context.left_foot_joint, before_frame);
                record.left_contact_after = ContactStateAtFrame(
                    contacts, context.left_foot_joint, after_frame);
            }
            if (context.right_foot_joint >= 0) {
                record.right_contact_before = ContactStateAtFrame(
                    contacts, context.right_foot_joint, before_frame);
                record.right_contact_after = ContactStateAtFrame(
                    contacts, context.right_foot_joint, after_frame);
            }
        }
    }

    record.classification = Classify(record, context, config);
    return record;
}

const char* TransitionQualityClassificationName(
    TransitionQualityClassification classification) {
    switch (classification) {
        case TransitionQualityClassification::kSmooth:
            return "smooth";
        case TransitionQualityClassification::kPosePop:
            return "pose_pop";
        case TransitionQualityClassification::kRootSpeedDiscontinuity:
            return "root_speed_discontinuity";
        case TransitionQualityClassification::kYawRateDiscontinuity:
            return "yaw_rate_discontinuity";
        case TransitionQualityClassification::kContactMismatch:
            return "contact_mismatch";
        case TransitionQualityClassification::kCyclicSeamMismatch:
            return "cyclic_seam_mismatch";
        case TransitionQualityClassification::kInsufficientData:
            return "insufficient_data";
    }
    throw std::runtime_error(
        "TransitionQualityClassificationName: unsupported classification");
}

const char* TransitionContactStateName(TransitionContactState state) {
    switch (state) {
        case TransitionContactState::kUnknown:
            return "unknown";
        case TransitionContactState::kNotInContact:
            return "not_in_contact";
        case TransitionContactState::kInContact:
            return "in_contact";
    }
    throw std::runtime_error(
        "TransitionContactStateName: unsupported contact state");
}

}  // namespace pmg
