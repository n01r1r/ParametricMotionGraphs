#pragma once

#include "pmg/ContactDetection.h"
#include "pmg/Pose.h"
#include "pmg/Skeleton.h"

#include <optional>
#include <span>

namespace pmg {

enum class TransitionQualityClassification {
    kSmooth,
    kPosePop,
    kRootSpeedDiscontinuity,
    kYawRateDiscontinuity,
    kContactMismatch,
    kCyclicSeamMismatch,
    kInsufficientData,
};

enum class TransitionContactState {
    kUnknown,
    kNotInContact,
    kInContact,
};

struct TransitionQualityConfig {
    // Number of frame intervals sampled on each side of transition_pose_index.
    int frames_before = 3;
    int frames_after = 3;

    // Dimensionless classification thresholds. Root speed uses symmetric
    // magnitude ratio. Yaw rate uses signed change, so +omega -> -omega is
    // discontinuous even when magnitudes match.
    float pose_pop_ratio_threshold = 2.0f;
    float root_speed_ratio_threshold = 1.3f;
    float yaw_rate_ratio_threshold = 1.3f;
    // Signed yaw rates with magnitude at or below this rad/s value are treated
    // as stationary noise when computing yaw_rate_ratio.
    float yaw_rate_deadband = 0.05f;
    float contact_drift_threshold = 1.0f;
};

struct TransitionQualityContext {
    // Runtime sampling rate used to convert frame deltas to units/s and rad/s.
    float frames_per_second = 30.0f;

    // Build-time point-cloud distance D for the scheduled transition.
    float transition_distance = 0.0f;

    // Median mean-joint frame step over the enclosing run. Required for the
    // existing local pop ratio; zero means the ratio is unavailable.
    float baseline_median_step = 0.0f;

    // Optional contact joints. Drift remains observable without contact
    // settings, but contact states are then kUnknown.
    int left_foot_joint = -1;
    int right_foot_joint = -1;
    std::optional<ContactDetectionSettings> contact_settings;

    // True only when the measured window straddles a raw cyclic clip seam.
    // Reserved for future CyclicContinuity wiring; current runtime CSV
    // generation leaves this false.
    bool cyclic_seam = false;
};

struct TransitionQualityRecord {
    float transition_distance = 0.0f;
    float local_max_step = 0.0f;
    float local_pop_ratio = 0.0f;

    float pre_root_speed = 0.0f;
    float post_root_speed = 0.0f;
    float root_speed_ratio = 1.0f;

    float pre_yaw_rate = 0.0f;
    float post_yaw_rate = 0.0f;
    float yaw_rate_ratio = 1.0f;

    float left_foot_drift = 0.0f;
    float right_foot_drift = 0.0f;
    float max_contact_drift = 0.0f;

    TransitionContactState left_contact_before =
        TransitionContactState::kUnknown;
    TransitionContactState left_contact_after =
        TransitionContactState::kUnknown;
    TransitionContactState right_contact_before =
        TransitionContactState::kUnknown;
    TransitionContactState right_contact_after =
        TransitionContactState::kUnknown;

    TransitionQualityClassification classification =
        TransitionQualityClassification::kInsufficientData;
};

// Measures transition-local continuity from world-space poses.
//
// transition_pose_index is the first pose captured after the transition became
// active. The default window uses three frame intervals before and after that
// pose. Root speed uses floor-plane displacement; yaw rate uses wrapped root
// facing deltas; foot drift uses floor-plane joint displacement across the
// sampled window. Contact states are known only when contact_settings and valid
// foot joint indices are supplied.
TransitionQualityRecord MeasureTransitionQuality(
    const Skeleton& skeleton,
    std::span<const Pose> world_poses,
    int transition_pose_index,
    const TransitionQualityContext& context,
    const TransitionQualityConfig& config = {});

const char* TransitionQualityClassificationName(
    TransitionQualityClassification classification);

const char* TransitionContactStateName(TransitionContactState state);

}  // namespace pmg
