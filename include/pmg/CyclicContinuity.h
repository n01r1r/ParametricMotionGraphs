#pragma once

#include "pmg/ContactDetection.h"
#include "pmg/MotionClip.h"
#include "pmg/RigidTransform2D.h"
#include "pmg/Skeleton.h"

#include <optional>

namespace pmg {

enum class CyclicContinuityClassification {
    kStrong,
    kWeakPoseSeam,
    kWeakRootSpeed,
    kWeakYawRate,
    kWeakContact,
    kInsufficientData,
};

struct CyclicContinuityConfig {
    // Number of adjacent frame intervals used on each side of the seam for
    // root-speed and yaw-rate estimates.
    int speed_window_frames = 3;

    // Dimensionless thresholds. Pose seam compares mean joint distance across
    // the cycle seam to the median in-clip adjacent-frame step. Root speed
    // compares pre-seam and post-seam in-clip speeds. Yaw rate compares
    // pre-seam and post-seam signed in-clip rates with a deadband in
    // radians/second. Raw seam root/yaw rates are diagnostic fields only.
    float pose_seam_ratio_threshold = 2.0f;
    float root_speed_ratio_threshold = 1.3f;
    float yaw_rate_ratio_threshold = 1.5f;
    float yaw_rate_deadband = 0.05f;

    // Native motion-corpus distance units.
    float contact_drift_threshold = 1.0f;
};

struct CyclicContinuityContext {
    int left_foot_joint = -1;
    int right_foot_joint = -1;
    std::optional<ContactDetectionSettings> contact_settings;
};

struct CyclicContinuityRecord {
    float seam_step = 0.0f;
    float median_step = 0.0f;
    float seam_step_ratio = 0.0f;

    float pre_root_speed = 0.0f;
    float seam_root_speed = 0.0f;
    float post_root_speed = 0.0f;
    float root_speed_ratio = 1.0f;

    float pre_yaw_rate = 0.0f;
    float seam_yaw_rate = 0.0f;
    float post_yaw_rate = 0.0f;
    float yaw_rate_ratio = 1.0f;

    float left_foot_drift = 0.0f;
    float right_foot_drift = 0.0f;
    float max_contact_drift = 0.0f;
    bool has_contact_evidence = false;
    bool contact_state_matches = false;

    RigidTransform2D cycle_delta;
    CyclicContinuityClassification classification =
        CyclicContinuityClassification::kInsufficientData;
};

// Rigid floor transform that maps the first frame of a cyclic clip onto its
// final frame. Runtime cycle folding and cyclic diagnostics share this helper
// so seam measurements use exactly the placement rule used during streaming.
RigidTransform2D ComputeCycleDelta(const MotionClip& clip);

// Measures the seam from the last clip frame to the first frame of the next
// cycle, with the first frame transformed by ComputeCycleDelta(clip).
//
// Inputs:
// - skeleton: joint hierarchy for world-position pose comparison.
// - clip: cyclic clip in local coordinates; at least three frames for a valid
//   record.
// - context: optional contact joints/settings. Drift is still emitted when
//   valid foot joints are supplied without contact settings.
//
// Outputs:
// - ratios are dimensionless.
// - speeds use native distance units/second.
// - yaw rates use radians/second.
CyclicContinuityRecord MeasureCyclicContinuity(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const CyclicContinuityContext& context = {},
    const CyclicContinuityConfig& config = {});

const char* CyclicContinuityClassificationName(
    CyclicContinuityClassification classification);

}  // namespace pmg
