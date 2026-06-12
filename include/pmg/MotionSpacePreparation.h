#pragma once

#include "pmg/ContactDetection.h"
#include "pmg/GraphSpec.h"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace pmg {

struct MotionSpacePreparationConfig {
    std::string default_cycle_joint;
    std::vector<std::string> default_contact_joints{"LeftAnkle", "RightAnkle"};
    int default_min_contact_frames = 3;
    bool default_dtw_refine = true;
    float calibration_frames_per_second = 30.0f;
    float skeleton_offset_tolerance = 1.0e-4f;
};

// Snapshots produced by one preparation pass. "authored" includes optional
// cycle extraction. "production" additionally includes the registration,
// DTW refinement, and ParameterCalibration declared by the GraphSpec.
struct PreparedMotionSpace {
    NodeRegistrationMetadata registration;
    std::vector<int> contact_joint_indices;
    std::optional<ContactDetectionSettings> contact_settings;
    ParametricMotionSpace authored;
    std::optional<ParametricMotionSpace> contact_registered;
    std::optional<ParametricMotionSpace> dtw_refined;
    ParametricMotionSpace production;
};

struct PreparedMotionSpaces {
    Skeleton skeleton;
    std::map<std::string, PreparedMotionSpace> nodes;
    std::vector<std::string> source_bvh_paths;

    const PreparedMotionSpace& Node(const std::string& name) const;
};

// Loads every GraphSpec example once, enforces one compatible Skeleton, and
// applies cycle extraction, contact Registration, DTW refinement, and
// ParameterCalibration in production order. Stage snapshots let diagnostics
// compare authored and registered behavior without reimplementing preparation.
PreparedMotionSpaces PrepareMotionSpaces(const GraphSpec& spec,
                                         const MotionSpacePreparationConfig& config = {});

}  // namespace pmg
