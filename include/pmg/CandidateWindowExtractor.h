#pragma once

#include "pmg/MotionClip.h"
#include "pmg/Skeleton.h"

#include <string>
#include <iosfwd>
#include <vector>

namespace pmg {

struct CandidateMotionWindow {
    int start_frame = 0;
    int end_frame = 0;  // inclusive
    float score = 0.0f;  // lower is better
    std::string reason;
    float root_displacement = 0.0f;  // BVH native distance units
    float heading_delta = 0.0f;  // radians, wrapped to [-pi, pi]
};

struct CandidateWindowExtractionConfig {
    int min_length_frames = 20;
    int max_length_frames = 120;
    int stride_frames = 1;
    int top_k = 10;
    bool prefer_cyclic = true;
    int endpoint_window_frames = 3;
    float max_normalized_pose_distance = 25.0f;
    float max_heading_delta_radians = 0.8f;
    float heading_score_weight = 1.0f;
    // Opt-in locomotion filter: drop windows whose travel speed
    // (root_displacement / duration) is below this, in BVH native units/sec.
    // 0 disables it -> pure loop-seam ranking (the legacy default), which favors
    // near-standing low-seam windows. Set >0 to surface real locomotion windows.
    float min_window_speed = 0.0f;
};

// Offline proposal only. Scans inclusive frame windows and ranks endpoint
// similarity under rigid floor alignment. Does not infer motion families or
// motion-space membership. Invalid config throws; clips shorter than the
// configured minimum return no candidates.
std::vector<CandidateMotionWindow> ExtractCandidateMotionWindows(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const CandidateWindowExtractionConfig& config = {});

// Writes a human-inspectable, spec-authoring intermediate. This preserves
// provenance only; it does not create GraphSpec nodes or examples.
void WriteCandidateWindowsJson(
    std::ostream& output,
    const std::string& source_bvh_path,
    const CandidateWindowExtractionConfig& config,
    const std::vector<CandidateMotionWindow>& candidates);

}  // namespace pmg
