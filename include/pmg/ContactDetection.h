#pragma once

#include "pmg/MotionClip.h"
#include "pmg/Skeleton.h"

#include <vector>

namespace pmg {

// Thresholds for classifying a joint frame as in contact with the floor. A
// frame is a contact when the joint's world height (Y) and world speed are
// both at or below their thresholds. Units follow the clip's own units.
struct ContactDetectionSettings {
    float height_threshold = 0.0f;
    float speed_threshold = 0.0f;
    // Runs shorter than this many frames are discarded as noise.
    int min_contact_frames = 2;
};

// A maximal run of contact frames for one joint. Frames are inclusive.
struct ContactInterval {
    int joint_index = -1;
    int first_frame = 0;
    int last_frame = 0;

    // Normalized phases of the strike (contact begins) and lift (contact
    // ends) within a clip of `clip_frame_count` frames.
    float StrikePhase(int clip_frame_count) const;
    float LiftPhase(int clip_frame_count) const;
};

// Heuristic thresholds from the clip itself: height threshold sits just above
// each contact joint's lowest height, speed threshold well below the joint's
// typical swing speed. A reasonable default for locomotion-like clips; hand
// authored settings win for unusual data.
ContactDetectionSettings EstimateContactSettings(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const std::vector<int>& contact_joints);

// Detect contact intervals for the given joints. Result is sorted by
// first_frame, then joint index, so corresponding contacts across structurally
// similar clips (e.g. all walk cycles) appear in the same order.
std::vector<ContactInterval> DetectContacts(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const std::vector<int>& contact_joints,
    const ContactDetectionSettings& settings);

}  // namespace pmg
