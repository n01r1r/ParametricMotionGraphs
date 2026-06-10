#include "pmg/ContactDetection.h"

#include "pmg/ForwardKinematics.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

namespace pmg {

namespace {

constexpr float kHeightBandFraction = 0.2f;
constexpr float kSpeedFraction = 0.25f;

// World position of one joint per frame.
std::vector<Vec3> JointTrack(
    const Skeleton& skeleton,
    const MotionClip& clip,
    int joint_index) {
    std::vector<Vec3> track;
    track.reserve(clip.frames.size());
    for (const Pose& pose : clip.frames) {
        track.push_back(ComputeJointWorldPositions(skeleton, pose)[joint_index]);
    }
    return track;
}

// Central-difference world speed per frame; one-sided at the clip boundaries.
std::vector<float> JointSpeeds(const std::vector<Vec3>& track, float frames_per_second) {
    const int frame_count = static_cast<int>(track.size());
    std::vector<float> speeds(track.size(), 0.0f);
    if (frame_count < 2) {
        return speeds;
    }

    for (int frame = 0; frame < frame_count; ++frame) {
        const int previous = std::max(frame - 1, 0);
        const int next = std::min(frame + 1, frame_count - 1);
        const float dt = static_cast<float>(next - previous) / frames_per_second;
        speeds[frame] = (track[next] - track[previous]).Norm() / dt;
    }
    return speeds;
}

void RequireContactInputs(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const std::vector<int>& contact_joints,
    const char* context) {
    clip.RequireNotEmpty(context);
    if (clip.NumJoints() != skeleton.NumJoints()) {
        throw std::runtime_error(std::string(context) + ": clip/skeleton joint count mismatch");
    }
    if (contact_joints.empty()) {
        throw std::runtime_error(std::string(context) + ": no contact joints given");
    }
    for (const int joint_index : contact_joints) {
        skeleton.RequireValidJointIndex(joint_index, context);
    }
}

}  // namespace

float ContactInterval::StrikePhase(int clip_frame_count) const {
    if (clip_frame_count < 2) {
        return 0.0f;
    }
    return static_cast<float>(first_frame) / static_cast<float>(clip_frame_count - 1);
}

float ContactInterval::LiftPhase(int clip_frame_count) const {
    if (clip_frame_count < 2) {
        return 0.0f;
    }
    return static_cast<float>(last_frame) / static_cast<float>(clip_frame_count - 1);
}

ContactDetectionSettings EstimateContactSettings(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const std::vector<int>& contact_joints) {
    RequireContactInputs(skeleton, clip, contact_joints, "EstimateContactSettings");

    float lowest_height = std::numeric_limits<float>::max();
    float highest_height = std::numeric_limits<float>::lowest();
    float speed_sum = 0.0f;
    int speed_samples = 0;

    for (const int joint_index : contact_joints) {
        const std::vector<Vec3> track = JointTrack(skeleton, clip, joint_index);
        const std::vector<float> speeds = JointSpeeds(track, clip.frames_per_second);
        for (const Vec3& position : track) {
            lowest_height = std::min(lowest_height, position.y);
            highest_height = std::max(highest_height, position.y);
        }
        for (const float speed : speeds) {
            speed_sum += speed;
            ++speed_samples;
        }
    }

    ContactDetectionSettings settings;
    settings.height_threshold =
        lowest_height + kHeightBandFraction * (highest_height - lowest_height);
    settings.speed_threshold =
        speed_samples > 0 ? kSpeedFraction * (speed_sum / static_cast<float>(speed_samples))
                          : 0.0f;
    return settings;
}

std::vector<ContactInterval> DetectContacts(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const std::vector<int>& contact_joints,
    const ContactDetectionSettings& settings) {
    RequireContactInputs(skeleton, clip, contact_joints, "DetectContacts");

    std::vector<ContactInterval> intervals;
    const int frame_count = clip.NumFrames();

    for (const int joint_index : contact_joints) {
        const std::vector<Vec3> track = JointTrack(skeleton, clip, joint_index);
        const std::vector<float> speeds = JointSpeeds(track, clip.frames_per_second);

        int run_start = -1;
        for (int frame = 0; frame <= frame_count; ++frame) {
            const bool in_contact =
                frame < frame_count &&
                track[frame].y <= settings.height_threshold &&
                speeds[frame] <= settings.speed_threshold;

            if (in_contact && run_start < 0) {
                run_start = frame;
            } else if (!in_contact && run_start >= 0) {
                const int run_length = frame - run_start;
                if (run_length >= settings.min_contact_frames) {
                    intervals.push_back({joint_index, run_start, frame - 1});
                }
                run_start = -1;
            }
        }
    }

    std::sort(intervals.begin(), intervals.end(),
              [](const ContactInterval& left, const ContactInterval& right) {
                  if (left.first_frame != right.first_frame) {
                      return left.first_frame < right.first_frame;
                  }
                  return left.joint_index < right.joint_index;
              });
    return intervals;
}

}  // namespace pmg
