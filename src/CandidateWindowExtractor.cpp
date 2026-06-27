#include "pmg/CandidateWindowExtractor.h"

#include "pmg/GoalDirectedLocomotion.h"
#include "pmg/MotionDistance.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace pmg {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float WrappedAngle(float radians) {
    return std::remainder(radians, 2.0f * kPi);
}

void Validate(const CandidateWindowExtractionConfig& config) {
    if (config.min_length_frames < 2 || config.max_length_frames < config.min_length_frames) {
        throw std::invalid_argument("candidate extraction: frame lengths must satisfy 2 <= min <= max");
    }
    if (config.stride_frames < 1 || config.top_k < 1 || config.endpoint_window_frames < 1) {
        throw std::invalid_argument("candidate extraction: stride, top_k, and endpoint window must be positive");
    }
    if (config.max_normalized_pose_distance < 0.0f ||
        config.max_heading_delta_radians < 0.0f || config.heading_score_weight < 0.0f) {
        throw std::invalid_argument("candidate extraction: thresholds and score weights must be non-negative");
    }
}

std::string JsonString(const std::string& text) {
    std::ostringstream escaped;
    escaped << '"';
    for (const unsigned char character : text) {
        switch (character) {
            case '"': escaped << "\\\""; break;
            case '\\': escaped << "\\\\"; break;
            case '\n': escaped << "\\n"; break;
            case '\r': escaped << "\\r"; break;
            case '\t': escaped << "\\t"; break;
            default:
                if (character < 0x20) {
                    escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                            << static_cast<int>(character) << std::dec;
                } else {
                    escaped << static_cast<char>(character);
                }
        }
    }
    return escaped.str() + '"';
}

}  // namespace

std::vector<CandidateMotionWindow> ExtractCandidateMotionWindows(
    const Skeleton& skeleton,
    const MotionClip& clip,
    const CandidateWindowExtractionConfig& config) {
    Validate(config);
    if (clip.NumFrames() < config.min_length_frames || skeleton.NumJoints() == 0) {
        return {};
    }

    std::vector<CandidateMotionWindow> candidates;
    const int max_length = std::min(config.max_length_frames, clip.NumFrames());
    for (int length = config.min_length_frames; length <= max_length;
         length += config.stride_frames) {
        for (int start = 0; start + length <= clip.NumFrames();
             start += config.stride_frames) {
            const int end = start + length - 1;
            const PointCloud first = MotionDistance::BuildPointCloud(
                skeleton, clip, start, config.endpoint_window_frames);
            const PointCloud last = MotionDistance::BuildPointCloud(
                skeleton, clip, end, config.endpoint_window_frames);
            const float pose_distance =
                MotionDistance::AlignedPointCloudDistance(first, last).distance /
                static_cast<float>(first.Size());
            const float heading_delta = WrappedAngle(
                PoseFacingYaw(clip.frames[static_cast<std::size_t>(end)]) -
                PoseFacingYaw(clip.frames[static_cast<std::size_t>(start)]));

            if (config.prefer_cyclic &&
                (pose_distance > config.max_normalized_pose_distance ||
                 std::abs(heading_delta) > config.max_heading_delta_radians)) {
                continue;
            }

            const Vec3 root_delta = clip.frames[static_cast<std::size_t>(end)].root_position -
                                    clip.frames[static_cast<std::size_t>(start)].root_position;
            const float root_displacement =
                std::sqrt(root_delta.x * root_delta.x + root_delta.z * root_delta.z);
            if (config.min_window_speed > 0.0f && clip.frames_per_second > 0.0f) {
                const float duration = static_cast<float>(length) / clip.frames_per_second;
                if (duration <= 0.0f || root_displacement / duration < config.min_window_speed) {
                    continue;
                }
            }
            CandidateMotionWindow candidate;
            candidate.start_frame = start;
            candidate.end_frame = end;
            candidate.score = pose_distance +
                              config.heading_score_weight * heading_delta * heading_delta;
            candidate.root_displacement = root_displacement;
            candidate.heading_delta = heading_delta;
            std::ostringstream reason;
            reason << "aligned endpoint pose=" << pose_distance
                   << ", heading delta=" << heading_delta << " rad";
            candidate.reason = reason.str();
            candidates.push_back(std::move(candidate));
        }
    }

    std::stable_sort(candidates.begin(), candidates.end(),
        [](const CandidateMotionWindow& left, const CandidateMotionWindow& right) {
            if (left.score != right.score) return left.score < right.score;
            if (left.start_frame != right.start_frame) return left.start_frame < right.start_frame;
            return left.end_frame < right.end_frame;
        });
    if (static_cast<int>(candidates.size()) > config.top_k) {
        candidates.resize(static_cast<std::size_t>(config.top_k));
    }
    return candidates;
}

void WriteCandidateWindowsJson(
    std::ostream& output,
    const std::string& source_bvh_path,
    const CandidateWindowExtractionConfig& config,
    const std::vector<CandidateMotionWindow>& candidates) {
    output << "{\n  \"schema\": \"pmg_candidate_windows_v1\",\n"
           << "  \"source_bvh_path\": " << JsonString(source_bvh_path) << ",\n"
           << "  \"extraction_config\": {\n"
           << "    \"min_frames\": " << config.min_length_frames << ",\n"
           << "    \"max_frames\": " << config.max_length_frames << ",\n"
           << "    \"stride_frames\": " << config.stride_frames << ",\n"
           << "    \"top_k\": " << config.top_k << ",\n"
           << "    \"prefer_cyclic\": " << (config.prefer_cyclic ? "true" : "false") << ",\n"
           << "    \"endpoint_window_frames\": " << config.endpoint_window_frames << ",\n"
           << "    \"max_normalized_pose_distance\": " << config.max_normalized_pose_distance << ",\n"
           << "    \"max_heading_delta_radians\": " << config.max_heading_delta_radians << ",\n"
           << "    \"heading_score_weight\": " << config.heading_score_weight << ",\n"
           << "    \"min_window_speed\": " << config.min_window_speed << "\n  },\n"
           << "  \"candidates\": [";
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        const auto& candidate = candidates[index];
        output << (index == 0 ? "\n" : ",\n")
               << "    {\"source_bvh_path\": " << JsonString(source_bvh_path)
               << ", \"start_frame\": " << candidate.start_frame
               << ", \"end_frame\": " << candidate.end_frame
               << ", \"score\": " << candidate.score
               << ", \"reason\": " << JsonString(candidate.reason)
               << ", \"root_displacement\": " << candidate.root_displacement
               << ", \"heading_delta_radians\": " << candidate.heading_delta << "}";
    }
    output << (candidates.empty() ? "]\n}\n" : "\n  ]\n}\n");
    if (!output) throw std::runtime_error("failed to write candidate JSON output");
}

}  // namespace pmg
