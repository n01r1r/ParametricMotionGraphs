#include "pmg/BvhLoader.h"
#include "pmg/CandidateWindowExtractor.h"

#include <cassert>
#include <sstream>

int main() {
    const pmg::BvhData data = pmg::BvhLoader::Load(PMG_SOURCE_DIR "/BVH/WalkLoopA.bvh");
    pmg::CandidateWindowExtractionConfig config;
    config.min_length_frames = 10;
    config.max_length_frames = 80;
    config.stride_frames = 5;
    config.top_k = 4;
    config.prefer_cyclic = false;

    const auto first = pmg::ExtractCandidateMotionWindows(data.skeleton, data.clip, config);
    const auto second = pmg::ExtractCandidateMotionWindows(data.skeleton, data.clip, config);
    assert(!first.empty());
    assert(first.size() <= static_cast<std::size_t>(config.top_k));
    assert(first.size() == second.size());
    for (std::size_t index = 0; index < first.size(); ++index) {
        assert(first[index].start_frame >= 0);
        assert(first[index].end_frame < data.clip.NumFrames());
        assert(first[index].end_frame > first[index].start_frame);
        assert(first[index].score == second[index].score);
        if (index > 0) assert(first[index - 1].score <= first[index].score);
    }

    pmg::MotionClip short_clip = data.clip;
    short_clip.frames.resize(3);
    assert(pmg::ExtractCandidateMotionWindows(data.skeleton, short_clip, config).empty());

    // Opt-in locomotion filter: every surviving window must clear the speed floor,
    // and an impossibly high floor must drop everything (faithful, not just looser).
    pmg::CandidateWindowExtractionConfig travel = config;
    travel.min_window_speed = 1.0f;  // BVH native units/sec
    const auto filtered = pmg::ExtractCandidateMotionWindows(data.skeleton, data.clip, travel);
    for (const auto& window : filtered) {
        const float duration =
            static_cast<float>(window.end_frame - window.start_frame + 1) / data.clip.frames_per_second;
        assert(duration > 0.0f && window.root_displacement / duration >= travel.min_window_speed);
    }
    travel.min_window_speed = 1.0e9f;
    assert(pmg::ExtractCandidateMotionWindows(data.skeleton, data.clip, travel).empty());

    std::ostringstream exported;
    pmg::WriteCandidateWindowsJson(exported, "clips/Walk\\\"A.bvh", config, first);
    const std::string json = exported.str();
    assert(json.find("pmg_candidate_windows_v1") != std::string::npos);
    assert(json.find("clips/Walk\\\\\\\"A.bvh") != std::string::npos);
    assert(json.find("\"start_frame\": " + std::to_string(first.front().start_frame)) != std::string::npos);
    assert(json.find("\"end_frame\": " + std::to_string(first.front().end_frame)) != std::string::npos);
    assert(json.find("\"score\":") != std::string::npos);
    assert(json.find("\"reason\":") != std::string::npos);

    std::ostringstream empty_export;
    pmg::WriteCandidateWindowsJson(empty_export, "short.bvh", config, {});
    assert(empty_export.str().find("\"candidates\": []") != std::string::npos);
    return 0;
}
