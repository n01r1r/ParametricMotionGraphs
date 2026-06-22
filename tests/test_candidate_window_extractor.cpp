#include "pmg_test.h"

#include "pmg/BvhLoader.h"
#include "pmg/CandidateWindowExtractor.h"

#include <cassert>

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
    return 0;
}
