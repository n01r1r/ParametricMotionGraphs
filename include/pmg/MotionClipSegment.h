#pragma once

#include <string>

namespace pmg {

struct MotionClipSegment {
    std::string source_bvh;
    int start_frame = 0;
    int end_frame = -1;  // -1 = full clip through last frame
    std::string phase_label;
    std::string contact_start;
    std::string contact_end;
};

inline MotionClipSegment FullClipSegment(std::string source_bvh) {
    MotionClipSegment segment;
    segment.source_bvh = std::move(source_bvh);
    return segment;
}

}  // namespace pmg
