#pragma once

#include "pmg/ParametricMotionGraph.h"

#include <string>

namespace pmg {

// Text serialization for built PMG graphs. It stores node names, motion-space
// examples, generated clip poses, registration time warps, and edge samples;
// it does not store Skeletons because runtime graph playback only needs
// generated MotionClip data. Reads both V3 (current) and V2 (pre-warp) files.
void SaveGraphText(const ParametricMotionGraph& graph, const std::string& path);
ParametricMotionGraph LoadGraphText(const std::string& path);

}  // namespace pmg
