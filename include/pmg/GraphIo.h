#pragma once

#include "pmg/PmgArtifact.h"

#include <string>

namespace pmg {

// V4 stores the complete offline/online artifact: Skeleton, graph, registration
// metadata, edge build settings/reports, and generated runtime dimensions.
// Readers retain V2/V3 compatibility; those legacy files load with an empty
// Skeleton and cannot use point-cloud runtime alignment without an external one.
void SavePmgArtifactText(const BuiltPmgArtifact& artifact, const std::string& path);
BuiltPmgArtifact LoadPmgArtifactText(const std::string& path);

// Compatibility wrappers for callers that only need graph data. New writes use
// V4 with an empty Skeleton; LoadGraphText accepts V2, V3, and V4.
void SaveGraphText(const ParametricMotionGraph& graph, const std::string& path);
ParametricMotionGraph LoadGraphText(const std::string& path);

}  // namespace pmg
