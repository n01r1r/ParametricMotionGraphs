#pragma once

#include "pmg/PmgArtifact.h"

#include <string>

namespace pmg {

// V5 stores the complete offline/online artifact: Skeleton, graph, registration
// metadata, edge build settings/reports, runtime sampling rate, and per-node
// parameter calibrations. Readers retain V2/V3/V4 compatibility; V2/V3 load
// with an empty Skeleton and cannot use point-cloud runtime alignment without
// an external one; V4 loads without parameter calibrations.
void SavePmgArtifactText(const BuiltPmgArtifact& artifact, const std::string& path);
BuiltPmgArtifact LoadPmgArtifactText(const std::string& path);

// Compatibility wrappers for callers that only need graph data. New writes use
// V5 with an empty Skeleton; LoadGraphText accepts V2 through V5.
void SaveGraphText(const ParametricMotionGraph& graph, const std::string& path);
ParametricMotionGraph LoadGraphText(const std::string& path);

}  // namespace pmg
