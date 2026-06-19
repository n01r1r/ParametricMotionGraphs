#pragma once

#include "pmg/GraphSpec.h"
#include "pmg/PmgArtifact.h"

namespace pmg {

// Paper offline construction sequence. Input is a parsed, structurally validated
// GraphSpec; output is the runtime artifact before GraphIo persistence.
BuiltPmgArtifact BuildPmgOfflinePipeline(
    const GraphSpec& spec,
    const ArtifactBuildConfig& config = {});

}  // namespace pmg
