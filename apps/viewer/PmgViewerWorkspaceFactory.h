#pragma once

#include <memory>

#include "ViewerWorkspace.h"

namespace pmgviewer {

// Creates the built-in PMG Adapter without exposing pmg_core types to the
// ViewerHost or the executable composition root.
std::unique_ptr<ViewerWorkspace> CreatePmgViewerWorkspace();

}  // namespace pmgviewer
