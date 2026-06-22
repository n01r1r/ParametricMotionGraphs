#pragma once

#include <functional>
#include <vector>

#include "ViewerUiState.h"

namespace pmgviewer {

// Dear ImGui widget adapter only. Platform/backend lifecycle stays in main.cpp.
class ImGuiViewerUi {
public:
    std::vector<ViewerUiCommand> Build(
        const ViewerUiState& state,
        const std::function<void()>& build_legacy_tabs);

private:
    char clip_filter_[64] = "";
};

}  // namespace pmgviewer
