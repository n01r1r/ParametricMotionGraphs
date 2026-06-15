#include "ViewerHost.h"

#include <imgui.h>

#include <stdexcept>
#include <utility>

namespace pmgviewer {

ViewerHost::ViewerHost(std::unique_ptr<ViewerWorkspace> workspace)
    : workspace_(std::move(workspace)) {
    if (workspace_ == nullptr) {
        throw std::invalid_argument("ViewerHost requires a ViewerWorkspace");
    }
}

ViewerHost::~ViewerHost() {
    Shutdown();
}

void ViewerHost::Initialize(const std::string& bootstrap_path) {
    workspace_->Initialize(bootstrap_path);
    renderer_.Initialize();
    renderer_initialized_ = true;
}

void ViewerHost::Shutdown() {
    if (!renderer_initialized_) {
        return;
    }
    renderer_.Shutdown();
    renderer_initialized_ = false;
}

void ViewerHost::Update(float delta_seconds) {
    workspace_->Update(delta_seconds);
    if (follow_scene_focus_) {
        camera_.SetTarget(workspace_->Scene().focus_point);
    }
}

void ViewerHost::BuildUi() {
    workspace_->BuildUi();
    BuildViewUi();
}

void ViewerHost::BuildViewUi() {
    ImGui::SetNextWindowPos(ImVec2(544.0f, 12.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280.0f, 150.0f), ImGuiCond_FirstUseEver);
    ImGui::Begin("Viewer");
    ImGui::Checkbox("Follow scene focus", &follow_scene_focus_);
    if (ImGui::Button("Reset camera")) {
        camera_.Reset();
        follow_scene_focus_ = true;
    }
    ImGui::TextDisabled("WASD / drag orbit - scroll zoom");
    ImGui::End();
}

void ViewerHost::Render(int framebuffer_width, int framebuffer_height) {
    if (!renderer_initialized_) {
        throw std::logic_error("ViewerHost::Initialize must be called before Render");
    }
    if (framebuffer_height > 0) {
        camera_.SetAspectRatio(
            static_cast<float>(framebuffer_width) /
            static_cast<float>(framebuffer_height));
    }
    renderer_.Render(
        workspace_->Scene(),
        camera_.ViewMatrix(),
        camera_.ProjectionMatrix(),
        camera_.Position(),
        framebuffer_width,
        framebuffer_height);
}

bool ViewerHost::HandleGroundClick(
    const glm::vec3& ray_origin, const glm::vec3& ray_direction) {
    return workspace_->HandleGroundClick(ray_origin, ray_direction);
}

}  // namespace pmgviewer
