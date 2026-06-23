#pragma once

#include <memory>
#include <string>

#include "Camera.h"
#include "SkeletonRenderer.h"
#include "ViewerWorkspace.h"

namespace pmgviewer {

// Owns viewer-level camera, rendering, and input dispatch. Algorithm state and
// diagnostics stay behind the ViewerWorkspace Interface.
class ViewerHost {
public:
    explicit ViewerHost(std::unique_ptr<ViewerWorkspace> workspace);
    ~ViewerHost();

    ViewerHost(const ViewerHost&) = delete;
    ViewerHost& operator=(const ViewerHost&) = delete;

    void Initialize(const std::string& bootstrap_path);
    void Shutdown();
    void Update(float delta_seconds);
    void BuildUi();
    void Render(int framebuffer_width, int framebuffer_height);

    bool HandleGroundClick(
        const glm::vec3& ray_origin, const glm::vec3& ray_direction);

    OrbitCamera& Camera() { return camera_; }

    // WASD panning takes manual control of the target, so it turns auto-follow
    // off; "Reset camera" / the checkbox turn it back on.
    void SetFollowSceneFocus(bool follow) { follow_scene_focus_ = follow; }
    void SetDirectSteeringInput(float turn, float speed) {
        workspace_->SetDirectSteeringInput(turn, speed);
    }

private:
    // ponytail: temporary Dear ImGui leakage for camera-only host controls.
    // Migrate into a small host state/command adapter if this window grows.
    void BuildViewUi();

    std::unique_ptr<ViewerWorkspace> workspace_;
    OrbitCamera camera_;
    SkeletonRenderer renderer_;
    bool follow_scene_focus_ = true;
    bool renderer_initialized_ = false;
};

}  // namespace pmgviewer
