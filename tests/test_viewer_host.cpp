#include "ViewerHost.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

namespace {

constexpr float kTolerance = 1.0e-6f;

bool NearlyEqual(float left, float right) {
    return std::abs(left - right) <= kTolerance;
}

class TestWorkspace final : public pmgviewer::ViewerWorkspace {
public:
    void Initialize(const std::string& bootstrap_path) override {
        (void)bootstrap_path;
    }

    void Update(float delta_seconds) override {
        update_seconds = delta_seconds;
        scene.focus_point = {3.0f, 4.0f, 5.0f};
    }

    void BuildUi() override {}

    bool HandleGroundClick(
        const glm::vec3& ray_origin, const glm::vec3& ray_direction) override {
        click_origin = ray_origin;
        click_direction = ray_direction;
        return true;
    }

    const pmgviewer::RenderScene& Scene() const override {
        return scene;
    }

    float update_seconds = 0.0f;
    glm::vec3 click_origin{0.0f};
    glm::vec3 click_direction{0.0f};
    pmgviewer::RenderScene scene;
};

}  // namespace

int main() {
    auto workspace = std::make_unique<TestWorkspace>();
    TestWorkspace* workspace_observer = workspace.get();
    pmgviewer::ViewerHost host(std::move(workspace));

    host.Update(0.25f);
    assert(NearlyEqual(workspace_observer->update_seconds, 0.25f));
    assert(NearlyEqual(host.Camera().Target().x, 3.0f));
    assert(NearlyEqual(host.Camera().Target().y, 4.0f));
    assert(NearlyEqual(host.Camera().Target().z, 5.0f));

    const glm::vec3 origin(1.0f, 2.0f, 3.0f);
    const glm::vec3 direction(0.0f, -1.0f, 0.0f);
    assert(host.HandleGroundClick(origin, direction));
    assert(workspace_observer->click_origin == origin);
    assert(workspace_observer->click_direction == direction);

    std::cout << "test_viewer_host passed\n";
    return 0;
}
