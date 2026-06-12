#pragma once

#include <glm/glm.hpp>

#include <string>

#include "RenderScene.h"

namespace pmgviewer {

// Compile-time seam between the algorithm-neutral ViewerHost and one
// algorithm-specific Adapter.
//
// Initialize may throw when the bootstrap artifact is invalid. Update and
// BuildUi run once per frame. HandleGroundClick receives a world-space ray and
// returns whether the Adapter consumed it. Scene remains owned by the Adapter
// and is valid until its next mutation.
class ViewerWorkspace {
public:
    virtual ~ViewerWorkspace() = default;

    virtual void Initialize(const std::string& bootstrap_path) = 0;
    virtual void Update(float delta_seconds) = 0;
    virtual void BuildUi() = 0;
    virtual bool HandleGroundClick(
        const glm::vec3& ray_origin, const glm::vec3& ray_direction) = 0;
    virtual const RenderScene& Scene() const = 0;
};

}  // namespace pmgviewer
