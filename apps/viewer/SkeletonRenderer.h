#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>

#include "MeshPrimitives.h"
#include "RenderScene.h"

namespace pmgviewer {

// Forward-lit renderer with a single directional light and a shadow map.
// Draws a grid floor that receives shadows and skeleton bones/joints that cast
// and receive them.
class SkeletonRenderer {
public:
    void Initialize();
    void Shutdown();

    void Render(const RenderScene& scene,
                const glm::mat4& view_matrix,
                const glm::mat4& projection_matrix,
                const glm::vec3& camera_position,
                int framebuffer_width,
                int framebuffer_height);

private:
    void RenderShadowPass(const RenderScene& scene, const glm::mat4& light_space_matrix);
    void RenderScenePass(const RenderScene& scene,
                         const glm::mat4& view_matrix,
                         const glm::mat4& projection_matrix,
                         const glm::vec3& camera_position,
                         const glm::mat4& light_space_matrix);
    void DrawSceneGeometry(const RenderScene& scene, GLuint program, bool is_depth_pass);

    GLuint scene_program_ = 0;
    GLuint depth_program_ = 0;

    GLuint shadow_framebuffer_ = 0;
    GLuint shadow_depth_texture_ = 0;

    GpuMesh cylinder_mesh_;
    GpuMesh sphere_mesh_;
    GpuMesh floor_mesh_;
};

}  // namespace pmgviewer
