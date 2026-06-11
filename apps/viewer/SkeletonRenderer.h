#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>

#include <vector>

#include "MeshPrimitives.h"

namespace pmgviewer {

// A single bone drawn as a capsule/cylinder between two world-space joints.
struct BoneSegment {
    glm::vec3 parent_position;
    glm::vec3 child_position;
};

// Everything the renderer needs for one frame. World space, Y-up, units match
// the BVH source (feet already offset onto y = 0 by the caller).
struct RenderScene {
    std::vector<BoneSegment> bones;
    std::vector<glm::vec3> joints;
    // Overlay markers (e.g. the goto target gizmo): points render as spheres,
    // lines as thin cylinders, both in the marker color.
    std::vector<glm::vec3> marker_points;
    std::vector<BoneSegment> marker_lines;
    glm::vec3 light_to_direction{0.4f, 0.85f, 0.35f};  // normalized direction toward the light
    glm::vec3 focus_point{0.0f, 8.0f, 0.0f};           // shadow framing center
};

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
