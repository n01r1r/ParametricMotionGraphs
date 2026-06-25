#pragma once

#include <glm/glm.hpp>

#include <vector>

namespace pmgviewer {

// A single line segment between two world-space points.
struct BoneSegment {
    glm::vec3 parent_position;
    glm::vec3 child_position;
};

struct DiagnosticPoint {
    glm::vec3 position;
    glm::vec3 color;
    float radius = 1.0f;
};

struct DiagnosticLine {
    glm::vec3 start;
    glm::vec3 end;
    glm::vec3 color;
    float radius = 0.25f;
    float alpha = 1.0f;  // <1 = translucent (overlapping ghost trails read as a cloud)
};

// Algorithm-neutral scene description consumed by the viewer renderer.
// Coordinates are world-space, Y-up. The producing ViewerWorkspace owns unit
// conversion; the ViewerHost only renders these values and follows focus_point.
struct RenderScene {
    std::vector<BoneSegment> bones;
    std::vector<glm::vec3> joints;
    std::vector<glm::vec3> marker_points;
    std::vector<BoneSegment> marker_lines;
    std::vector<DiagnosticPoint> diagnostic_points;
    std::vector<DiagnosticLine> diagnostic_lines;
    glm::vec3 light_to_direction{0.4f, 0.85f, 0.35f};
    glm::vec3 focus_point{0.0f, 8.0f, 0.0f};
    glm::vec3 floor_color{0.52f, 0.55f, 0.60f};
};

}  // namespace pmgviewer
