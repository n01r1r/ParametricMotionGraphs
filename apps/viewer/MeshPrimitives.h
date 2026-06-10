#pragma once

#include <GL/glew.h>

namespace pmgviewer {

// A GPU mesh of interleaved position+normal vertices (6 floats per vertex)
// with an index buffer. Owns its GL objects; call Destroy() before context
// teardown.
struct GpuMesh {
    GLuint vertex_array = 0;
    GLuint vertex_buffer = 0;
    GLuint index_buffer = 0;
    GLsizei index_count = 0;

    void Draw() const;
    void Destroy();
};

// Unit cylinder spanning y in [0, 1] with radius 1, capped at both ends.
// `radial_segments` controls smoothness (>= 3).
GpuMesh CreateCylinderMesh(int radial_segments);

// Unit UV sphere of radius 1 centered at the origin.
GpuMesh CreateSphereMesh(int stack_count, int sector_count);

// Square floor on the y = 0 plane, spanning [-half_extent, half_extent] in x/z,
// with an upward normal.
GpuMesh CreateFloorMesh(float half_extent);

}  // namespace pmgviewer
