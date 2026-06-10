#include "MeshPrimitives.h"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace pmgviewer {

namespace {

constexpr float kTwoPi = 6.28318530718f;

struct MeshData {
    std::vector<float> vertices;        // interleaved pos(3) + normal(3)
    std::vector<unsigned int> indices;

    unsigned int PushVertex(float px, float py, float pz, float nx, float ny, float nz) {
        const unsigned int index = static_cast<unsigned int>(vertices.size() / 6);
        vertices.insert(vertices.end(), {px, py, pz, nx, ny, nz});
        return index;
    }
};

GpuMesh Upload(const MeshData& mesh) {
    GpuMesh gpu;
    gpu.index_count = static_cast<GLsizei>(mesh.indices.size());

    glGenVertexArrays(1, &gpu.vertex_array);
    glGenBuffers(1, &gpu.vertex_buffer);
    glGenBuffers(1, &gpu.index_buffer);

    glBindVertexArray(gpu.vertex_array);

    glBindBuffer(GL_ARRAY_BUFFER, gpu.vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh.vertices.size() * sizeof(float)),
                 mesh.vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.index_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(unsigned int)),
                 mesh.indices.data(), GL_STATIC_DRAW);

    const GLsizei stride = 6 * sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                          reinterpret_cast<void*>(3 * sizeof(float)));

    glBindVertexArray(0);
    return gpu;
}

}  // namespace

void GpuMesh::Draw() const {
    glBindVertexArray(vertex_array);
    glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void GpuMesh::Destroy() {
    if (index_buffer != 0) glDeleteBuffers(1, &index_buffer);
    if (vertex_buffer != 0) glDeleteBuffers(1, &vertex_buffer);
    if (vertex_array != 0) glDeleteVertexArrays(1, &vertex_array);
    *this = GpuMesh{};
}

GpuMesh CreateCylinderMesh(int radial_segments) {
    if (radial_segments < 3) {
        throw std::runtime_error("CreateCylinderMesh: radial_segments must be >= 3");
    }

    MeshData mesh;

    // Side wall: a wrapped ring of quads with radial normals.
    for (int segment = 0; segment <= radial_segments; ++segment) {
        const float angle = kTwoPi * static_cast<float>(segment) /
                            static_cast<float>(radial_segments);
        const float cos_a = std::cos(angle);
        const float sin_a = std::sin(angle);
        mesh.PushVertex(cos_a, 0.0f, sin_a, cos_a, 0.0f, sin_a);
        mesh.PushVertex(cos_a, 1.0f, sin_a, cos_a, 0.0f, sin_a);
    }
    for (int segment = 0; segment < radial_segments; ++segment) {
        const unsigned int base = static_cast<unsigned int>(segment * 2);
        mesh.indices.insert(mesh.indices.end(),
                            {base, base + 1, base + 2, base + 1, base + 3, base + 2});
    }

    // Bottom cap (normal -Y) and top cap (normal +Y) as triangle fans.
    const unsigned int bottom_center = mesh.PushVertex(0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f);
    const unsigned int bottom_ring_start = static_cast<unsigned int>(mesh.vertices.size() / 6);
    for (int segment = 0; segment <= radial_segments; ++segment) {
        const float angle = kTwoPi * static_cast<float>(segment) /
                            static_cast<float>(radial_segments);
        mesh.PushVertex(std::cos(angle), 0.0f, std::sin(angle), 0.0f, -1.0f, 0.0f);
    }
    for (int segment = 0; segment < radial_segments; ++segment) {
        mesh.indices.insert(mesh.indices.end(),
                            {bottom_center, bottom_ring_start + segment + 1,
                             bottom_ring_start + segment});
    }

    const unsigned int top_center = mesh.PushVertex(0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f);
    const unsigned int top_ring_start = static_cast<unsigned int>(mesh.vertices.size() / 6);
    for (int segment = 0; segment <= radial_segments; ++segment) {
        const float angle = kTwoPi * static_cast<float>(segment) /
                            static_cast<float>(radial_segments);
        mesh.PushVertex(std::cos(angle), 1.0f, std::sin(angle), 0.0f, 1.0f, 0.0f);
    }
    for (int segment = 0; segment < radial_segments; ++segment) {
        mesh.indices.insert(mesh.indices.end(),
                            {top_center, top_ring_start + segment,
                             top_ring_start + segment + 1});
    }

    return Upload(mesh);
}

GpuMesh CreateSphereMesh(int stack_count, int sector_count) {
    if (stack_count < 2 || sector_count < 3) {
        throw std::runtime_error("CreateSphereMesh: need stack_count>=2, sector_count>=3");
    }

    MeshData mesh;
    for (int stack = 0; stack <= stack_count; ++stack) {
        const float stack_angle = kTwoPi * 0.5f *
            (0.5f - static_cast<float>(stack) / static_cast<float>(stack_count));
        const float xy = std::cos(stack_angle);
        const float y = std::sin(stack_angle);
        for (int sector = 0; sector <= sector_count; ++sector) {
            const float sector_angle = kTwoPi * static_cast<float>(sector) /
                                       static_cast<float>(sector_count);
            const float x = xy * std::cos(sector_angle);
            const float z = xy * std::sin(sector_angle);
            mesh.PushVertex(x, y, z, x, y, z);  // unit sphere: position == normal
        }
    }

    const int stride = sector_count + 1;
    for (int stack = 0; stack < stack_count; ++stack) {
        for (int sector = 0; sector < sector_count; ++sector) {
            const unsigned int current = static_cast<unsigned int>(stack * stride + sector);
            const unsigned int next = static_cast<unsigned int>(current + stride);
            mesh.indices.insert(mesh.indices.end(),
                                {current, next, current + 1,
                                 current + 1, next, next + 1});
        }
    }

    return Upload(mesh);
}

GpuMesh CreateFloorMesh(float half_extent) {
    if (half_extent <= 0.0f) {
        throw std::runtime_error("CreateFloorMesh: half_extent must be positive");
    }

    MeshData mesh;
    mesh.PushVertex(-half_extent, 0.0f, -half_extent, 0.0f, 1.0f, 0.0f);
    mesh.PushVertex(half_extent, 0.0f, -half_extent, 0.0f, 1.0f, 0.0f);
    mesh.PushVertex(half_extent, 0.0f, half_extent, 0.0f, 1.0f, 0.0f);
    mesh.PushVertex(-half_extent, 0.0f, half_extent, 0.0f, 1.0f, 0.0f);
    mesh.indices = {0, 1, 2, 0, 2, 3};
    return Upload(mesh);
}

}  // namespace pmgviewer
