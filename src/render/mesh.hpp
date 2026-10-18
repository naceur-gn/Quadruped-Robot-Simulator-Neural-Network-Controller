#pragma once

#include "gl.hpp"
#include "math.hpp"

#include <cstdint>
#include <vector>

struct Vertex {
    math::Vec3 position;
    math::Vec3 normal;
};

class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    void upload(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices);
    void draw() const;

    bool valid() const { return vao_ != 0; }

private:
    void destroy();

    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLuint ebo_ = 0;
    GLsizei indexCount_ = 0;
};

namespace meshes {

// 1x1 quad in the XZ plane, normal +Y, centered at the origin.
Mesh makeGroundPlane();

// 1x1x1 cube centered at the origin, flat per-face normals.
Mesh makeUnitBox();

// Unit-radius sphere centered at the origin.
Mesh makeUnitSphere(int rings = 24, int segments = 32);

// Flat unit disk in the XZ plane, normal +Y, centered at the origin.
Mesh makeFlatDisk(int segments = 32);

} // namespace meshes
