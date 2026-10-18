#include "render/mesh.hpp"

#include <utility>

Mesh::~Mesh() { destroy(); }

Mesh::Mesh(Mesh&& other) noexcept
    : vao_(other.vao_), vbo_(other.vbo_), ebo_(other.ebo_), indexCount_(other.indexCount_) {
    other.vao_ = 0;
    other.vbo_ = 0;
    other.ebo_ = 0;
    other.indexCount_ = 0;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
    if (this != &other) {
        destroy();
        vao_ = other.vao_;
        vbo_ = other.vbo_;
        ebo_ = other.ebo_;
        indexCount_ = other.indexCount_;
        other.vao_ = 0;
        other.vbo_ = 0;
        other.ebo_ = 0;
        other.indexCount_ = 0;
    }
    return *this;
}

void Mesh::destroy() {
    if (ebo_ != 0) glDeleteBuffers(1, &ebo_);
    if (vbo_ != 0) glDeleteBuffers(1, &vbo_);
    if (vao_ != 0) glDeleteVertexArrays(1, &vao_);
    vao_ = vbo_ = ebo_ = 0;
    indexCount_ = 0;
}

void Mesh::upload(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices) {
    destroy();

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                 vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(uint32_t)),
                 indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<const void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<const void*>(offsetof(Vertex, normal)));

    glBindVertexArray(0);
    indexCount_ = static_cast<GLsizei>(indices.size());
}

void Mesh::draw() const {
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

namespace meshes {

Mesh makeGroundPlane() {
    std::vector<Vertex> vertices = {
        {{-0.5f, 0.0f,  0.5f}, {0.0f, 1.0f, 0.0f}},
        {{ 0.5f, 0.0f,  0.5f}, {0.0f, 1.0f, 0.0f}},
        {{ 0.5f, 0.0f, -0.5f}, {0.0f, 1.0f, 0.0f}},
        {{-0.5f, 0.0f, -0.5f}, {0.0f, 1.0f, 0.0f}},
    };
    std::vector<uint32_t> indices = {0, 1, 2, 0, 2, 3};

    Mesh mesh;
    mesh.upload(vertices, indices);
    return mesh;
}

Mesh makeUnitBox() {
    struct Face {
        math::Vec3 normal, u, v;
    };
    const Face faces[6] = {
        {{ 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}},
        {{-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f,  1.0f}, {0.0f, 1.0f, 0.0f}},
        {{ 0.0f, 1.0f, 0.0f}, {1.0f, 0.0f,  0.0f}, {0.0f, 0.0f, -1.0f}},
        {{ 0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}},
        {{ 0.0f, 0.0f, 1.0f}, {1.0f, 0.0f,  0.0f}, {0.0f, 1.0f, 0.0f}},
        {{ 0.0f, 0.0f, -1.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}},
    };

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(24);
    indices.reserve(36);

    for (const Face& face : faces) {
        uint32_t base = static_cast<uint32_t>(vertices.size());
        math::Vec3 center = face.normal * 0.5f;
        const math::Vec3 corners[4] = {
            center - face.u * 0.5f - face.v * 0.5f,
            center + face.u * 0.5f - face.v * 0.5f,
            center + face.u * 0.5f + face.v * 0.5f,
            center - face.u * 0.5f + face.v * 0.5f,
        };
        for (const math::Vec3& corner : corners) {
            vertices.push_back({corner, face.normal});
        }
        indices.insert(indices.end(),
                       {base + 0, base + 1, base + 2,
                        base + 0, base + 2, base + 3});
    }

    Mesh mesh;
    mesh.upload(vertices, indices);
    return mesh;
}

Mesh makeUnitSphere(int rings, int segments) {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(static_cast<size_t>((rings + 1) * (segments + 1)));

    for (int i = 0; i <= rings; ++i) {
        float theta = math::PI * static_cast<float>(i) / static_cast<float>(rings);
        float y = std::cos(theta);
        float r = std::sin(theta);
        for (int j = 0; j <= segments; ++j) {
            float phi = 2.0f * math::PI * static_cast<float>(j) / static_cast<float>(segments);
            math::Vec3 p{r * std::cos(phi), y, r * std::sin(phi)};
            vertices.push_back({p, p});
        }
    }

    for (int i = 0; i < rings; ++i) {
        for (int j = 0; j < segments; ++j) {
            uint32_t a = static_cast<uint32_t>(i * (segments + 1) + j);
            uint32_t b = a + static_cast<uint32_t>(segments + 1);
            indices.insert(indices.end(), {a, a + 1, b, a + 1, b + 1, b});
        }
    }

    Mesh mesh;
    mesh.upload(vertices, indices);
    return mesh;
}

Mesh makeFlatDisk(int segments) {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    vertices.reserve(static_cast<size_t>(segments + 1));
    indices.reserve(static_cast<size_t>(segments * 3));

    vertices.push_back({{0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}}); // Center

    for (int i = 0; i < segments; ++i) {
        float angle = 2.0f * math::PI * static_cast<float>(i) / static_cast<float>(segments);
        vertices.push_back({{std::cos(angle), 0.0f, std::sin(angle)}, {0.0f, 1.0f, 0.0f}});
    }

    for (int i = 0; i < segments; ++i) {
        uint32_t next = (i + 1) % segments;
        indices.push_back(0);
        indices.push_back(static_cast<uint32_t>(1 + i));
        indices.push_back(static_cast<uint32_t>(1 + next));
    }

    Mesh mesh;
    mesh.upload(vertices, indices);
    return mesh;
}

} // namespace meshes
