#pragma once

#include "gl.hpp"
#include "math.hpp"

#include <string>
#include <unordered_map>

class Shader {
public:
    Shader() = default;
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    bool compile(const char* vertexSource, const char* fragmentSource, const char* debugName);
    void use() const;

    void setFloat(const char* name, float value);
    void setInt(const char* name, int value);
    void setVec3(const char* name, const math::Vec3& value);
    void setMat4(const char* name, const math::Mat4& value);

    bool valid() const { return program_ != 0; }

private:
    GLint location(const char* name);
    void destroy();

    GLuint program_ = 0;
    std::unordered_map<std::string, GLint> locations_;
};
