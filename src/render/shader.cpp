#include "render/shader.hpp"

#include <cstdio>
#include <utility>
#include <vector>

namespace {

GLuint compileStage(GLenum type, const char* source, const char* debugName) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == 0) {
        GLint logLength = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> log(logLength > 1 ? static_cast<size_t>(logLength) : 1, '\0');
        glGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr, log.data());
        const char* stageName = (type == GL_VERTEX_SHADER) ? "vertex" : "fragment";
        std::fprintf(stderr, "[shader] %s (%s) compile failed:\n%s\n", debugName, stageName, log.data());
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

} // namespace

Shader::~Shader() { destroy(); }

Shader::Shader(Shader&& other) noexcept
    : program_(other.program_), locations_(std::move(other.locations_)) {
    other.program_ = 0;
}

Shader& Shader::operator=(Shader&& other) noexcept {
    if (this != &other) {
        destroy();
        program_ = other.program_;
        locations_ = std::move(other.locations_);
        other.program_ = 0;
    }
    return *this;
}

void Shader::destroy() {
    if (program_ != 0) {
        glDeleteProgram(program_);
        program_ = 0;
    }
    locations_.clear();
}

bool Shader::compile(const char* vertexSource, const char* fragmentSource, const char* debugName) {
    destroy();

    GLuint vertexStage = compileStage(GL_VERTEX_SHADER, vertexSource, debugName);
    if (vertexStage == 0) return false;
    GLuint fragmentStage = compileStage(GL_FRAGMENT_SHADER, fragmentSource, debugName);
    if (fragmentStage == 0) {
        glDeleteShader(vertexStage);
        return false;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vertexStage);
    glAttachShader(program, fragmentStage);
    glLinkProgram(program);
    glDeleteShader(vertexStage);
    glDeleteShader(fragmentStage);

    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok == 0) {
        GLint logLength = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> log(logLength > 1 ? static_cast<size_t>(logLength) : 1, '\0');
        glGetProgramInfoLog(program, static_cast<GLsizei>(log.size()), nullptr, log.data());
        std::fprintf(stderr, "[shader] %s link failed:\n%s\n", debugName, log.data());
        glDeleteProgram(program);
        return false;
    }

    program_ = program;
    return true;
}

void Shader::use() const { glUseProgram(program_); }

GLint Shader::location(const char* name) {
    auto it = locations_.find(name);
    if (it != locations_.end()) return it->second;
    GLint loc = glGetUniformLocation(program_, name);
    locations_.emplace(name, loc);
    return loc;
}

void Shader::setFloat(const char* name, float value) { glUniform1f(location(name), value); }
void Shader::setInt(const char* name, int value) { glUniform1i(location(name), value); }

void Shader::setVec3(const char* name, const math::Vec3& value) {
    glUniform3fv(location(name), 1, &value.x);
}

void Shader::setMat4(const char* name, const math::Mat4& value) {
    glUniformMatrix4fv(location(name), 1, GL_FALSE, value.m);
}
