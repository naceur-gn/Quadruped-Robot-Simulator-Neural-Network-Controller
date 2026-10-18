#include "gl.hpp"

#include <GLFW/glfw3.h>

#include <cstdio>

#define GL_DEFINE(ret, name, args) PFN_##name p_##name = nullptr;
GL_FUNC_LIST(GL_DEFINE)
#undef GL_DEFINE

bool gl_loadFunctions() {
    struct Entry {
        const char* name;
        void** pointer;
    };

#define GL_ENTRY(ret, name, args) {#name, reinterpret_cast<void**>(&p_##name)},
    static const Entry entries[] = {
        GL_FUNC_LIST(GL_ENTRY)
    };
#undef GL_ENTRY

    for (const Entry& entry : entries) {
        *entry.pointer = reinterpret_cast<void*>(glfwGetProcAddress(entry.name));
        if (*entry.pointer == nullptr) {
            std::fprintf(stderr, "[gl] failed to load entry point: %s\n", entry.name);
            return false;
        }
    }
    return true;
}
