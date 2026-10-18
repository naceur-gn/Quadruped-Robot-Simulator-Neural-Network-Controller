#pragma once

// GLSL 3.30 shader sources, embedded so the executable runs from anywhere.

inline constexpr const char* kObjectVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 uModel;
uniform mat4 uViewProj;

out vec3 vWorldPos;
out vec3 vNormal;

void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorldPos = world.xyz;
    vNormal = mat3(uModel) * aNormal;
    gl_Position = uViewProj * world;
}
)";

inline constexpr const char* kObjectFragmentShader = R"(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;

uniform vec3 uColor;
uniform vec3 uLightDir;
uniform vec3 uCamPos;

out vec4 FragColor;

void main() {
    vec3 n = normalize(vNormal);
    vec3 l = normalize(-uLightDir);
    float ndl = max(dot(n, l), 0.0);

    vec3 v = normalize(uCamPos - vWorldPos);
    vec3 h = normalize(l + v);

    // Improved specular with GGX-like falloff
    float spec = pow(max(dot(n, h), 0.0), 48.0) * 0.30;

    // Rim lighting: subtle backlight silhouette glow
    float rim = 1.0 - max(dot(n, v), 0.0);
    rim = pow(rim, 3.0) * 0.18;

    // Hemisphere ambient: slightly brighter on surfaces facing up
    float hemiAmbient = 0.36 + 0.06 * max(n.y, 0.0);

    vec3 color = uColor * (hemiAmbient + 0.68 * ndl) + vec3(spec) + vec3(rim) * uColor;

    float dist = length(vWorldPos - uCamPos);
    float fog = smoothstep(25.0, 60.0, dist);
    FragColor = vec4(mix(color, vec3(0.58, 0.60, 0.63), fog), 1.0);
}
)";

inline constexpr const char* kGroundFragmentShader = R"(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;

uniform vec3 uLightDir;
uniform vec3 uCamPos;
uniform float uTileSize;
uniform vec3 uColorWhite;
uniform vec3 uColorViolet;

out vec4 FragColor;

float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

void main() {
    vec2 tile = vWorldPos.xz / uTileSize;
    vec2 cell = floor(tile);

    // Checker pattern for clean technical platform
    bool alternate = mod(cell.x + cell.y, 2.0) < 1.0;
    vec3 base = alternate ? uColorWhite : uColorViolet;

    // Subtle per-square shade variation
    base *= 0.97 + 0.05 * hash(cell);

    // Crisp seam lines along square grid borders
    vec2 f = fract(tile);
    vec2 edge = min(f, 1.0 - f);
    float line = smoothstep(0.0, 0.04, min(edge.x, edge.y));
    base *= mix(0.75, 1.0, line);

    // Subtle inner-tile beveled highlight near edges
    float bevel = smoothstep(0.0, 0.08, min(edge.x, edge.y));
    base += vec3(0.015) * (1.0 - bevel);

    vec3 n = normalize(vNormal);
    vec3 l = normalize(-uLightDir);
    float ndl = max(dot(n, l), 0.0);

    vec3 v = normalize(uCamPos - vWorldPos);
    vec3 h = normalize(l + v);

    // Wider specular for polished floor look
    float spec = pow(max(dot(n, h), 0.0), 32.0) * 0.12;

    // Subtle Fresnel floor reflection (brighter at grazing angles)
    float fresnel = pow(1.0 - max(dot(n, v), 0.0), 3.0) * 0.08;

    vec3 color = base * (0.42 + 0.65 * ndl) + vec3(spec) + vec3(fresnel);

    // Distance-based subtle darkening (arena boundary fade)
    float distFromCenter = length(vWorldPos.xz);
    float arenaFade = smoothstep(8.0, 10.5, distFromCenter) * 0.15;
    color *= 1.0 - arenaFade;

    float dist = length(vWorldPos - uCamPos);
    float fog = smoothstep(25.0, 60.0, dist);
    FragColor = vec4(mix(color, vec3(0.58, 0.60, 0.63), fog), 1.0);
}
)";
