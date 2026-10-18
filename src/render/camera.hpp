#pragma once

#include "math.hpp"

class Camera {
public:
    math::Vec3 position{8.0f, 6.0f, 11.0f};
    float yaw = 0.0f;    // radians, rotation about +Y
    float pitch = 0.0f;  // radians, positive = looking up
    float fovDegrees = 60.0f;

    void lookAt(const math::Vec3& target);
    void rotate(float dxPixels, float dyPixels);
    void move(const math::Vec3& localDirection, float distance);

    math::Vec3 forward() const;
    math::Vec3 right() const;
    math::Vec3 up() const;

    math::Mat4 view() const;
    math::Mat4 projection(float aspect) const;
};
