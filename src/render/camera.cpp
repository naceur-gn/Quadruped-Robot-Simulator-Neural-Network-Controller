#include "render/camera.hpp"

namespace {
constexpr math::Vec3 kWorldUp{0.0f, 1.0f, 0.0f};
constexpr float kMouseSensitivity = 0.0025f;
constexpr float kMaxPitch = 1.5533f; // ~89 degrees
} // namespace

void Camera::lookAt(const math::Vec3& target) {
    math::Vec3 dir = (target - position).normalized();
    pitch = std::asin(math::clampf(dir.y, -1.0f, 1.0f));
    yaw = std::atan2(dir.z, dir.x);
}

void Camera::rotate(float dxPixels, float dyPixels) {
    yaw += dxPixels * kMouseSensitivity;
    pitch -= dyPixels * kMouseSensitivity;
    pitch = math::clampf(pitch, -kMaxPitch, kMaxPitch);
}

void Camera::move(const math::Vec3& localDirection, float distance) {
    position += right() * (localDirection.x * distance);
    position += kWorldUp * (localDirection.y * distance);
    position += forward() * (localDirection.z * distance);

    // Keep the camera above the ground plane.
    if (position.y < 0.2f) position.y = 0.2f;
}

math::Vec3 Camera::forward() const {
    float cp = std::cos(pitch);
    return {cp * std::cos(yaw), std::sin(pitch), cp * std::sin(yaw)};
}

math::Vec3 Camera::right() const { return math::cross(forward(), kWorldUp).normalized(); }

math::Vec3 Camera::up() const { return math::cross(right(), forward()); }

math::Mat4 Camera::view() const {
    return math::Mat4::lookAt(position, position + forward(), kWorldUp);
}

math::Mat4 Camera::projection(float aspect) const {
    return math::Mat4::perspective(math::radians(fovDegrees), aspect, 0.05f, 250.0f);
}
