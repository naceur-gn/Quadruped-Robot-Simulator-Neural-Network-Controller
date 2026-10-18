#pragma once
#include "robot/quadruped.hpp"
#include "math.hpp"
#include "physics/physics.hpp"
#include "render/camera.hpp"
#include "render/mesh.hpp"
#include "render/shader.hpp"

#include <vector>

struct SceneConfig {
    float groundSize = 20.0f; // Limited 20m x 20m square arena
    float tileSize = 1.0f;
    math::Vec3 whiteSquare{0.88f, 0.89f, 0.91f};  // Light tech tile
    math::Vec3 violetSquare{0.64f, 0.66f, 0.70f}; // Slate gray tile
    math::Vec3 sunDirection{-0.42f, -1.0f, -0.30f};
};

class Scene {
public:
    bool init(const SceneConfig& config = SceneConfig{});
    void update(float dt);
    void render(const Camera& camera, float aspect);

    // Cast a ray from the camera through (mouseNdcX, mouseNdcY) in [-1,1].
    // If it hits the robot, apply a push force and return true.
    bool tryPushRobot(const Camera& camera, float aspect,
                      float mouseNdcX, float mouseNdcY,
                      float pushStrength = 3.0f);

    const physics::World& physicsWorld() const { return physicsWorld_; }
    robot::Quadruped& robot() { return robot_; }
private:
    struct ObjectView {
        physics::RigidBody* body = nullptr;
        math::Vec3 color;
    };

    SceneConfig config_;
    physics::World physicsWorld_;
    robot::Quadruped robot_;
    Shader objectShader_;
    Shader groundShader_;
    Mesh groundMesh_;
    Mesh boxMesh_;
    Mesh sphereMesh_;
    Mesh diskMesh_;

    std::vector<ObjectView> objects_;
    math::Vec3 lightDirection_{0.0f, -1.0f, 0.0f};
    float animTime_ = 0.0f; // Accumulated animation time for pulsing effects
};
