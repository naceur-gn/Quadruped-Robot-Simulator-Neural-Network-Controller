#pragma once

#include "math.hpp"

#include <memory>
#include <vector>

namespace physics {

// A rigid body: point mass + orientation + full inertia tensor.
// Inertia is stored in the body-local principal frame; the world-space
// inverse tensor is rebuilt from the current orientation on demand.
struct RigidBody {
    enum class Shape { Box, Sphere };

    math::Vec3 position;
    math::Quat orientation;
    math::Vec3 linearVelocity;
    math::Vec3 angularVelocity;

    float mass = 1.0f;
    float invMass = 1.0f;
    math::Vec3 inertiaLocal{1.0f, 1.0f, 1.0f};
    math::Vec3 invInertiaLocal{1.0f, 1.0f, 1.0f};

    float restitution = 0.2f;
    float friction = 0.6f;
    float linearDamping = 0.03f;
    float angularDamping = 0.05f;

    Shape shape = Shape::Box;
    math::Vec3 halfExtents{0.5f, 0.5f, 0.5f}; // Box
    float radius = 0.5f;                      // Sphere

    math::Vec3 force{0.0f, 0.0f, 0.0f};
    math::Vec3 torque{0.0f, 0.0f, 0.0f};

    bool isStatic() const { return invMass == 0.0f; }

    math::Mat3 inverseInertiaWorld() const;
    math::Vec3 velocityAtPoint(const math::Vec3& worldPoint) const;

    void applyForce(const math::Vec3& f);
    void applyForceAtPoint(const math::Vec3& f, const math::Vec3& worldPoint);
    void applyTorque(const math::Vec3& t);
    void applyImpulseAtPoint(const math::Vec3& impulse, const math::Vec3& worldPoint);
    void clearForces();
};

// Infinite plane: all points x with dot(normal, x) = offset.
struct Plane {
    math::Vec3 normal{0.0f, 1.0f, 0.0f};
    float offset = 0.0f;
};

// One contact between a dynamic body and a static plane.
struct Contact {
    RigidBody* body = nullptr;
    math::Vec3 point;      // world-space contact point
    math::Vec3 normal;     // unit normal, pointing away from the plane
    float penetration = 0.0f;
    math::Vec3 r;          // point - body->position
    math::Vec3 tangent;    // unit friction direction
    float normalMass = 0.0f;  // inverse effective mass along the normal
    float tangentMass = 0.0f; // inverse effective mass along the tangent
    float targetNormalVelocity = 0.0f;
    float jn = 0.0f; // accumulated normal impulse
    float jt = 0.0f; // accumulated friction impulse
};

class World {
public:
    math::Vec3 gravity{0.0f, -9.81f, 0.0f};
    int solverIterations = 10;
    float restitutionVelocityThreshold = 1.0f; // below this speed, no bouncing
    float penetrationSlop = 0.002f;
    float positionCorrectionRate = 0.5f;

    RigidBody& addBody(RigidBody body);
    RigidBody& addBox(float mass, const math::Vec3& halfExtents,
                      const math::Vec3& position,
                      const math::Quat& orientation = math::Quat{});
    RigidBody& addSphere(float mass, float radius,
                         const math::Vec3& position,
                         const math::Quat& orientation = math::Quat{});

    void addGroundPlane(const Plane& plane);
    const std::vector<Plane>& planes() const { return planes_; }
    const std::vector<std::unique_ptr<RigidBody>>& bodies() const { return bodies_; }

    void step(float dt);

private:
    void integrateVelocities(float dt);
    void generateContacts();
    void solveVelocities();
    void integratePositions(float dt);
    void correctPositions();

    std::vector<std::unique_ptr<RigidBody>> bodies_;
    std::vector<Plane> planes_;
    std::vector<Contact> contacts_;
};

} // namespace physics
