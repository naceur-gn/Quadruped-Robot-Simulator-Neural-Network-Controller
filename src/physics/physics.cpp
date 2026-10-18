#include "physics/physics.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace physics {

namespace {
constexpr float kMaxLinearSpeed = 100.0f;
constexpr float kMaxAngularSpeed = 80.0f;
constexpr float kMinEffectiveMass = 1e-8f;

math::Vec3 anyPerpendicular(const math::Vec3& n) {
    math::Vec3 axis = (std::fabs(n.x) < 0.9f) ? math::Vec3{1.0f, 0.0f, 0.0f}
                                              : math::Vec3{0.0f, 1.0f, 0.0f};
    return math::cross(n, axis).normalized();
}
} // namespace

// ---------------------------------------------------------------------------
// RigidBody
// ---------------------------------------------------------------------------

math::Mat3 RigidBody::inverseInertiaWorld() const {
    // R * diag(invInertiaLocal) * R^T, built from the rotation columns.
    const math::Mat3 rot = orientation.toMat3();
    return math::Mat3::outerProduct(rot.c0, rot.c0) * invInertiaLocal.x +
           math::Mat3::outerProduct(rot.c1, rot.c1) * invInertiaLocal.y +
           math::Mat3::outerProduct(rot.c2, rot.c2) * invInertiaLocal.z;
}

math::Vec3 RigidBody::velocityAtPoint(const math::Vec3& worldPoint) const {
    return linearVelocity + math::cross(angularVelocity, worldPoint - position);
}

void RigidBody::applyForce(const math::Vec3& f) { force += f; }

void RigidBody::applyForceAtPoint(const math::Vec3& f, const math::Vec3& worldPoint) {
    force += f;
    torque += math::cross(worldPoint - position, f);
}

void RigidBody::applyTorque(const math::Vec3& t) { torque += t; }

void RigidBody::applyImpulseAtPoint(const math::Vec3& impulse, const math::Vec3& worldPoint) {
    if (isStatic()) return;
    linearVelocity += impulse * invMass;
    angularVelocity += inverseInertiaWorld() * math::cross(worldPoint - position, impulse);
}

void RigidBody::clearForces() {
    force = {0.0f, 0.0f, 0.0f};
    torque = {0.0f, 0.0f, 0.0f};
}

// ---------------------------------------------------------------------------
// World
// ---------------------------------------------------------------------------

RigidBody& World::addBody(RigidBody body) {
    if (body.mass <= 0.0f) {
        body.mass = 0.0f;
        body.invMass = 0.0f;
        body.invInertiaLocal = {0.0f, 0.0f, 0.0f};
    }
    bodies_.push_back(std::make_unique<RigidBody>(std::move(body)));
    return *bodies_.back();
}

RigidBody& World::addBox(float mass, const math::Vec3& halfExtents,
                         const math::Vec3& position, const math::Quat& orientation) {
    RigidBody body;
    body.shape = RigidBody::Shape::Box;
    body.halfExtents = halfExtents;
    body.mass = mass;
    body.invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;

    // Principal moments of a solid box: I = m/12 * (w^2 + d^2) per axis
    // (w = 2 * halfExtent).
    const float m = mass;
    const float hx = halfExtents.x, hy = halfExtents.y, hz = halfExtents.z;
    body.inertiaLocal = {
        m / 3.0f * (hy * hy + hz * hz),
        m / 3.0f * (hx * hx + hz * hz),
        m / 3.0f * (hx * hx + hy * hy),
    };
    body.invInertiaLocal = {1.0f / body.inertiaLocal.x,
                            1.0f / body.inertiaLocal.y,
                            1.0f / body.inertiaLocal.z};
    body.position = position;
    body.orientation = orientation.normalized();
    body.restitution = 0.15f;
    body.friction = 0.55f;
    return addBody(body);
}

RigidBody& World::addSphere(float mass, float radius, const math::Vec3& position,
                            const math::Quat& orientation) {
    RigidBody body;
    body.shape = RigidBody::Shape::Sphere;
    body.radius = radius;
    body.mass = mass;
    body.invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;

    const float inertia = 0.4f * mass * radius * radius; // 2/5 * m * r^2
    body.inertiaLocal = {inertia, inertia, inertia};
    body.invInertiaLocal = {1.0f / inertia, 1.0f / inertia, 1.0f / inertia};
    body.position = position;
    body.orientation = orientation.normalized();
    body.restitution = 0.55f;
    body.friction = 0.5f;
    return addBody(body);
}

void World::addGroundPlane(const Plane& plane) { planes_.push_back(plane); }

void World::step(float dt) {
    integrateVelocities(dt);

    contacts_.clear();
    generateContacts();

    for (int i = 0; i < solverIterations; ++i) {
        solveVelocities();
    }

    integratePositions(dt);
    correctPositions();
}

void World::integrateVelocities(float dt) {
    for (const std::unique_ptr<RigidBody>& bodyPtr : bodies_) {
        RigidBody& body = *bodyPtr;
        if (body.isStatic()) continue;

        const math::Vec3 acceleration = gravity + body.force * body.invMass;
        body.linearVelocity += acceleration * dt;
        body.angularVelocity += body.inverseInertiaWorld() * (body.torque * dt);

        body.linearVelocity *= std::max(0.0f, 1.0f - body.linearDamping * dt);
        body.angularVelocity *= std::max(0.0f, 1.0f - body.angularDamping * dt);

        const float linearSpeedSq = body.linearVelocity.lengthSq();
        if (linearSpeedSq > kMaxLinearSpeed * kMaxLinearSpeed) {
            body.linearVelocity = body.linearVelocity.normalized() * kMaxLinearSpeed;
        }
        const float angularSpeedSq = body.angularVelocity.lengthSq();
        if (angularSpeedSq > kMaxAngularSpeed * kMaxAngularSpeed) {
            body.angularVelocity = body.angularVelocity.normalized() * kMaxAngularSpeed;
        }

        body.clearForces();
    }
}

void World::generateContacts() {
    auto initContact = [this](RigidBody& body, const math::Vec3& point,
                              const math::Vec3& normal, float penetration) {
        Contact contact;
        contact.body = &body;
        contact.point = point;
        contact.normal = normal;
        contact.penetration = penetration;
        contact.r = point - body.position;

        const math::Mat3 inverseInertia = body.inverseInertiaWorld();
        const math::Vec3 relativeVelocity = body.velocityAtPoint(point);

        const float normalVelocity = math::dot(relativeVelocity, normal);
        contact.targetNormalVelocity =
            (normalVelocity < -restitutionVelocityThreshold)
                ? -body.restitution * normalVelocity
                : 0.0f;

        const math::Vec3 rCrossN = math::cross(contact.r, normal);
        contact.normalMass =
            body.invMass + math::dot(math::cross(inverseInertia * rCrossN, contact.r), normal);
        contact.normalMass = std::max(contact.normalMass, kMinEffectiveMass);

        const math::Vec3 tangentialVelocity = relativeVelocity - normal * normalVelocity;
        contact.tangent = (tangentialVelocity.lengthSq() > 1e-8f)
                              ? tangentialVelocity.normalized()
                              : anyPerpendicular(normal);

        const math::Vec3 rCrossT = math::cross(contact.r, contact.tangent);
        contact.tangentMass =
            body.invMass + math::dot(math::cross(inverseInertia * rCrossT, contact.r), contact.tangent);
        contact.tangentMass = std::max(contact.tangentMass, kMinEffectiveMass);

        contacts_.push_back(contact);
    };

    for (const std::unique_ptr<RigidBody>& bodyPtr : bodies_) {
        RigidBody& body = *bodyPtr;
        if (body.isStatic()) continue;

        for (const Plane& plane : planes_) {
            const math::Vec3& n = plane.normal;

            if (body.shape == RigidBody::Shape::Sphere) {
                const float distance = math::dot(n, body.position) - plane.offset;
                if (distance < body.radius) {
                    const math::Vec3 point = body.position - n * body.radius;
                    initContact(body, point, n, body.radius - distance);
                }
                continue;
            }

            // Box: the ground only has to be tested against the 8 corners.
            for (int sx = -1; sx <= 1; sx += 2) {
                for (int sy = -1; sy <= 1; sy += 2) {
                    for (int sz = -1; sz <= 1; sz += 2) {
                        const math::Vec3 localCorner{
                            static_cast<float>(sx) * body.halfExtents.x,
                            static_cast<float>(sy) * body.halfExtents.y,
                            static_cast<float>(sz) * body.halfExtents.z,
                        };
                        const math::Vec3 corner =
                            body.position + body.orientation.rotate(localCorner);
                        const float distance = math::dot(n, corner) - plane.offset;
                        if (distance < 0.0f) {
                            initContact(body, corner, n, -distance);
                        }
                    }
                }
            }
        }
    }
}

void World::solveVelocities() {
    for (Contact& contact : contacts_) {
        RigidBody& body = *contact.body;

        // Normal impulse (with accumulated clamp >= 0).
        {
            const math::Vec3 relativeVelocity = body.velocityAtPoint(contact.point);
            const float normalVelocity = math::dot(relativeVelocity, contact.normal);
            const float lambda =
                (contact.targetNormalVelocity - normalVelocity) / contact.normalMass;

            const float newJn = std::max(contact.jn + lambda, 0.0f);
            const float delta = newJn - contact.jn;
            contact.jn = newJn;
            if (delta != 0.0f) {
                body.applyImpulseAtPoint(contact.normal * delta, contact.point);
            }
        }

        // Coulomb friction along the (fixed) tangent direction.
        {
            const math::Vec3 relativeVelocity = body.velocityAtPoint(contact.point);
            const float tangentialVelocity = math::dot(relativeVelocity, contact.tangent);
            const float lambda = -tangentialVelocity / contact.tangentMass;

            const float maxFriction = body.friction * contact.jn;
            const float newJt =
                math::clampf(contact.jt + lambda, -maxFriction, maxFriction);
            const float delta = newJt - contact.jt;
            contact.jt = newJt;
            if (delta != 0.0f) {
                body.applyImpulseAtPoint(contact.tangent * delta, contact.point);
            }
        }
    }
}

void World::integratePositions(float dt) {
    for (const std::unique_ptr<RigidBody>& bodyPtr : bodies_) {
        RigidBody& body = *bodyPtr;
        if (body.isStatic()) continue;

        body.position += body.linearVelocity * dt;

        const math::Quat spin{0.0f, body.angularVelocity.x, body.angularVelocity.y,
                              body.angularVelocity.z};
        const math::Quat delta = spin * body.orientation;
        body.orientation = (body.orientation + delta * (0.5f * dt)).normalized();
    }
}

void World::correctPositions() {
    // Lift each body once by its deepest penetration (avoids N-corner overshoot).
    struct Correction {
        RigidBody* body;
        float penetration;
        math::Vec3 normal;
    };

    std::vector<Correction> corrections;
    for (const Contact& contact : contacts_) {
        const float penetration = contact.penetration - penetrationSlop;
        if (penetration <= 0.0f) continue;

        bool found = false;
        for (Correction& correction : corrections) {
            if (correction.body == contact.body) {
                if (penetration > correction.penetration) {
                    correction.penetration = penetration;
                    correction.normal = contact.normal;
                }
                found = true;
                break;
            }
        }
        if (!found) {
            corrections.push_back({contact.body, penetration, contact.normal});
        }
    }

    for (const Correction& correction : corrections) {
        correction.body->position +=
            correction.normal * (correction.penetration * positionCorrectionRate);
    }
}

} // namespace physics
