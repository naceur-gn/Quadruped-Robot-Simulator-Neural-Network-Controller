#include "scene/scene.hpp"

#include "render/shaders.hpp"

#include <cstdio>

bool Scene::init(const SceneConfig& config) {
    config_ = config;
    lightDirection_ = config_.sunDirection.normalized();

    if (!objectShader_.compile(kObjectVertexShader, kObjectFragmentShader, "object")) {
        return false;
    }
    if (!groundShader_.compile(kObjectVertexShader, kGroundFragmentShader, "ground")) {
        return false;
    }

    groundMesh_ = meshes::makeGroundPlane();
    boxMesh_ = meshes::makeUnitBox();
    sphereMesh_ = meshes::makeUnitSphere();
    diskMesh_ = meshes::makeFlatDisk();

    // The world floor is an infinite plane at y = 0.
    physicsWorld_.addGroundPlane(physics::Plane{math::Vec3{0.0f, 1.0f, 0.0f}, 0.0f});

    // Clean limited square arena: only the robot (no demo spheres or cubes)
    robot_.setPosition({0.0f, 0.0f, 0.0f}); // Center of the square platform
    return groundMesh_.valid() && boxMesh_.valid() && sphereMesh_.valid() && diskMesh_.valid();
}

void Scene::update(float dt) {
    physicsWorld_.step(dt);
    robot_.update(dt);
}

void Scene::render(const Camera& camera, float aspect) {
    const math::Mat4 view = camera.view();
    const math::Mat4 projection = camera.projection(aspect);
    const math::Mat4 viewProjection = projection * view;

    // Ground: one large quad, pattern computed per-pixel in the shader.
    groundShader_.use();
    groundShader_.setMat4("uViewProj", viewProjection);
    groundShader_.setMat4(
        "uModel",
        math::Mat4::scale({config_.groundSize, 1.0f, config_.groundSize}));
    groundShader_.setVec3("uLightDir", lightDirection_);
    groundShader_.setVec3("uCamPos", camera.position);
    groundShader_.setFloat("uTileSize", config_.tileSize);
    groundShader_.setVec3("uColorWhite", config_.whiteSquare);
    groundShader_.setVec3("uColorViolet", config_.violetSquare);
    groundMesh_.draw();

    // 3D base platform slab under the limited square ground
    objectShader_.use();
    objectShader_.setMat4("uViewProj", viewProjection);
    objectShader_.setVec3("uLightDir", lightDirection_);
    objectShader_.setVec3("uCamPos", camera.position);

    const float halfG = config_.groundSize * 0.5f;
    constexpr float kSlabDepth = 0.40f;
    const math::Mat4 slabModel = math::Mat4::translation({0.0f, -kSlabDepth * 0.5f, 0.0f}) *
                                 math::Mat4::scale({config_.groundSize, kSlabDepth, config_.groundSize});
    objectShader_.setMat4("uModel", slabModel);
    objectShader_.setVec3("uColor", {0.22f, 0.24f, 0.27f}); // Sleek dark slate base
    boxMesh_.draw();

    // Sleek border curb / edge framing around the 4 edges of the limited square arena
    constexpr float kCurbW = 0.16f;
    constexpr float kCurbH = 0.04f;
    const math::Vec3 curbColor{0.16f, 0.18f, 0.21f}; // Dark graphite trim

    // North edge (-Z)
    objectShader_.setMat4("uModel", math::Mat4::translation({0.0f, kCurbH * 0.5f, -halfG + kCurbW * 0.5f}) *
                                    math::Mat4::scale({config_.groundSize, kCurbH, kCurbW}));
    objectShader_.setVec3("uColor", curbColor);
    boxMesh_.draw();

    // South edge (+Z)
    objectShader_.setMat4("uModel", math::Mat4::translation({0.0f, kCurbH * 0.5f, halfG - kCurbW * 0.5f}) *
                                    math::Mat4::scale({config_.groundSize, kCurbH, kCurbW}));
    objectShader_.setVec3("uColor", curbColor);
    boxMesh_.draw();

    // West edge (-X)
    objectShader_.setMat4("uModel", math::Mat4::translation({-halfG + kCurbW * 0.5f, kCurbH * 0.5f, 0.0f}) *
                                    math::Mat4::scale({kCurbW, kCurbH, config_.groundSize}));
    objectShader_.setVec3("uColor", curbColor);
    boxMesh_.draw();

    // East edge (+X)
    objectShader_.setMat4("uModel", math::Mat4::translation({halfG - kCurbW * 0.5f, kCurbH * 0.5f, 0.0f}) *
                                    math::Mat4::scale({kCurbW, kCurbH, config_.groundSize}));
    objectShader_.setVec3("uColor", curbColor);
    boxMesh_.draw();

    // Dynamic soft trunk ambient occlusion shadow beneath the robot
    const math::Vec3 robotPos = robot_.position();
    const float robotYaw = robot_.yaw();

    // Elongated sun-cast shadow (projected along light direction onto ground)
    const float shadowStretch = 1.0f + std::abs(lightDirection_.x) * 0.5f;
    const float shadowSkewX = lightDirection_.x * 0.4f;
    const float shadowSkewZ = lightDirection_.z * 0.3f;
    const math::Mat4 trunkShadow = math::Mat4::translation({robotPos.x + shadowSkewX, 0.004f, robotPos.z + shadowSkewZ}) *
                                   math::Mat4::rotation(math::Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, -robotYaw)) *
                                   math::Mat4::scale({shadowStretch * 1.45f * robot_.scale * 0.42f, 1.0f, 0.85f * robot_.scale * 0.42f});
    objectShader_.setMat4("uModel", trunkShadow);
    objectShader_.setVec3("uColor", {0.32f, 0.34f, 0.38f}); // Soft ambient shadow
    diskMesh_.draw();

    // Dynamic foot contact shadows + impact flash rings for each of the 4 legs
    for (int leg = 0; leg < 4; ++leg) {
        const math::Vec3 fWorld = robot_.footWorldPosition(leg);
        const float footH = std::max(0.0f, fWorld.y - 0.02f * robot_.scale);
        const float shadowRadius = (0.075f + footH * 0.35f) * robot_.scale;
        const float alpha = math::clampf(1.0f - footH * 3.5f, 0.20f, 1.0f);
        const math::Vec3 shadowColor = math::Vec3{0.52f, 0.54f, 0.58f} * (1.0f - alpha) +
                                       math::Vec3{0.18f, 0.19f, 0.22f} * alpha;

        // Foot contact shadow
        const math::Mat4 footShadow = math::Mat4::translation({fWorld.x, 0.006f, fWorld.z}) *
                                      math::Mat4::scale({shadowRadius, 1.0f, shadowRadius});
        objectShader_.setMat4("uModel", footShadow);
        objectShader_.setVec3("uColor", shadowColor);
        diskMesh_.draw();

        // Foot-strike impact ring: flash white on landing, quickly fade and expand
        const float ct = robot_.footContactTimer(leg);
        if (ct < 0.25f) {
            const float impactT = ct / 0.25f; // 0..1 over 250ms
            const float ringRadius = (0.08f + 0.22f * impactT) * robot_.scale;
            const float ringAlpha = (1.0f - impactT) * (1.0f - impactT); // quadratic fade
            const math::Vec3 ringColor = math::Vec3{0.95f, 0.97f, 1.0f} * ringAlpha +
                                         math::Vec3{0.45f, 0.47f, 0.50f} * (1.0f - ringAlpha);

            const math::Mat4 ringModel = math::Mat4::translation({fWorld.x, 0.008f, fWorld.z}) *
                                         math::Mat4::scale({ringRadius, 1.0f, ringRadius});
            objectShader_.setMat4("uModel", ringModel);
            objectShader_.setVec3("uColor", ringColor);
            diskMesh_.draw();
        }
    }

    // Quadruped robot: boxes and spheres posed by forward kinematics.
    std::vector<robot::DrawPart> parts;
    robot_.collectParts(parts);
    for (const robot::DrawPart& part : parts) {
        objectShader_.setMat4("uModel", part.model);
        objectShader_.setVec3("uColor", part.color);
        if (part.sphere) {
            sphereMesh_.draw();
        } else {
            boxMesh_.draw();
        }
    }

    // Target coordinate marker and illuminated directional pathway
    const math::Vec3 target = robot_.targetCoordinate();
    const math::Vec3 toTarget = target - robotPos;
    const float pathDist = std::sqrt(toTarget.x * toTarget.x + toTarget.z * toTarget.z);

    // Animated time for pulsing effects
    animTime_ += 0.016f; // ~60 FPS accumulation

    // Glowing directional ground pathway with animated pulse
    if (pathDist > 0.4f) {
        const int numDashes = std::min(12, std::max(2, static_cast<int>(pathDist / 0.7f)));
        for (int i = 1; i <= numDashes; ++i) {
            const float t = static_cast<float>(i) / (numDashes + 1);
            // Animated wave pulse traveling along the path
            const float wave = 0.7f + 0.3f * std::sin(animTime_ * 4.0f - t * 6.28f);
            const math::Vec3 dashPos{
                robotPos.x + toTarget.x * t,
                0.025f,
                robotPos.z + toTarget.z * t
            };
            const float dashScale = 0.14f + 0.06f * wave;
            const math::Mat4 dashModel = math::Mat4::translation(dashPos) *
                                         math::Mat4::scale({dashScale, 0.03f, dashScale});
            objectShader_.setMat4("uModel", dashModel);
            // Pulsing cyan-emerald with brightness wave
            const math::Vec3 pathColor{0.12f * wave, 0.85f * wave, 0.55f * wave};
            objectShader_.setVec3("uColor", pathColor);
            diskMesh_.draw(); // Use disk instead of box for smoother path dots
        }
    }

    // Animated emerald destination beacon with pulsing glow
    const float beaconPulse = 0.85f + 0.15f * std::sin(animTime_ * 3.0f);
    constexpr float kBeaconHeight = 0.7f;

    // Orbital glow ring around beacon base
    const float orbRadius = 0.55f + 0.1f * std::sin(animTime_ * 2.0f);
    const math::Mat4 orbRing = math::Mat4::translation({target.x, 0.012f, target.z}) *
                               math::Mat4::rotation(math::Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, animTime_ * 1.5f)) *
                               math::Mat4::scale({orbRadius, 1.0f, orbRadius * 0.6f});
    objectShader_.setMat4("uModel", orbRing);
    objectShader_.setVec3("uColor", {0.08f, 0.75f * beaconPulse, 0.40f * beaconPulse});
    diskMesh_.draw();

    // Beacon sphere (pulsing size)
    const float sphereScale = 0.55f * beaconPulse;
    const math::Mat4 beaconSphere = math::Mat4::translation({target.x, kBeaconHeight, target.z}) *
                                    math::Mat4::scale({sphereScale, sphereScale, sphereScale});
    objectShader_.setMat4("uModel", beaconSphere);
    objectShader_.setVec3("uColor", {0.10f * beaconPulse, 0.98f * beaconPulse, 0.50f * beaconPulse});
    sphereMesh_.draw();

    // Pillar
    const math::Mat4 beaconPillar = math::Mat4::translation({target.x, kBeaconHeight * 0.5f, target.z}) *
                                    math::Mat4::scale({0.12f, kBeaconHeight, 0.12f});
    objectShader_.setMat4("uModel", beaconPillar);
    objectShader_.setVec3("uColor", {0.05f, 0.70f * beaconPulse, 0.35f * beaconPulse});
    boxMesh_.draw();

    // Ground target ring
    const math::Mat4 beaconBase = math::Mat4::translation({target.x, 0.010f, target.z}) *
                                  math::Mat4::scale({0.60f, 1.0f, 0.60f});
    objectShader_.setMat4("uModel", beaconBase);
    objectShader_.setVec3("uColor", {0.10f, 0.95f * beaconPulse, 0.55f * beaconPulse});
    diskMesh_.draw();
}

bool Scene::tryPushRobot(const Camera& camera, float aspect,
                         float mouseNdcX, float mouseNdcY,
                         float pushStrength) {
    using namespace math;

    // ---- build the pick ray in world space ---------------------------------
    // 1. NDC → clip space (on the near plane)
    // 2. clip → eye space  (invert projection)
    // 3. eye → world space (invert view)

    const Mat4 proj = camera.projection(aspect);
    const Mat4 view = camera.view();

    // Invert the projection for the x/y part (symmetric perspective).
    const float eyeX = mouseNdcX / proj.m[0];
    const float eyeY = mouseNdcY / proj.m[5];
    Vec3 eyeDir{eyeX, eyeY, -1.0f};

    Vec3 worldDir;
    worldDir.x = view.m[0] * eyeDir.x + view.m[4] * eyeDir.y + view.m[8]  * eyeDir.z;
    worldDir.y = view.m[1] * eyeDir.x + view.m[5] * eyeDir.y + view.m[9]  * eyeDir.z;
    worldDir.z = view.m[2] * eyeDir.x + view.m[6] * eyeDir.y + view.m[10] * eyeDir.z;
    worldDir = worldDir.normalized();

    const Vec3 rayOrigin = camera.position;

    // ---- sphere intersection with robot bounding volume --------------------
    const Vec3 robotCenter = robot_.position() + Vec3{0.0f, 0.4f * robot_.scale, 0.0f};
    const float robotRadius = 1.2f * robot_.scale;  // bounding sphere

    const Vec3 oc = rayOrigin - robotCenter;
    const float a = dot(worldDir, worldDir);       // == 1 for normalised dir
    const float b = 2.0f * dot(oc, worldDir);
    const float c = dot(oc, oc) - robotRadius * robotRadius;
    const float discriminant = b * b - 4.0f * a * c;

    const float halfG = config_.groundSize * 0.5f - 0.6f;

    if (discriminant < 0.0f) {
        // Missed robot: test if ray hits ground plane (Y = 0) to set target coordinate
        if (worldDir.y < -1e-4f) {
            const float tGround = -rayOrigin.y / worldDir.y;
            if (tGround > 0.0f) {
                Vec3 groundPoint = rayOrigin + worldDir * tGround;
                // Clamp within limited square arena
                groundPoint.x = math::clampf(groundPoint.x, -halfG, halfG);
                groundPoint.z = math::clampf(groundPoint.z, -halfG, halfG);
                groundPoint.y = 0.0f;
                robot_.setTargetCoordinate(groundPoint);
                std::printf(">>> [TARGET SET] Ground clicked at (%.2f, %.2f) | Robot distance: %.2fm <<<\n",
                            groundPoint.x, groundPoint.z, robot_.distanceToTarget());
                return true;
            }
        }
        return false;
    }

    const float t = (-b - std::sqrt(discriminant)) / (2.0f * a);
    if (t < 0.0f) {
        // Missed robot: test ground
        if (worldDir.y < -1e-4f) {
            const float tGround = -rayOrigin.y / worldDir.y;
            if (tGround > 0.0f) {
                Vec3 groundPoint = rayOrigin + worldDir * tGround;
                groundPoint.x = math::clampf(groundPoint.x, -halfG, halfG);
                groundPoint.z = math::clampf(groundPoint.z, -halfG, halfG);
                groundPoint.y = 0.0f;
                robot_.setTargetCoordinate(groundPoint);
                std::printf(">>> [TARGET SET] Ground clicked at (%.2f, %.2f) | Robot distance: %.2fm <<<\n",
                            groundPoint.x, groundPoint.z, robot_.distanceToTarget());
                return true;
            }
        }
        return false;
    }

    // ---- compute push direction from the hit point -------------------------
    const Vec3 hitPoint = rayOrigin + worldDir * t;
    const Vec3 rTrunk = (hitPoint - robotCenter) / robot_.scale;

    // Apply linear impulse along the ray direction (pushing into the robot)
    const Vec3 impulse = worldDir * (pushStrength * 8.0f);
    robot_.applyImpulse(impulse, rTrunk);

    const auto& forces = robot_.legForces();
    const auto& torques = robot_.jointTorques();
    float maxTorque = 0.0f;
    for (float tq : torques) maxTorque = std::max(maxTorque, std::abs(tq));

    const auto cp = robot_.capturePoint();
    const bool inPoly = robot_.isCapturePointInPolygon();

    std::printf("[push] hit at (%.2f, %.2f, %.2f) | impulse (%.1f, %.1f, %.1f) N*s\n"
                "       capture point: (%.2f, %.2f) | in support polygon: %s\n"
                "       leg forces: FL=%.1f N, FR=%.1f N, RL=%.1f N, RR=%.1f N | peak joint torque: %.1f N*m\n",
                hitPoint.x, hitPoint.y, hitPoint.z, impulse.x, impulse.y, impulse.z,
                cp.x, cp.z, inPoly ? "YES (in polygon)" : "NO (stepping)",
                forces[0], forces[1], forces[2], forces[3], maxTorque);

    return true;
}