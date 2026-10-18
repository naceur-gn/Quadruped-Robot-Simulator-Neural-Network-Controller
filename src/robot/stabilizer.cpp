#include "robot/stabilizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace robot {

namespace {

// 2D cross product of vectors AB and AC: (B.x - A.x)*(C.z - A.z) - (B.z - A.z)*(C.x - A.x)
inline float cross2D(const math::Vec3& a, const math::Vec3& b, const math::Vec3& c) {
    return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}

} // namespace

void Stabilizer::push(float pitchImpulse, float rollImpulse) {
    pitchVel_ += pitchImpulse;
    rollVel_  += rollImpulse;
    // Pushing also directly impacts CoM horizontal velocity (LIPM perturbation)
    comVel_.x += -pitchImpulse * zc * 0.4f;
    comVel_.z +=  rollImpulse  * zc * 0.4f;
}

void Stabilizer::applyImpulse(const math::Vec3& impulse, const math::Vec3& r) {
    // Linear velocity changes
    comVel_.x  += impulse.x / trunkMass;
    heightVel_ += impulse.y / trunkMass;
    comVel_.z  += impulse.z / trunkMass;

    // Angular impulse tau = r x impulse
    // Pitch about Z: r.x * impulse.y - r.y * impulse.x
    const float pitchTorqueImpulse = r.x * impulse.y - r.y * impulse.x;
    pitchVel_ += pitchTorqueImpulse / pitchInertia;

    // Roll about X: r.y * impulse.z - r.z * impulse.y
    const float rollTorqueImpulse = r.y * impulse.z - r.z * impulse.y;
    rollVel_ += rollTorqueImpulse / rollInertia;

    // Yaw about Y: r.z * impulse.x - r.x * impulse.z
    const float yawTorqueImpulse = r.z * impulse.x - r.x * impulse.z;
    yawVel_ += yawTorqueImpulse / yawInertia;
}

void Stabilizer::reset() {
    pitch_       = 0.0f;
    roll_        = 0.0f;
    yaw_         = 0.0f;
    pitchVel_    = 0.0f;
    rollVel_     = 0.0f;
    yawVel_      = 0.0f;
    targetYaw_   = 0.0f;
    heightDelta_ = 0.0f;
    heightVel_   = 0.0f;

    comPos_      = {0.0f, 0.0f, 0.0f};
    comVel_      = {0.0f, 0.0f, 0.0f};

    intRollErr_  = 0.0f;
    intPitchErr_ = 0.0f;

    accumForce_  = {0.0f, 0.0f, 0.0f};
    accumTorque_ = {0.0f, 0.0f, 0.0f};

    inStance_.fill(true);
    cpInPolygon_      = true;
    recoveryStepping_ = false;
    steppingLeg_      = -1;
    stepTimer_        = 0.0f;

    const float nomLoad = (trunkMass * gravity) / 4.0f;
    for (int i = 0; i < kLegs; ++i) {
        qpForces_[i] = math::Vec3{0.0f, nomLoad, 0.0f};
    }
    jointTorques_.fill(0.0f);
    jointOffsets_.fill(0.0f);
}

math::Vec3 Stabilizer::capturePoint() const {
    const float omega = omegaLIPM();
    // xi = c + dot_c / omega (in horizontal X-Z plane)
    return math::Vec3{
        comPos_.x + comVel_.x / omega,
        0.0f,
        comPos_.z + comVel_.z / omega
    };
}

void Stabilizer::computeLegJacobian(float qHip, float qKnee, float thigh, float calf, float J[2][2]) {
    const float s1  = std::sin(qHip);
    const float c1  = std::cos(qHip);
    const float s12 = std::sin(qHip + qKnee);
    const float c12 = std::cos(qHip + qKnee);

    // x = thigh * s1 + calf * s12
    // y = -thigh * c1 - calf * c12
    J[0][0] = thigh * c1 + calf * c12;  // dx / dqHip
    J[0][1] = calf * c12;               // dx / dqKnee
    J[1][0] = thigh * s1 + calf * s12;  // dy / dqHip
    J[1][1] = calf * s12;               // dy / dqKnee
}

bool Stabilizer::solve2LinkIK(float x, float y, float thigh, float calf, float& outHip, float& outKnee) {
    const float d = std::sqrt(x * x + y * y);
    const float maxReach = (thigh + calf) - 0.002f;
    const float clampedD = math::clampf(d, 0.05f, maxReach);

    const float cosPsi = math::clampf(clampedD / (thigh + calf), -1.0f, 1.0f);
    const float psi = std::acos(cosPsi);
    const float alpha = std::atan2(x, -y);

    outHip  = psi + alpha;
    outKnee = -2.0f * psi;
    return true;
}

bool Stabilizer::testSupportPolygon(const math::Vec3& xi) const {
    // Gather current stance foot ground positions
    std::vector<math::Vec3> poly;
    poly.reserve(kLegs);

    // Stance foot nominal layout relative to robot CoM
    // Leg 0: FL (+hipX, -hipZ)
    // Leg 1: FR (+hipX, +hipZ)
    // Leg 3: RR (-hipX, +hipZ)
    // Leg 2: RL (-hipX, -hipZ)
    // Ordered CCW: FL -> FR -> RR -> RL
    constexpr int ccwOrder[4] = {0, 1, 3, 2};

    for (int idx : ccwOrder) {
        if (inStance_[idx]) {
            poly.push_back(stanceFeet_[idx]);
        }
    }

    if (poly.size() < 3) {
        // Less than 3 feet: linear support polygon during trot
        if (poly.size() == 2) {
            // Distance to line segment
            const math::Vec3 p1 = poly[0];
            const math::Vec3 p2 = poly[1];
            const math::Vec3 seg = p2 - p1;
            const float lenSq = seg.x * seg.x + seg.z * seg.z;
            if (lenSq < 1e-6f) return false;
            const float t = math::clampf(((xi.x - p1.x) * seg.x + (xi.z - p1.z) * seg.z) / lenSq, 0.0f, 1.0f);
            const math::Vec3 proj{p1.x + t * seg.x, 0.0f, p1.z + t * seg.z};
            const float dist = std::sqrt((xi.x - proj.x) * (xi.x - proj.x) + (xi.z - proj.z) * (xi.z - proj.z));
            return dist < 0.04f;
        }
        return false;
    }

    // Check if xi is inside the convex polygon (left of every CCW edge)
    constexpr float kSafetyMargin = -0.015f; // small margin for early stepping
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const math::Vec3& vCurr = poly[i];
        const math::Vec3& vNext = poly[(i + 1) % n];
        if (cross2D(vCurr, vNext, xi) < kSafetyMargin) {
            return false;
        }
    }
    return true;
}

void Stabilizer::planRecoveryStep(const math::Vec3& xi) {
    if (recoveryStepping_) return;

    // Divergence direction relative to CoM
    const float dx = xi.x - comPos_.x;
    const float dz = xi.z - comPos_.z;

    // Choose the leg closest to the fall direction
    if (dx >= 0.0f && dz <= 0.0f) steppingLeg_ = 0; // Front-Left
    else if (dx >= 0.0f && dz >  0.0f) steppingLeg_ = 1; // Front-Right
    else if (dx <  0.0f && dz <= 0.0f) steppingLeg_ = 2; // Rear-Left
    else                               steppingLeg_ = 3; // Rear-Right

    recoveryStepping_ = true;
    inStance_[steppingLeg_] = false;
    stepTimer_ = 0.0f;
    stepStartPos_ = stanceFeet_[steppingLeg_];

    // Nominal hip ground projection
    const float hx = (steppingLeg_ == 0 || steppingLeg_ == 1) ? hipX : -hipX;
    const float hz = (steppingLeg_ == 0 || steppingLeg_ == 2) ? -hipZ : hipZ;
    const math::Vec3 hipNominal{comPos_.x + hx, 0.0f, comPos_.z + hz};

    // Capture-point foot placement rule + Raibert velocity correction:
    //   p_step = xi + K_cp * (xi - c) + (dot_c * T_s) / 2
    math::Vec3 target{
        xi.x + kCp * (xi.x - comPos_.x) + (comVel_.x * stepDuration) * 0.5f,
        0.0f,
        xi.z + kCp * (xi.z - comPos_.z) + (comVel_.z * stepDuration) * 0.5f
    };

    // Clamp inside reachable leg workspace
    const float reachX = target.x - hipNominal.x;
    const float reachZ = target.z - hipNominal.z;
    const float dist = std::sqrt(reachX * reachX + reachZ * reachZ);
    if (dist > maxReach) {
        target.x = hipNominal.x + (reachX / dist) * maxReach;
        target.z = hipNominal.z + (reachZ / dist) * maxReach;
    }
    stepTargetPos_ = target;
}

math::Vec3 Stabilizer::evaluateSwingTrajectory(float tNorm) const {
    const float tau = math::clampf(tNorm, 0.0f, 1.0f);
    // 5th-order minimum-jerk polynomial: s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    const float s = tau * tau * tau * (10.0f - 15.0f * tau + 6.0f * tau * tau);

    // Horizontal interpolation
    const float x = stepStartPos_.x + s * (stepTargetPos_.x - stepStartPos_.x);
    const float z = stepStartPos_.z + s * (stepTargetPos_.z - stepStartPos_.z);

    // Vertical clearance: smooth sine profile h_step * sin(pi * tau)
    const float y = stepHeight * std::sin(math::PI * tau);

    return math::Vec3{x, y, z};
}

Stabilizer::JointArray Stabilizer::update(float dt) {
    using math::Quat;
    using math::Vec3;

    // --- 1. State Estimation & Orientation Quaternions -----------------------
    const Quat tiltQ = Quat::fromAxisAngle({0.0f, 0.0f, 1.0f}, pitch_) *
                       Quat::fromAxisAngle({1.0f, 0.0f, 0.0f}, roll_);

    const float curTrunkHeight = nominalHeight + heightDelta_;

    // Leg geometry flags:
    // Leg 0: FL (+hipX, -hipZ)
    // Leg 1: FR (+hipX, +hipZ)
    // Leg 2: RL (-hipX, -hipZ)
    // Leg 3: RR (-hipX, +hipZ)
    constexpr float kLegX[4] = { 1.0f,  1.0f, -1.0f, -1.0f};
    constexpr float kLegZ[4] = {-1.0f,  1.0f, -1.0f,  1.0f};

    // Update stance feet ground positions
    for (int leg = 0; leg < kLegs; ++leg) {
        if (inStance_[leg]) {
            stanceFeet_[leg] = math::Vec3{
                comPos_.x + kLegX[leg] * hipX,
                0.0f,
                comPos_.z + kLegZ[leg] * hipZ
            };
        }
    }

    // --- 2. LIPM & Capture Point --------------------------------------------
    const Vec3 xi = capturePoint();
    cpInPolygon_ = testSupportPolygon(xi);

    // If capture point is outside support polygon, initiate recovery step!
    if (!cpInPolygon_ && !recoveryStepping_) {
        planRecoveryStep(xi);
    }

    // Advance recovery step swing trajectory if active
    Vec3 swingFootLocal{0.0f, 0.0f, 0.0f};
    if (recoveryStepping_) {
        stepTimer_ += dt;
        const float tNorm = stepTimer_ / stepDuration;
        const Vec3 swingFootWorld = evaluateSwingTrajectory(tNorm);

        // Compute hip position in world for stepping leg
        const Vec3 hipLocal{kLegX[steppingLeg_] * hipX, 0.0f, kLegZ[steppingLeg_] * hipZ};
        const Vec3 hipWorld = comPos_ + tiltQ.rotate(hipLocal) + Vec3{0.0f, curTrunkHeight, 0.0f};
        swingFootLocal = swingFootWorld - hipWorld;

        if (tNorm >= 1.0f) {
            // Touchdown! Complete recovery step
            stanceFeet_[steppingLeg_] = stepTargetPos_;
            inStance_[steppingLeg_] = true;
            recoveryStepping_ = false;
            steppingLeg_ = -1;
            // Absorb horizontal velocity on touchdown
            comVel_.x *= 0.3f;
            comVel_.z *= 0.3f;
        }
    }

    // --- 3. Body-Posture PID Stabilizer -------------------------------------
    const float rollErr  = -roll_;
    const float pitchErr = -pitch_;

    // Target yaw tracking with angle wrapping to [-PI, +PI]
    float yawErr = targetYaw_ - yaw_;
    while (yawErr > math::PI) yawErr -= 2.0f * math::PI;
    while (yawErr < -math::PI) yawErr += 2.0f * math::PI;

    const float heightErr = -heightDelta_;

    intRollErr_  = math::clampf(intRollErr_  + rollErr  * dt, -0.2f, 0.2f);
    intPitchErr_ = math::clampf(intPitchErr_ + pitchErr * dt, -0.2f, 0.2f);

    const float desTorqueRoll  = kpRollPitch * rollErr  + kiRollPitch * intRollErr_  - kdRollPitch * rollVel_;
    const float desTorquePitch = kpRollPitch * pitchErr + kiRollPitch * intPitchErr_ - kdRollPitch * pitchVel_;
    const float desTorqueYaw   = kpYaw       * yawErr                                - kdYaw       * yawVel_;
    const float desForceHeight = kpHeight    * heightErr                             - kdHeight    * heightVel_;

    // Desired horizontal deceleration
    const float desForceX = -trunkMass * 2.5f * comVel_.x;
    const float desForceZ = -trunkMass * 2.5f * comVel_.z;

    // --- 4. Ground-Reaction Force (GRF) Distribution via QP -----------------
    QpForceAllocator::Problem qpProb;
    qpProb.desiredForce = math::Vec3{desForceX, desForceHeight, desForceZ};
    qpProb.desiredTorque = math::Vec3{desTorqueRoll, desTorqueYaw, desTorquePitch};
    qpProb.inStance = inStance_;
    qpProb.totalMass = trunkMass;
    qpProb.gravity = gravity;
    qpProb.frictionCoeff = frictionMu;

    // Positions of feet relative to trunk CoM:
    for (int leg = 0; leg < kLegs; ++leg) {
        const Vec3 hipLocal{kLegX[leg] * hipX, 0.0f, kLegZ[leg] * hipZ};
        const Vec3 hipRot = tiltQ.rotate(hipLocal);
        qpProb.footPositions[leg] = math::Vec3{
            hipRot.x,
            -(curTrunkHeight + hipRot.y),
            hipRot.z
        };
    }

    const QpForceAllocator::Result qpRes = QpForceAllocator::solve(qpProb);
    qpForces_ = qpRes.forces;

    // --- 5. Integrate Trunk Dynamics under Gravity, QP Forces, & Pushes ----
    float totalGRF_x = 0.0f;
    float totalGRF_y = 0.0f;
    float totalGRF_z = 0.0f;
    float totalTorque_roll = 0.0f;
    float totalTorque_pitch = 0.0f;
    float totalTorque_yaw = 0.0f;

    for (int leg = 0; leg < kLegs; ++leg) {
        if (!inStance_[leg]) continue;
        const Vec3& f = qpForces_[leg];
        const Vec3& r = qpProb.footPositions[leg];

        totalGRF_x += f.x;
        totalGRF_y += f.y;
        totalGRF_z += f.z;

        // r x f
        totalTorque_roll  += (r.y * f.z - r.z * f.y);
        totalTorque_yaw   += (r.z * f.x - r.x * f.z);
        totalTorque_pitch += (r.x * f.y - r.y * f.x);
    }

    // Inverted-pendulum gravity torque:
    const float gravTorquePitch = trunkMass * gravity * zc * std::sin(pitch_);
    const float gravTorqueRoll  = trunkMass * gravity * zc * std::sin(roll_);

    // Net accelerations
    const float accelX = (totalGRF_x + accumForce_.x) / trunkMass;
    const float accelY = (totalGRF_y - trunkMass * gravity + accumForce_.y) / trunkMass;
    const float accelZ = (totalGRF_z + accumForce_.z) / trunkMass;

    const float accelPitch = (gravTorquePitch + totalTorque_pitch + accumTorque_.z) / pitchInertia;
    const float accelRoll  = (gravTorqueRoll  + totalTorque_roll  + accumTorque_.x) / rollInertia;
    const float accelYaw   = (totalTorque_yaw + accumTorque_.y) / yawInertia;

    // Integrate CoM linear velocity & position
    comVel_.x  += accelX * dt;
    comVel_.z  += accelZ * dt;
    heightVel_ += accelY * dt;

    comPos_.x   += comVel_.x * dt;
    comPos_.z   += comVel_.z * dt;
    heightDelta_ += heightVel_ * dt;
    heightDelta_  = math::clampf(heightDelta_, -0.15f, 0.08f);

    // Integrate orientation & angular velocity
    pitchVel_ += accelPitch * dt;
    rollVel_  += accelRoll  * dt;
    yawVel_   += accelYaw   * dt;

    pitch_ += pitchVel_ * dt;
    roll_  += rollVel_  * dt;
    yaw_   += yawVel_   * dt;

    pitch_ = math::clampf(pitch_, -0.65f, 0.65f);
    roll_  = math::clampf(roll_,  -0.65f, 0.65f);

    // Leg odometry damping (stance feet anchor horizontal CoM motion)
    comVel_.x *= (1.0f - 1.2f * dt);
    comVel_.z *= (1.0f - 1.2f * dt);
    pitchVel_ *= (1.0f - 0.2f * dt);
    rollVel_  *= (1.0f - 0.2f * dt);
    yawVel_   *= (1.0f - 0.5f * dt);
    heightVel_*= (1.0f - 0.4f * dt);

    accumForce_  = {0.0f, 0.0f, 0.0f};
    accumTorque_ = {0.0f, 0.0f, 0.0f};

    // --- 6. Contact Jacobians & Inverse Kinematics ---------------------------
    JointArray offsets{};

    for (int leg = 0; leg < kLegs; ++leg) {
        float desHip = 0.8f;
        float desKnee = -1.6f;

        if (recoveryStepping_ && leg == steppingLeg_) {
            // Swing leg: track swing trajectory in hip frame
            solve2LinkIK(swingFootLocal.x, swingFootLocal.y, thighLen, calfLen, desHip, desKnee);
            offsets[2 * leg]     = desHip  - 0.8f;
            offsets[2 * leg + 1] = desKnee - (-1.6f);

            // Swing leg joint PD torque
            jointTorques_[2 * leg]     = kpJoint * (desHip - (0.8f + offsets[2 * leg]));
            jointTorques_[2 * leg + 1] = kpJoint * (desKnee - (-1.6f + offsets[2 * leg + 1]));
        } else {
            // Stance leg: compute hip height to ground
            const Vec3 hipLocal{kLegX[leg] * hipX, 0.0f, kLegZ[leg] * hipZ};
            const Vec3 hipRot = tiltQ.rotate(hipLocal);
            const float hipY = curTrunkHeight + hipRot.y;

            const float d = math::clampf(hipY, 0.08f, (thighLen + calfLen) - 0.005f);
            const float psi = std::acos(math::clampf(d / (thighLen + calfLen), 0.0f, 1.0f));
            const float dPsi = psi - 0.80f;

            // Hip pitch compensation keeps leg vertical in world frame
            float hipOff  = dPsi + pitch_;
            float kneeOff = -2.0f * dPsi;

            hipOff  = math::clampf(hipOff,  -maxHipOffset,  maxHipOffset);
            kneeOff = math::clampf(kneeOff, -maxKneeOffset, maxKneeOffset);

            offsets[2 * leg]     = hipOff;
            offsets[2 * leg + 1] = kneeOff;

            // Jacobian-Transpose Torque mapping: tau = J^T * f
            const float curHip = 0.8f + hipOff;
            const float curKnee = -1.6f + kneeOff;
            float J[2][2]{};
            computeLegJacobian(curHip, curKnee, thighLen, calfLen, J);

            const Vec3& f = qpForces_[leg];
            // In sagittal plane: f.x is forward force, f.y is vertical normal force
            const float tauHip  = J[0][0] * f.x + J[1][0] * f.y;
            const float tauKnee = J[0][1] * f.x + J[1][1] * f.y;

            jointTorques_[2 * leg]     = tauHip;
            jointTorques_[2 * leg + 1] = tauKnee;
        }
    }

    jointOffsets_ = offsets;
    return offsets;
}


} // namespace robot
