#include "robot/quadruped.hpp"

#include <algorithm>
#include <cmath>

namespace robot {

namespace {

using math::Mat4;
using math::Quat;
using math::Vec3;

// ---- geometry (metres, unscaled) ------------------------------------------------------
constexpr float kHipX = 0.1881f;        // hip joint distance from trunk centre, along X
constexpr float kHipZ = 0.1265f;        // hip joint distance from trunk centre, sideways
constexpr float kThighLen = 0.213f;
constexpr float kCalfLen = 0.213f;
constexpr float kFootRadius = 0.02f;

// ---- joints / gait --------------------------------------------------------------------
constexpr float kHipStand = 0.8f;
constexpr float kKneeStand = -1.6f;
constexpr float kHipAmp = 0.25f;
constexpr float kKneeLift = 0.5f;
constexpr float kGaitFreq = 1.5f;        // Hz
constexpr float kBlendRate = 3.0f;       // 1/s, stand <-> trot transition
constexpr float kStanceTolerance = 0.006f; // foot counts as "on the ground" within this height
constexpr float kWorldLimit = 25.0f;     // robot wraps around at +-this X so it stays on the floor
constexpr float kLegPhase[4] = {0.0f, math::PI, math::PI, 0.0f}; // trot: FL+RR vs FR+RL

// ---- colours ----------------------------------------------------------------------------
const Vec3 kDark{0.16f, 0.17f, 0.19f};
const Vec3 kWhite{0.86f, 0.86f, 0.88f};
const Vec3 kMetal{0.55f, 0.57f, 0.60f};
const Vec3 kRubber{0.05f, 0.05f, 0.05f};

constexpr bool isLeftLeg(int leg) { return leg == 0 || leg == 2; }
constexpr bool isFrontLeg(int leg) { return leg == 0 || leg == 1; }

// +Z is the robot's right side, so left legs sit at negative Z.
Vec3 hipPosition(int leg) {
    return {isFrontLeg(leg) ? kHipX : -kHipX, 0.0f, isLeftLeg(leg) ? -kHipZ : kHipZ};
}

// Pitch rotation about the lateral (Z) axis. The MJCF angle convention has "positive = foot
// swings backward", which is a rotation by -angle about +Z for a robot walking along +X.
Mat4 pitch(float angle) { return Mat4::rotation(Quat::fromAxisAngle({0.0f, 0.0f, 1.0f}, -angle)); }

Vec3 transformPoint(const Mat4& m, const Vec3& p) {
    return {m.m[0] * p.x + m.m[4] * p.y + m.m[8] * p.z + m.m[12],
            m.m[1] * p.x + m.m[5] * p.y + m.m[9] * p.z + m.m[13],
            m.m[2] * p.x + m.m[6] * p.y + m.m[10] * p.z + m.m[14]};
}

// Box centred at `center` (in the frame `parent`) with full size `size`.
DrawPart box(const Mat4& parent, const Vec3& center, const Vec3& size, const Vec3& color) {
    return {parent * Mat4::translation(center) * Mat4::scale(size), color, false};
}

DrawPart sphere(const Mat4& parent, const Vec3& center, float radius, const Vec3& color) {
    return {parent * Mat4::translation(center) * Mat4::scale({radius, radius, radius}), color, true};
}

} // namespace

Quadruped::JointArray Quadruped::standPose() {
    JointArray j{};
    for (int leg = 0; leg < kLegs; ++leg) {
        j[2 * leg] = kHipStand;
        j[2 * leg + 1] = kKneeStand;
    }
    return j;
}

void Quadruped::setPosition(const Vec3& groundPoint) {
    position_.x = groundPoint.x;
    position_.z = groundPoint.z;
    havePrev_ = false;
    solvePose();
    position_.y = trunkHeight_ * scale;
}

Quadruped::LegFrames Quadruped::legFrames(int leg, float hipAngle, float kneeAngle) const {
    LegFrames f;
    f.hip = Mat4::translation(hipPosition(leg)) * pitch(hipAngle);
    f.knee = f.hip * Mat4::translation({0.0f, -kThighLen, 0.0f}) * pitch(kneeAngle);
    return f;
}

Vec3 Quadruped::footInTrunk(int leg) const {
    const LegFrames f = legFrames(leg, joints_[2 * leg], joints_[2 * leg + 1]);
    return transformPoint(f.knee, {0.0f, -kCalfLen, 0.0f});
}

Vec3 Quadruped::footWorldPosition(int leg) const {
    const float y = stabilizer_.yaw();
    const Quat rotQ = Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, -y) *
                      Quat::fromAxisAngle({0.0f, 0.0f, 1.0f}, visualPitch_) *
                      Quat::fromAxisAngle({1.0f, 0.0f, 0.0f}, visualRoll_);
    return position_ + rotQ.rotate(footInTrunk(leg)) * scale;
}

float Quadruped::footGroundHeight(int leg) const {
    const Vec3 fw = footWorldPosition(leg);
    return std::max(0.0f, fw.y);
}

// Computes the trunk height, suspension dynamics, and advances the robot
// using exact zero-slip stance foot ground anchoring.
void Quadruped::solvePose() {
    const float yaw = stabilizer_.yaw();
    const Quat rotQ = Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, -yaw) *
                      Quat::fromAxisAngle({0.0f, 0.0f, 1.0f}, stabilizer_.pitch()) *
                      Quat::fromAxisAngle({1.0f, 0.0f, 0.0f}, stabilizer_.roll());

    std::array<Vec3, kLegs> footWorldRel;
    float lowestY = 1e9f;
    std::array<bool, kLegs> inStance;

    for (int leg = 0; leg < kLegs; ++leg) {
        const Vec3 fLocal = footInTrunk(leg);
        const Vec3 fWorld = rotQ.rotate(fLocal);
        footWorldRel[leg] = fWorld;
        lowestY = std::min(lowestY, fWorld.y);

        // A leg is in stance when knee is not lifted (cos(phi) >= -0.15)
        const float phi = phase_ + kLegPhase[leg];
        inStance[leg] = (std::cos(phi) >= -0.15f);
    }

    // Dynamic trotting heave bounce (suspension compliance):
    // 2 bounces per trot cycle (one bounce per diagonal strike: FL+RR, then FR+RL)
    const float speedScale = math::clampf(std::abs(cmdFwdSpeed_) / 0.8f, 0.0f, 1.0f);
    const float heaveBounce = -0.014f * std::abs(std::cos(phase_)) * speedScale;
    // Dynamic pitch rock: forward-leaning thrust with diagonal push
    const float pitchBounce = 0.024f * std::sin(2.0f * phase_) * speedScale;
    // Inward banking during turns: lean into the turn
    const float bankRoll = -0.055f * (cmdYawRate_ / 2.0f) * speedScale;
    // Dynamic lateral sway: natural body sway toward the stance diagonal
    const float trotRollSway = 0.018f * std::sin(phase_) * speedScale;

    visualPitch_ = stabilizer_.pitch() + pitchBounce;
    visualRoll_  = stabilizer_.roll()  + bankRoll + trotRollSway;

    trunkHeight_ = (kFootRadius - lowestY) + heaveBounce;

    const bool isWalking = (controlMode_ == ControlMode::RLNeuralNetwork) || (blend_ > 0.05f);
    if (havePrev_ && isWalking) {
        Vec3 sumDelta{0.0f, 0.0f, 0.0f};
        int stanceCount = 0;
        float leftSum = 0.0f;  int leftCount = 0;
        float rightSum = 0.0f; int rightCount = 0;

        for (int leg = 0; leg < kLegs; ++leg) {
            // If foot was in stance both frames, it is firmly planted on ground
            if (inStance[leg] && prevInStance_[leg]) {
                // Ground anchor: trunk moves opposite to planted foot's displacement
                const Vec3 delta = -(footWorldRel[leg] - prevFootWorld_[leg]);
                sumDelta.x += delta.x;
                sumDelta.z += delta.z;
                ++stanceCount;

                // Local forward displacement for differential yaw
                const float forwardDisp = -(footInTrunk(leg).x - prevFoot_[leg].x);
                if (isLeftLeg(leg)) {
                    leftSum += forwardDisp;
                    ++leftCount;
                } else {
                    rightSum += forwardDisp;
                    ++rightCount;
                }
            }
        }

        if (stanceCount > 0) {
            // Zero-slip physical translation
            position_.x += (sumDelta.x / static_cast<float>(stanceCount)) * scale;
            position_.z += (sumDelta.z / static_cast<float>(stanceCount)) * scale;

            // Physical yaw rotation produced by differential leg strides
            if (leftCount > 0 && rightCount > 0) {
                const float dLeft = leftSum / static_cast<float>(leftCount);
                const float dRight = rightSum / static_cast<float>(rightCount);
                const float dYaw = ((dLeft - dRight) / (2.0f * stabilizer_.hipZ)) * 0.70f;
                stabilizer_.addYaw(dYaw);
            }
        }
    }

    for (int leg = 0; leg < kLegs; ++leg) {
        prevFoot_[leg] = footInTrunk(leg);
        prevFootWorld_[leg] = footWorldRel[leg];
        prevInStance_[leg] = inStance[leg];

        // Track foot contact events for impact effects
        const bool grounded = inStance[leg];
        footLanded_[leg] = (grounded && !prevFootGrounded_[leg]); // rising edge: just landed
        prevFootGrounded_[leg] = grounded;

        if (footLanded_[leg]) {
            footContactTimer_[leg] = 0.0f; // reset on new contact
        } else {
            footContactTimer_[leg] += 0.008333f; // ~120Hz step
        }
    }
    havePrev_ = true;

    // Head bob: natural oscillation tied to trot gait
    const float headBobFreq = 2.0f; // 2x gait frequency (one bob per diagonal strike)
    headBob_ = -0.008f * std::sin(headBobFreq * phase_) * speedScale;

    // Acceleration jerk: derivative of forward speed command
    accelJerk_ = (cmdFwdSpeed_ - prevCmdFwdSpeed_) * 120.0f; // scale to per-second
    prevCmdFwdSpeed_ = cmdFwdSpeed_;
}

void Quadruped::push(float pitchImpulse, float rollImpulse) {
    stabilizer_.push(pitchImpulse, rollImpulse);
}

void Quadruped::applyImpulse(const math::Vec3& impulse, const math::Vec3& r) {
    stabilizer_.applyImpulse(impulse, r);
}

void Quadruped::setTargetCoordinate(const math::Vec3& target) {
    constexpr float kArenaBound = 8.8f;
    targetCoordinate_.x = math::clampf(target.x, -kArenaBound, kArenaBound);
    targetCoordinate_.y = target.y;
    targetCoordinate_.z = math::clampf(target.z, -kArenaBound, kArenaBound);
    targetMode_ = TargetMode::FixedWaypoint;
    targetsReached_ = 0;
    const float dx = targetCoordinate_.x - position_.x;
    const float dz = targetCoordinate_.z - position_.z;
    chosenAngle_ = std::atan2(dz, dx);
    if (controlMode_ == ControlMode::ClassicalStabilizer) {
        stabilizer_.setTargetYaw(chosenAngle_);
    }
}

void Quadruped::setTargetDirectionAngle(float angleRad, float distance) {
    chosenAngle_ = angleRad;
    targetMode_ = TargetMode::DirectionalSteering;
    targetsReached_ = 0;
    constexpr float kArenaBound = 8.5f;
    const float distClamped = std::min(distance, 7.0f);
    targetCoordinate_.x = math::clampf(position_.x + distClamped * std::cos(chosenAngle_), -kArenaBound, kArenaBound);
    targetCoordinate_.z = math::clampf(position_.z + distClamped * std::sin(chosenAngle_), -kArenaBound, kArenaBound);
    if (controlMode_ == ControlMode::ClassicalStabilizer) {
        stabilizer_.setTargetYaw(chosenAngle_);
    }
}

void Quadruped::adjustTargetAngle(float deltaAngleRad) {
    chosenAngle_ += deltaAngleRad;
    targetMode_ = TargetMode::DirectionalSteering;
    constexpr float kDist = 6.0f;
    constexpr float kArenaBound = 8.5f;
    targetCoordinate_.x = math::clampf(position_.x + kDist * std::cos(chosenAngle_), -kArenaBound, kArenaBound);
    targetCoordinate_.z = math::clampf(position_.z + kDist * std::sin(chosenAngle_), -kArenaBound, kArenaBound);
    if (controlMode_ == ControlMode::ClassicalStabilizer) {
        stabilizer_.setTargetYaw(chosenAngle_);
    }
}

void Quadruped::setDirectionByIndex(int index) {
    constexpr float kPi = math::PI;
    constexpr float kDist = 10.0f;
    switch (index) {
        case 8: setTargetDirectionAngle(0.0f, kDist); break;                // North (+X)
        case 9: setTargetDirectionAngle(0.25f * kPi, kDist); break;         // North-East (+X, +Z)
        case 6: setTargetDirectionAngle(0.50f * kPi, kDist); break;         // East (+Z)
        case 3: setTargetDirectionAngle(0.75f * kPi, kDist); break;         // South-East (-X, +Z)
        case 2: setTargetDirectionAngle(kPi, kDist); break;                 // South (-X)
        case 1: setTargetDirectionAngle(-0.75f * kPi, kDist); break;        // South-West (-X, -Z)
        case 4: setTargetDirectionAngle(-0.50f * kPi, kDist); break;        // West (-Z)
        case 7: setTargetDirectionAngle(-0.25f * kPi, kDist); break;        // North-West (+X, -Z)
        default: setTargetDirectionAngle(0.0f, kDist); break;
    }
}

const char* Quadruped::targetDirectionName() const {
    float deg = chosenAngle_ * (180.0f / math::PI);
    while (deg < -180.0f) deg += 360.0f;
    while (deg > 180.0f) deg -= 360.0f;

    if (deg >= -22.5f && deg < 22.5f) return "North [Forward +X]";
    if (deg >= 22.5f && deg < 67.5f) return "North-East [+X, +Z]";
    if (deg >= 67.5f && deg < 112.5f) return "East [Right +Z]";
    if (deg >= 112.5f && deg < 157.5f) return "South-East [-X, +Z]";
    if (deg >= 157.5f || deg < -157.5f) return "South [Backward -X]";
    if (deg >= -157.5f && deg < -112.5f) return "South-West [-X, -Z]";
    if (deg >= -112.5f && deg < -67.5f) return "West [Left -Z]";
    if (deg >= -67.5f && deg < -22.5f) return "North-West [+X, -Z]";
    return "Custom Direction";
}

float Quadruped::distanceToTarget() const {
    const float dx = targetCoordinate_.x - position_.x;
    const float dz = targetCoordinate_.z - position_.z;
    return std::sqrt(dx * dx + dz * dz);
}

void Quadruped::update(float dt) {
    prevJoints_ = joints_;
    totalStepCount_++;

    if (controlMode_ == ControlMode::RLNeuralNetwork) {
        // --- 1. Construct 32-dimensional observation vector -----------------
        float obs[PolicyNN::kInputDim]{};

        // Stationary target vector in world space
        const float dx_w = targetCoordinate_.x - position_.x;
        const float dz_w = targetCoordinate_.z - position_.z;
        const float dist = std::max(1e-4f, std::sqrt(dx_w * dx_w + dz_w * dz_w));

        // Heading error: angle from robot facing direction to target
        const float targetAngle = std::atan2(dz_w, dx_w);
        const float yaw = stabilizer_.yaw();
        float headingErr = targetAngle - yaw;
        // Wrap to [-PI, PI]
        while (headingErr > math::PI) headingErr -= 2.0f * math::PI;
        while (headingErr < -math::PI) headingErr += 2.0f * math::PI;

        // Features 0, 1: Heading error (sin, cos) - tells NN which way to turn
        obs[0] = std::sin(headingErr);
        obs[1] = std::cos(headingErr);

        // Feature 2: Normalized distance to target
        obs[2] = std::tanh(dist / (4.0f * scale));

        const Quat tiltQ = Quat::fromAxisAngle({0.0f, 0.0f, 1.0f}, stabilizer_.pitch()) *
                           Quat::fromAxisAngle({1.0f, 0.0f, 0.0f}, stabilizer_.roll());

        // Features 3, 4, 5: Projected gravity vector in body frame
        const Vec3 gravWorld{0.0f, -1.0f, 0.0f};
        const Vec3 gravBody = tiltQ.conjugate().rotate(gravWorld);
        obs[3] = gravBody.x;
        obs[4] = gravBody.y;
        obs[5] = gravBody.z;

        // Features 6, 7, 8: Angular velocities (rad/s)
        obs[6] = stabilizer_.rollVel();
        obs[7] = stabilizer_.yawVel();
        obs[8] = stabilizer_.pitchVel();

        // Features 9, 10, 11: Linear velocities in body frame (m/s)
        const float cos_y = std::cos(yaw);
        const float sin_y = std::sin(yaw);
        const Vec3 comV = stabilizer_.comVelocity();
        obs[9]  =  comV.x * cos_y + comV.z * sin_y;
        obs[10] =  0.0f;
        obs[11] = -comV.x * sin_y + comV.z * cos_y;

        // Features 12..19: Joint position errors from nominal standing pose
        for (int j = 0; j < kJoints; ++j) {
            const float nom = (j % 2 == 0) ? kHipStand : kKneeStand;
            obs[12 + j] = joints_[j] - nom;
        }

        // Features 20..27: Joint velocities (rad/s)
        for (int j = 0; j < kJoints; ++j) {
            obs[20 + j] = (joints_[j] - prevJoints_[j]) / std::max(dt, 1e-4f);
        }

        // Features 28, 29: Cyclic gait clock
        obs[28] = std::sin(phase_);
        obs[29] = std::cos(phase_);

        // Features 30, 31: Current command feedback (normalized)
        constexpr float kMaxYawRate = 2.0f;
        constexpr float kMaxFwdSpeed = 0.8f;
        obs[30] = cmdYawRate_ / kMaxYawRate;
        obs[31] = cmdFwdSpeed_ / kMaxFwdSpeed;

        // --- 2. Forward pass through Neural Network (32 -> 64 -> 10)
        float actions[PolicyNN::kOutputDim]{};
        policyNN_.forward(obs, actions);

        // --- 3. Extract NN steering commands (actions [8] and [9]) ---
        const float nnYawRateCmd = actions[8] * kMaxYawRate;     // [-2, +2] rad/s
        const float nnFwdSpeedCmd = actions[9] * kMaxFwdSpeed;   // [-0.8, +0.8] m/s

        // Smooth command tracking (low-pass filter)
        constexpr float kAlphaYaw = 0.15f;
        constexpr float kAlphaSpd = 0.15f;
        cmdYawRate_ += kAlphaYaw * (nnYawRateCmd - cmdYawRate_);
        cmdFwdSpeed_ += kAlphaSpd * (nnFwdSpeedCmd - cmdFwdSpeed_);

        // Apply NN yaw rate to robot heading (assisted by physical differential stride)
        stabilizer_.addYaw(cmdYawRate_ * 0.4f * dt);
        stabilizer_.setTargetYaw(stabilizer_.yaw()); // track actual yaw

        // Waypoint arrival detection
        const bool arrivedAtWaypoint = (dist < 0.65f * scale);
        if (arrivedAtWaypoint && targetsReached_ == 0) {
            targetsReached_ = 1;
            std::printf(">>> [GOAL REACHED #1] Step: %d | Arrived at (%.2f, %.2f) (Dist: %.2fm)! <<<\n",
                        totalStepCount_, targetCoordinate_.x, targetCoordinate_.z, dist);
            if (targetMode_ != TargetMode::FixedWaypoint) {
                constexpr float kDistAhead = 6.0f;
                constexpr float kArenaBound = 8.5f;
                float nextX = position_.x + kDistAhead * std::cos(chosenAngle_);
                float nextZ = position_.z + kDistAhead * std::sin(chosenAngle_);
                if (std::abs(nextX) > kArenaBound || std::abs(nextZ) > kArenaBound) {
                    chosenAngle_ += 2.2f; // turn inward toward arena center
                    nextX = math::clampf(position_.x + kDistAhead * std::cos(chosenAngle_), -kArenaBound, kArenaBound);
                    nextZ = math::clampf(position_.z + kDistAhead * std::sin(chosenAngle_), -kArenaBound, kArenaBound);
                }
                targetCoordinate_.x = nextX;
                targetCoordinate_.z = nextZ;
                targetsReached_ = 0; // ready for next
                std::printf(">>> Next target: (%.1f, %.1f) | Dist: %.1fm <<<\n",
                            targetCoordinate_.x, targetCoordinate_.z, distanceToTarget());
            } else {
                std::printf(">>> Holding at target destination: (%.2f, %.2f) <<<\n",
                            targetCoordinate_.x, targetCoordinate_.z);
            }
        }

        // --- 4. Trotting CPG modulated by NN forward speed command ---
        const bool holdingAtTarget = (targetMode_ == TargetMode::FixedWaypoint && dist < 0.40f * scale);
        const float effFwdSpeed = holdingAtTarget ? 0.0f : cmdFwdSpeed_;

        if (totalStepCount_ % 120 == 0) {
            std::printf("[RL NN] Target: (%.1f, %.1f) | Dist: %.2fm | HdgErr: %+.1f deg | NN YawRate: %+.2f rad/s | NN Speed: %.2f m/s\n",
                        targetCoordinate_.x, targetCoordinate_.z, dist,
                        headingErr * (180.0f / math::PI), cmdYawRate_, effFwdSpeed);
        }

        // Advance gait clock - scaled by NN forward speed (slower trot when turning/stopping)
        const float speedRatio = math::clampf(std::abs(effFwdSpeed) / kMaxFwdSpeed, 0.15f, 1.0f);
        phase_ = std::fmod(phase_ + 2.0f * math::PI * kGaitFreq * speedRatio * dt, 2.0f * math::PI);

        // Scale leg step amplitude with NN speed command (bigger strides when moving fast)
        const float speedScale = math::clampf(std::abs(effFwdSpeed) / kMaxFwdSpeed, 0.10f, 1.0f);
        const float hipAmp = kHipAmp * speedScale;
        const float kneeLift = kKneeLift * speedScale;

        // Differential stride: outer legs take bigger steps than inner legs to turn
        const float turnBias = math::clampf((cmdYawRate_ / kMaxYawRate) * 0.40f, -0.40f, 0.40f);
        const float hipAmpLeft = hipAmp * (1.0f - turnBias);
        const float hipAmpRight = hipAmp * (1.0f + turnBias);

        // Neural network joint residuals (actions [0..7]) applied on top of trotting CPG
        constexpr float kActionScale = 0.25f;
        for (int leg = 0; leg < kLegs; ++leg) {
            const float phi = phase_ + kLegPhase[leg];
            const float curHipAmp = isLeftLeg(leg) ? hipAmpLeft : hipAmpRight;
            const float cosP = std::cos(phi);
            const float sinP = std::sin(phi);

            // C1 smooth cycloid swing clearance: zero touchdown and liftoff velocity
            const float swingLift = (cosP < 0.0f) ? (kneeLift * cosP * cosP) : 0.0f;
            const float hipBase = kHipStand + curHipAmp * sinP;
            const float kneeBase = kKneeStand - swingLift;

            const float qTargetHip = hipBase + kActionScale * actions[2 * leg];
            const float qTargetKnee = kneeBase + kActionScale * actions[2 * leg + 1];

            joints_[2 * leg]     += 18.0f * dt * (qTargetHip - joints_[2 * leg]);
            joints_[2 * leg + 1] += 18.0f * dt * (qTargetKnee - joints_[2 * leg + 1]);
        }

        // Step physics stabilizer to maintain ground contacts and gravity balance
        stabilizer_.update(dt);
    } else {
        // --- Classical LIPM / Stabilizer Mode --------------------------------
        const float target = walking_ ? 1.0f : 0.0f;
        if (blend_ < target) blend_ = std::min(target, blend_ + kBlendRate * dt);
        else if (blend_ > target) blend_ = std::max(target, blend_ - kBlendRate * dt);

        if (blend_ > 0.0f) phase_ = std::fmod(phase_ + 2.0f * math::PI * kGaitFreq * dt, 2.0f * math::PI);

        const JointArray stabOffsets = stabilizer_.update(dt);

        for (int leg = 0; leg < kLegs; ++leg) {
            const float phi = phase_ + kLegPhase[leg];
            const float hipGait = kHipStand + kHipAmp * std::sin(phi);
            const float kneeGait = kKneeStand - kKneeLift * std::max(0.0f, -std::cos(phi));
            joints_[2 * leg]     = kHipStand + blend_ * (hipGait - kHipStand) + stabOffsets[2 * leg];
            joints_[2 * leg + 1] = kKneeStand + blend_ * (kneeGait - kKneeStand) + stabOffsets[2 * leg + 1];
        }
    }

    solvePose();
    position_.y = trunkHeight_ * scale;
}

void Quadruped::collectParts(std::vector<DrawPart>& out) const {
    // Apply visual trunk orientation with dynamic pitch/roll and bank angle
    const Quat rotQ = Quat::fromAxisAngle({0.0f, 1.0f, 0.0f}, -stabilizer_.yaw()) *
                      Quat::fromAxisAngle({0.0f, 0.0f, 1.0f}, visualPitch_) *
                      Quat::fromAxisAngle({1.0f, 0.0f, 0.0f}, visualRoll_);
    const Mat4 root = Mat4::translation(position_) *
                      Mat4::rotation(rotQ) *
                      Mat4::scale({scale, scale, scale});

    // --- trunk: dark frame, light cover, sensor block, rear cover ---------------------
    out.push_back(box(root, {0.0f, 0.0f, 0.0f}, {0.38f, 0.08f, 0.094f}, kDark));
    out.push_back(box(root, {0.0f, 0.05f, 0.0f}, {0.36f, 0.05f, 0.10f}, kWhite));
    // Head/sensor block bobs subtly with gait
    out.push_back(box(root, {0.205f, 0.03f + headBob_, 0.0f}, {0.04f, 0.06f, 0.09f}, kDark));
    // Rear cover counter-bobs
    out.push_back(box(root, {-0.2f, 0.03f - headBob_ * 0.6f, 0.0f}, {0.03f, 0.06f, 0.08f}, kDark));

    for (int leg = 0; leg < kLegs; ++leg) {
        const Vec3 hip = hipPosition(leg);
        const float side = isLeftLeg(leg) ? -1.0f : 1.0f;

        // Bracket rigidly joining the trunk side wall to the hip motor (no gap).
        out.push_back(box(root, {hip.x, 0.0f, side * 0.077f}, {0.08f, 0.08f, 0.064f}, kDark));

        const LegFrames f = legFrames(leg, joints_[2 * leg], joints_[2 * leg + 1]);
        const Mat4 hipFrame = root * f.hip;
        const Mat4 kneeFrame = root * f.knee;

        // Thigh: hip motor + link.
        out.push_back(box(hipFrame, {0.0f, 0.0f, 0.0f}, {0.072f, 0.072f, 0.04f}, kMetal));
        out.push_back(box(hipFrame, {0.0f, -kThighLen * 0.5f, 0.0f}, {0.036f, kThighLen, 0.024f}, kDark));

        // Calf: knee motor + thin link + rubber foot.
        out.push_back(box(kneeFrame, {0.0f, 0.0f, 0.0f}, {0.052f, 0.052f, 0.032f}, kMetal));
        out.push_back(box(kneeFrame, {0.0f, -kCalfLen * 0.5f, 0.0f}, {0.018f, kCalfLen, 0.018f}, kDark));
        out.push_back(sphere(kneeFrame, {0.0f, -kCalfLen, 0.0f}, kFootRadius, kRubber));
    }
}

} // namespace robot
