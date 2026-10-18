#pragma once

#include "math.hpp"
#include "robot/qp_solver.hpp"

#include <array>

namespace robot {

// Complete classical, no-neural-network stabilizer for a quadruped robot.
//
// Implements:
//   1. Robot State Estimation (IMU orientation R_B^W, angular velocity omega_B, CoM linear velocity)
//   2. Body-posture PID Stabilizer (roll, pitch, yaw, height)
//   3. Linear Inverted Pendulum Model (LIPM: omega = sqrt(g / z_c), ddot_c = omega^2 * (c - p))
//   4. Capture Point (DCM: xi = c + dot_c / omega)
//   5. Support Polygon Inclusion Test (xi in P)
//   6. Recovery Foot Placement (Capture-point stepping + Raibert heuristic, workspace clipped)
//   7. 5th-Order Minimum-Jerk Swing Trajectory Generator with vertical sine clearance
//   8. Ground-Reaction Force Distribution via Quadratic Programming (QP) with friction cones
//   9. Analytical 2-Link Contact Jacobians & Jacobian-Transpose Torque Mapping (tau = J^T * f)
//  10. 2-Link Inverse Kinematics for exact foot placement
class Stabilizer {
public:
    static constexpr int kLegs = 4;
    static constexpr int kJoints = 8;
    using JointArray = std::array<float, kJoints>;

    // ---- Physical Parameters (unscaled SI units: kg, m, s, N, N*m) ----------

    float trunkMass     = 12.0f;       // Total mass m (kg)
    float pitchInertia  = 0.18f;       // I_yy / I_zz (kg*m^2)
    float rollInertia   = 0.09f;       // I_xx (kg*m^2)
    float yawInertia    = 0.22f;       // I_yaw (kg*m^2)
    float gravity       = 9.81f;       // g (m/s^2)
    float zc            = 0.28f;       // Constant CoM height z_c (m)
    float nominalHeight = 0.2968f;     // Nominal standing height: 2 * l * cos(0.8) (m)

    // Leg geometry (metres, unscaled)
    float hipX          = 0.1881f;     // Forward/backward hip half-span
    float hipZ          = 0.1265f;     // Lateral hip half-span
    float thighLen      = 0.213f;      // Link 1 length
    float calfLen       = 0.213f;      // Link 2 length
    float footRadius    = 0.02f;       // Foot sphere radius
    float maxReach      = 0.38f;       // Max leg reach for workspace clipping

    // LIPM natural frequency omega = sqrt(g / z_c)
    float omegaLIPM() const { return std::sqrt(gravity / zc); }

    // ---- Controller Gains (as recommended in specification) -----------------

    // Posture PID: Roll & Pitch
    float kpRollPitch   = 55.0f;       // 20 - 80 N*m/rad
    float kdRollPitch   = 5.5f;        // 2 - 10 N*m*s/rad
    float kiRollPitch   = 2.0f;

    // Posture PID: Yaw
    float kpYaw         = 20.0f;       // 10 - 40 N*m/rad
    float kdYaw         = 2.5f;        // 1 - 5 N*m*s/rad

    // Posture PID: Height
    float kpHeight      = 120.0f;      // 50 - 200 N/m
    float kdHeight      = 15.0f;       // 5 - 20 N*s/m

    // Capture-Point Stepping
    float kCp           = 0.85f;       // 0.5 - 1.5 capture-point gain
    float stepDuration  = 0.20f;       // 0.15 - 0.30 s
    float stepHeight    = 0.035f;      // 0.02 - 0.05 m clearance
    float frictionMu    = 0.80f;       // 0.6 - 1.0 friction coefficient

    // Joint servo PD
    float kpJoint       = 80.0f;
    float kdJoint       = 4.5f;

    // Joint offset limits
    float maxHipOffset  = 0.50f;       // rad
    float maxKneeOffset = 0.70f;       // rad

    // ---- Public Interface --------------------------------------------------

    /// Direct angular-velocity push impulse (rad/s)
    void push(float pitchImpulse, float rollImpulse);

    /// Linear impulse `J` (N*s) applied at trunk-local point `r` (metres)
    void applyImpulse(const math::Vec3& impulse, const math::Vec3& r);

    /// Advance full control loop by `dt` and return corrective joint offsets
    JointArray update(float dt);

    /// Reset internal states to standing equilibrium
    void reset();

    // ---- Telemetry & State Accessors ----------------------------------------

    // Orientation (rad)
    float pitch()       const { return pitch_; }
    float roll()        const { return roll_; }
    float yaw()         const { return yaw_; }
    float pitchVel()    const { return pitchVel_; }
    float rollVel()     const { return rollVel_; }
    float yawVel()      const { return yawVel_; }
    float heightDelta() const { return heightDelta_; }
    float trunkHeight() const { return nominalHeight + heightDelta_; }

    void setYaw(float y) { yaw_ = y; }
    void addYaw(float dy) { yaw_ += dy; }
    void setTargetYaw(float ty) { targetYaw_ = ty; }
    float targetYaw() const { return targetYaw_; }

    // CoM horizontal position and velocity
    math::Vec3 comPosition() const { return comPos_; }
    math::Vec3 comVelocity() const { return comVel_; }

    // Capture Point xi = c + dot_c / omega (horizontal coordinates X, Z)
    math::Vec3 capturePoint() const;

    // Support polygon check
    bool isCapturePointInPolygon() const { return cpInPolygon_; }
    bool isRecoveryStepping()      const { return recoveryStepping_; }
    int  recoveryLeg()             const { return steppingLeg_; }
    float recoveryProgress()       const { return stepTimer_ / stepDuration; }

    // Ground reaction forces from QP (3D vectors per leg in Newtons)
    const std::array<math::Vec3, kLegs>& legForces3D() const { return qpForces_; }
    // Normal forces (Y component)
    std::array<float, kLegs> legForces() const {
        return {qpForces_[0].y, qpForces_[1].y, qpForces_[2].y, qpForces_[3].y};
    }
    // Joint torques (N*m) [FL_hip, FL_knee, FR_hip, FR_knee, ...]
    const JointArray& jointTorques() const { return jointTorques_; }

    // Analytical Leg Jacobian J(q) mapping joint velocities to foot velocity
    static void computeLegJacobian(float qHip, float qKnee, float thigh, float calf, float J[2][2]);

    // 2-Link Inverse Kinematics: target foot (x, y) relative to hip -> (qHip, qKnee)
    static bool solve2LinkIK(float x, float y, float thigh, float calf, float& outHip, float& outKnee);

private:
    // Core state variables:
    float pitch_       = 0.0f;
    float roll_        = 0.0f;
    float yaw_         = 0.0f;
    float pitchVel_    = 0.0f;
    float rollVel_     = 0.0f;
    float yawVel_      = 0.0f;
    float targetYaw_   = 0.0f;
    float heightDelta_ = 0.0f;
    float heightVel_   = 0.0f;

    // CoM motion (horizontal plane: X forward, Z lateral)
    math::Vec3 comPos_{0.0f, 0.0f, 0.0f};
    math::Vec3 comVel_{0.0f, 0.0f, 0.0f};

    // Posture error integrals
    float intRollErr_  = 0.0f;
    float intPitchErr_ = 0.0f;

    // External push accumulators
    math::Vec3 accumForce_{0.0f, 0.0f, 0.0f};
    math::Vec3 accumTorque_{0.0f, 0.0f, 0.0f};

    // Stance state
    std::array<bool, kLegs> inStance_{true, true, true, true};
    std::array<math::Vec3, kLegs> stanceFeet_{}; // World-relative contact positions

    // Capture point & recovery stepping
    bool cpInPolygon_       = true;
    bool recoveryStepping_  = false;
    int  steppingLeg_       = -1;
    float stepTimer_        = 0.0f;
    math::Vec3 stepStartPos_{0.0f, 0.0f, 0.0f};
    math::Vec3 stepTargetPos_{0.0f, 0.0f, 0.0f};

    // Outputs from QP and Jacobian torque conversion
    std::array<math::Vec3, kLegs> qpForces_{};
    JointArray jointTorques_{};
    JointArray jointOffsets_{};

    // Helper routines
    bool testSupportPolygon(const math::Vec3& xi) const;
    void planRecoveryStep(const math::Vec3& xi);
    math::Vec3 evaluateSwingTrajectory(float tNorm) const;
};

} // namespace robot
