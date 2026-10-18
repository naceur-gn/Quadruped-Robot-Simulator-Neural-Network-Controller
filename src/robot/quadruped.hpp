#pragma once

#include "math.hpp"
#include "robot/stabilizer.hpp"
#include "robot/policy_nn.hpp"

#include <array>
#include <string>
#include <vector>

namespace robot {

// One box or sphere of the robot, ready to be drawn with the scene's unit meshes.
struct DrawPart {
    math::Mat4 model;   // full world transform (includes the shape size)
    math::Vec3 color;
    bool sphere = false;
};

// 8-joint quadruped (Go1-class proportions): per leg one hip-pitch and one knee-pitch joint.
//
// Frame conventions (same as the rest of the simulator): +Y is up, the robot walks along +X,
// +Z is the robot's right side.  Joint angles use the same convention as the MuJoCo model
// (quadruped_realistic.xml): hip = +0.8 / knee = -1.6 rad is the standing pose, a positive hip
// angle swings the foot backward, a negative knee angle bends the knee backward.
//
// The model is kinematic: it is posed with forward kinematics and kept on the ground by
// the feet; it is not (yet) driven by the rigid-body solver in physics/.
class Quadruped {
public:
    static constexpr int kLegs = 4;
    static constexpr int kJoints = 8;

    // Joint order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee.
    using JointArray = std::array<float, kJoints>;

    float scale = 3.0f;   // render scale: 1.0 = real size (~0.38 m trunk), 3.0 = easy to see

    void setPosition(const math::Vec3& groundPoint);   // x/z on the floor, y is computed
    const math::Vec3& position() const { return position_; }

    void setWalking(bool walking) { walking_ = walking; }
    void toggleWalking() { walking_ = !walking_; }
    bool walking() const { return walking_; }

    // Apply an impulsive push to the robot (pitch/roll angular velocities in rad/s).
    void push(float pitchImpulse, float rollImpulse);

    // Apply linear impulse J at trunk-local point r (metres).
    void applyImpulse(const math::Vec3& impulse, const math::Vec3& r);

    // Current virtual trunk tilt (from the stabilizer).
    float tiltPitch() const { return stabilizer_.pitch(); }
    float tiltRoll()  const { return stabilizer_.roll(); }

    Stabilizer& stabilizer() { return stabilizer_; }
    const Stabilizer& stabilizer() const { return stabilizer_; }

    std::array<float, kLegs> legForces() const { return stabilizer_.legForces(); }
    const JointArray& jointTorques() const { return stabilizer_.jointTorques(); }

    math::Vec3 capturePoint() const { return stabilizer_.capturePoint(); }
    bool isCapturePointInPolygon() const { return stabilizer_.isCapturePointInPolygon(); }
    bool isRecoveryStepping() const { return stabilizer_.isRecoveryStepping(); }
    int recoveryLeg() const { return stabilizer_.recoveryLeg(); }

    math::Vec3 footWorldPosition(int leg) const;
    float yaw() const { return stabilizer_.yaw(); }

    // Per-foot contact event info for impact effects
    float footContactTimer(int leg) const { return footContactTimer_[leg]; }
    bool  footJustLanded(int leg) const { return footLanded_[leg]; }
    float footGroundHeight(int leg) const;

    // Head/sensor block bobbing amplitude
    float headBob() const { return headBob_; }
    // Acceleration jerk visualization
    float accelerationJerk() const { return accelJerk_; }

    enum class ControlMode {
        ClassicalStabilizer,
        RLNeuralNetwork
    };

    void setControlMode(ControlMode mode) { controlMode_ = mode; }
    ControlMode controlMode() const { return controlMode_; }
    void toggleControlMode() {
        controlMode_ = (controlMode_ == ControlMode::ClassicalStabilizer)
                           ? ControlMode::RLNeuralNetwork
                           : ControlMode::ClassicalStabilizer;
    }
    bool isRLMode() const { return controlMode_ == ControlMode::RLNeuralNetwork; }

    bool loadPolicyWeights(const std::string& path) { return policyNN_.loadFromJson(path); }
    const PolicyNN& policy() const { return policyNN_; }

    enum class TargetMode {
        DirectionalSteering,
        FixedWaypoint,
        AutonomousRandom
    };

    void setTargetCoordinate(const math::Vec3& target);
    void setTargetDirectionAngle(float angleRad, float distance = 4.5f);
    void adjustTargetAngle(float deltaAngleRad);
    void setDirectionByIndex(int index); // 1..9 (Numpad style)
    float targetDirectionAngle() const { return chosenAngle_; }
    const char* targetDirectionName() const;
    TargetMode targetMode() const { return targetMode_; }

    const math::Vec3& targetCoordinate() const { return targetCoordinate_; }
    int targetsReached() const { return targetsReached_; }
    int totalStepCount() const { return totalStepCount_; }
    float distanceToTarget() const;

    void update(float dt);

    // Appends every box / sphere of the robot in its current pose.
    void collectParts(std::vector<DrawPart>& out) const;

    const JointArray& jointAngles() const { return joints_; }

    // Foot centre of leg 0..3 (FL, FR, RL, RR) in trunk coordinates, unscaled metres.
    math::Vec3 footInTrunk(int leg) const;

    static JointArray standPose();

private:
    struct LegFrames {
        math::Mat4 hip;     // trunk -> hip joint frame (rotated by hip angle)
        math::Mat4 knee;    // trunk -> knee joint frame (rotated by knee angle)
    };

    LegFrames legFrames(int leg, float hipAngle, float kneeAngle) const;
    void solvePose();       // joints_ -> contact height / forward motion

    math::Vec3 position_{0.0f, 0.0f, 0.0f};
    JointArray joints_ = standPose();
    JointArray prevJoints_ = standPose();

    bool walking_ = false;
    float blend_ = 0.0f;    // 0 = stand, 1 = full trot
    float phase_ = 0.0f;    // gait phase, radians

    std::array<math::Vec3, kLegs> prevFoot_{};
    std::array<math::Vec3, kLegs> prevFootWorld_{};
    std::array<bool, kLegs> prevInStance_{false, false, false, false};
    bool havePrev_ = false;
    Stabilizer stabilizer_;
    PolicyNN policyNN_;
    ControlMode controlMode_ = ControlMode::RLNeuralNetwork; // Default to RL Neural Network mode!
    float trunkHeight_ = 0.0f; // unscaled

    // Dynamic suspension & banking angles for realistic motion
    float visualPitch_ = 0.0f;
    float visualRoll_ = 0.0f;

    // Per-foot contact event timers (seconds since last touchdown)
    std::array<float, kLegs> footContactTimer_{0.5f, 0.5f, 0.5f, 0.5f};
    std::array<bool, kLegs> footLanded_{false, false, false, false};
    std::array<bool, kLegs> prevFootGrounded_{false, false, false, false};

    // Head bob and acceleration jerk for visual micro-animations
    float headBob_ = 0.0f;
    float accelJerk_ = 0.0f;
    float prevCmdFwdSpeed_ = 0.0f;

    math::Vec3 targetCoordinate_{5.0f, 0.0f, 0.0f}; // Target coordinate in world space
    TargetMode targetMode_ = TargetMode::DirectionalSteering;
    float chosenAngle_ = 0.0f; // 0 rad = +X (North / Forward)
    int targetsReached_ = 0;
    int totalStepCount_ = 0;

    // NN-controlled steering state (updated by policy actions [8] and [9])
    float cmdYawRate_ = 0.0f;   // Current yaw rate command (rad/s)
    float cmdFwdSpeed_ = 0.0f;  // Current forward speed command (m/s)
};

} // namespace robot
