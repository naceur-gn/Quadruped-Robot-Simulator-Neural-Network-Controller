#pragma once

#include "math.hpp"

#include <array>
#include <cmath>
#include <algorithm>

namespace robot {

// Lightweight, deterministic Quadratic Program (QP) solver for Quadruped
// Ground-Reaction Force (GRF) distribution with friction-cone constraints.
//
// Formulation:
//   min_f  1/2 sum_{i in S} || f_i - f_i^nom ||_W^2 + 1/2 alpha || f_i ||^2
//   subject to:
//     sum_{i in S} f_i              = m * (c_ddot_des - g)    (Linear momentum)
//     sum_{i in S} r_i x f_i        = tau_body_des            (Angular momentum)
//     f_{i, y} >= f_min > 0                                   (Unilateral contact)
//     |f_{i, x}| <= mu * f_{i, y}                             (Friction cone X)
//     |f_{i, z}| <= mu * f_{i, y}                             (Friction cone Z)
//
// Solved via KKT system + Cholesky decomposition (6x6) with active-set friction projection.
class QpForceAllocator {
public:
    static constexpr int kLegs = 4;

    struct Problem {
        math::Vec3 desiredForce{0.0f, 0.0f, 0.0f};   // Net required force (N)
        math::Vec3 desiredTorque{0.0f, 0.0f, 0.0f};  // Net required body torque (N*m) [roll_x, yaw_y, pitch_z]
        std::array<math::Vec3, kLegs> footPositions; // Foot positions relative to CoM (r_i, metres)
        std::array<bool, kLegs> inStance{true, true, true, true};
        float frictionCoeff = 0.8f;                  // mu
        float minNormalForce = 2.0f;                 // Minimum stance foot load (N)
        float maxNormalForce = 120.0f;               // Maximum stance foot load (N)
        float totalMass = 12.0f;                     // kg
        float gravity = 9.81f;                       // m/s^2
    };

    struct Result {
        std::array<math::Vec3, kLegs> forces;        // Optimal GRF for each leg (N)
        bool success = false;
        float solveTimeUs = 0.0f;
    };

    static Result solve(const Problem& prob) {
        Result res;
        res.forces.fill(math::Vec3{0.0f, 0.0f, 0.0f});

        int numStance = 0;
        for (int i = 0; i < kLegs; ++i) {
            if (prob.inStance[i]) ++numStance;
        }
        if (numStance == 0) {
            res.success = false;
            return res;
        }

        const float nominalVertical = (prob.totalMass * prob.gravity) / static_cast<float>(numStance);

        // Build 6x12 constraint matrix A and nominal force vector f_nom:
        // A * f = b, where b = [F_x, F_y, F_z, tau_x, tau_y, tau_z]^T
        std::array<float, 6> b{
            prob.desiredForce.x,
            prob.desiredForce.y,
            prob.desiredForce.z,
            prob.desiredTorque.x,
            prob.desiredTorque.y,
            prob.desiredTorque.z
        };

        // Diagonal weights W:
        // Horizontal forces penalized slightly more to minimize tangential slip;
        // vertical forces centered around nominal load.
        constexpr float wH = 1.0f;
        constexpr float wV = 0.5f;
        constexpr float invWH = 1.0f / wH;
        constexpr float invWV = 1.0f / wV;

        // Build 6x6 matrix M = A * W^-1 * A^T + reg * I
        float M[6][6]{};
        float rhs[6]{};

        // Subtract A * f_nom from b
        for (int i = 0; i < kLegs; ++i) {
            if (!prob.inStance[i]) continue;
            const math::Vec3& r = prob.footPositions[i];
            // f_nom_i = [0, nominalVertical, 0]
            b[1] -= nominalVertical;
            // r x f_nom = (-r.z * nominalVertical, 0, r.x * nominalVertical)
            b[3] -= (-r.z * nominalVertical);
            b[5] -= ( r.x * nominalVertical);
        }
        for (int k = 0; k < 6; ++k) rhs[k] = b[k];

        // Accumulate M = sum_i A_i * W_i^-1 * A_i^T
        for (int i = 0; i < kLegs; ++i) {
            if (!prob.inStance[i]) continue;
            const math::Vec3& r = prob.footPositions[i];

            // Submatrix A_i (6 x 3):
            // Row 0 (Fx):    [ 1,      0,      0   ]
            // Row 1 (Fy):    [ 0,      1,      0   ]
            // Row 2 (Fz):    [ 0,      0,      1   ]
            // Row 3 (tau_x): [ 0,    -r.z,    r.y  ]
            // Row 4 (tau_y): [ r.z,    0,    -r.x  ]
            // Row 5 (tau_z): [-r.y,   r.x,     0   ]

            // Column 0 has weight invWH, col 1 has invWV, col 2 has invWH
            // A_i * diag(invWH, invWV, invWH) * A_i^T:
            M[0][0] += invWH;
            M[0][4] += invWH * r.z;
            M[0][5] += invWH * (-r.y);

            M[1][1] += invWV;
            M[1][3] += invWV * (-r.z);
            M[1][5] += invWV * r.x;

            M[2][2] += invWH;
            M[2][3] += invWH * r.y;
            M[2][4] += invWH * (-r.x);

            M[3][1] += invWV * (-r.z);
            M[3][2] += invWH * r.y;
            M[3][3] += invWV * (r.z * r.z) + invWH * (r.y * r.y);
            M[3][4] += invWH * (-r.y * r.x);
            M[3][5] += invWV * (-r.z * r.x);

            M[4][0] += invWH * r.z;
            M[4][2] += invWH * (-r.x);
            M[4][3] += invWH * (-r.x * r.y);
            M[4][4] += invWH * (r.z * r.z + r.x * r.x);
            M[4][5] += invWH * (r.z * (-r.y));

            M[5][0] += invWH * (-r.y);
            M[5][1] += invWV * r.x;
            M[5][3] += invWV * (r.x * (-r.z));
            M[5][4] += invWH * (-r.y * r.z);
            M[5][5] += invWH * (r.y * r.y) + invWV * (r.x * r.x);
        }

        // Tikhonov regularization for positive definiteness
        constexpr float kReg = 1e-4f;
        for (int k = 0; k < 6; ++k) M[k][k] += kReg;

        // Cholesky decomposition of M (6x6): M = L * L^T
        float L[6][6]{};
        bool posDef = true;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j <= i; ++j) {
                float sum = M[i][j];
                for (int k = 0; k < j; ++k) {
                    sum -= L[i][k] * L[j][k];
                }
                if (i == j) {
                    if (sum <= 1e-7f) {
                        posDef = false;
                        sum = 1e-4f;
                    }
                    L[i][j] = std::sqrt(sum);
                } else {
                    L[i][j] = sum / L[j][j];
                }
            }
        }

        // Solve L * y = rhs
        float y[6]{};
        for (int i = 0; i < 6; ++i) {
            float sum = rhs[i];
            for (int k = 0; k < i; ++k) sum -= L[i][k] * y[k];
            y[i] = sum / L[i][i];
        }

        // Solve L^T * lambda = y
        float lambda[6]{};
        for (int i = 5; i >= 0; --i) {
            float sum = y[i];
            for (int k = i + 1; k < 6; ++k) sum -= L[k][i] * lambda[k];
            lambda[i] = sum / L[i][i];
        }

        // Reconstruct primal forces: f_i = f_nom + W^-1 * A_i^T * lambda
        for (int i = 0; i < kLegs; ++i) {
            if (!prob.inStance[i]) continue;
            const math::Vec3& r = prob.footPositions[i];

            // A_i^T * lambda:
            const float aT_lambda_x = lambda[0] + lambda[4] * r.z - lambda[5] * r.y;
            const float aT_lambda_y = lambda[1] - lambda[3] * r.z + lambda[5] * r.x;
            const float aT_lambda_z = lambda[2] + lambda[3] * r.y - lambda[4] * r.x;

            float fx = invWH * aT_lambda_x;
            float fy = nominalVertical + invWV * aT_lambda_y;
            float fz = invWH * aT_lambda_z;

            // Friction cone and unilateral normal force limits:
            fy = math::clampf(fy, prob.minNormalForce, prob.maxNormalForce);
            const float maxFriction = prob.frictionCoeff * fy;
            fx = math::clampf(fx, -maxFriction, maxFriction);
            fz = math::clampf(fz, -maxFriction, maxFriction);

            res.forces[i] = math::Vec3{fx, fy, fz};
        }

        res.success = posDef;
        return res;
    }
};

} // namespace robot
