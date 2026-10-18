#pragma once

#include "robot/policy_weights_default.hpp"

#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cstdio>

namespace robot {

// Lightweight, zero-dependency Neural Network policy for quadruped navigation.
// Architecture: 1 Hidden Layer (32 inputs -> 64 hidden Tanh -> 10 outputs Tanh)
//
// Outputs [0..7]: joint residuals  (added to CPG base angles)
// Output  [8]   : yaw rate command (scaled by max_yaw_rate -> rad/s turning)
// Output  [9]   : forward speed    (scaled by max_fwd_speed -> m/s forward motion)
class PolicyNN {
public:
    static constexpr int kInputDim = 32;
    static constexpr int kHiddenDim = 64;
    static constexpr int kOutputDim = 10;

    float W1[kHiddenDim][kInputDim]{};
    float b1[kHiddenDim]{};
    float W2[kOutputDim][kHiddenDim]{};
    float b2[kOutputDim]{};
    bool loaded = false;

    PolicyNN() {
        loadDefault();
    }

    void loadDefault() {
        for (int i = 0; i < kHiddenDim; ++i) {
            b1[i] = PretrainedPolicy::b1[i];
            for (int j = 0; j < kInputDim; ++j) {
                W1[i][j] = PretrainedPolicy::W1[i][j];
            }
        }
        for (int i = 0; i < kOutputDim; ++i) {
            b2[i] = PretrainedPolicy::b2[i];
            for (int j = 0; j < kHiddenDim; ++j) {
                W2[i][j] = PretrainedPolicy::W2[i][j];
            }
        }
        loaded = true;
    }

    // Simple fast JSON parser for model_weights.json
    bool loadFromJson(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) return false;

        std::string content((std::istreambuf_iterator<char>(file)),
                             std::istreambuf_iterator<char>());

        // Helper to extract a float array from JSON key
        auto extractFloats = [&](const std::string& key) -> std::vector<float> {
            std::vector<float> res;
            size_t pos = content.find("\"" + key + "\"");
            if (pos == std::string::npos) return res;
            pos = content.find('[', pos);
            if (pos == std::string::npos) return res;

            // If 2D array, find outer closing bracket
            int brackets = 1;
            size_t idx = pos + 1;
            while (idx < content.size() && brackets > 0) {
                if (content[idx] == '[') brackets++;
                else if (content[idx] == ']') brackets--;
                idx++;
            }
            std::string sub = content.substr(pos, idx - pos);

            // Parse all numbers
            std::string num;
            for (char c : sub) {
                if ((c >= '0' && c <= '9') || c == '-' || c == '.' || c == 'e' || c == 'E' || c == '+') {
                    num += c;
                } else if (!num.empty()) {
                    try {
                        res.push_back(std::stof(num));
                    } catch (...) {}
                    num.clear();
                }
            }
            if (!num.empty()) {
                try { res.push_back(std::stof(num)); } catch (...) {}
            }
            return res;
        };

        std::vector<float> w1Vals = extractFloats("W1");
        std::vector<float> b1Vals = extractFloats("b1");
        std::vector<float> w2Vals = extractFloats("W2");
        std::vector<float> b2Vals = extractFloats("b2");

        if (w1Vals.size() != kHiddenDim * kInputDim ||
            b1Vals.size() != kHiddenDim ||
            w2Vals.size() != kOutputDim * kHiddenDim ||
            b2Vals.size() != kOutputDim) {
            std::fprintf(stderr, "[PolicyNN] parse error: sizes (%zu, %zu, %zu, %zu), expected (%d, %d, %d, %d)\n",
                         w1Vals.size(), b1Vals.size(), w2Vals.size(), b2Vals.size(),
                         kHiddenDim * kInputDim, kHiddenDim, kOutputDim * kHiddenDim, kOutputDim);
            return false;
        }

        for (int i = 0; i < kHiddenDim; ++i) {
            b1[i] = b1Vals[i];
            for (int j = 0; j < kInputDim; ++j) {
                W1[i][j] = w1Vals[i * kInputDim + j];
            }
        }
        for (int i = 0; i < kOutputDim; ++i) {
            b2[i] = b2Vals[i];
            for (int j = 0; j < kHiddenDim; ++j) {
                W2[i][j] = w2Vals[i * kHiddenDim + j];
            }
        }

        loaded = true;
        std::printf("[PolicyNN] Loaded RL navigation weights from %s (32->64->10)\n", path.c_str());
        return true;
    }

    // Forward inference: 32 inputs -> 64 hidden (Tanh) -> 10 outputs (Tanh)
    inline void forward(const float obs[kInputDim], float action[kOutputDim]) const {
        float hidden[kHiddenDim];

        // Layer 1: hidden = Tanh(W1 * obs + b1)
        for (int i = 0; i < kHiddenDim; ++i) {
            float sum = b1[i];
            for (int j = 0; j < kInputDim; ++j) {
                sum += W1[i][j] * obs[j];
            }
            hidden[i] = std::tanh(sum);
        }

        // Layer 2: action = Tanh(W2 * hidden + b2)
        for (int i = 0; i < kOutputDim; ++i) {
            float sum = b2[i];
            for (int j = 0; j < kHiddenDim; ++j) {
                sum += W2[i][j] * hidden[j];
            }
            action[i] = std::tanh(sum);
        }
    }
};

} // namespace robot
