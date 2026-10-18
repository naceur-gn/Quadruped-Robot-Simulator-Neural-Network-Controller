#include "gl.hpp"

#include <GLFW/glfw3.h>

#include "render/camera.hpp"
#include "scene/scene.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr double kFixedTimeStep = 1.0 / 120.0;

struct AppState {
    Camera camera;
    Scene scene;
    bool looking = false;
    double lastMouseX = 0.0;
    double lastMouseY = 0.0;
    float moveSpeed = 6.0f;
};

void glfwErrorCallback(int code, const char* description) {
    std::fprintf(stderr, "[glfw] error %d: %s\n", code, description);
}

void framebufferSizeCallback(GLFWwindow*, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int) {
    AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));

    // ---- Left click: push the robot by ray-picking ----
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        if (w > 0 && h > 0) {
            // Convert pixel coords to NDC [-1, 1].
            float ndcX = static_cast<float>(mx) / static_cast<float>(w) * 2.0f - 1.0f;
            float ndcY = 1.0f - static_cast<float>(my) / static_cast<float>(h) * 2.0f;
            float aspect = static_cast<float>(w) / static_cast<float>(h);
            app->scene.tryPushRobot(app->camera, aspect, ndcX, ndcY);
        }
        return;
    }

    // ---- Right click: camera look ----
    if (button != GLFW_MOUSE_BUTTON_RIGHT) return;

    if (action == GLFW_PRESS) {
        app->looking = true;
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwGetCursorPos(window, &app->lastMouseX, &app->lastMouseY);
    } else if (action == GLFW_RELEASE) {
        app->looking = false;
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
}

void cursorPosCallback(GLFWwindow* window, double x, double y) {
    AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (!app->looking) return;

    const double dx = x - app->lastMouseX;
    const double dy = y - app->lastMouseY;
    app->lastMouseX = x;
    app->lastMouseY = y;

    app->camera.rotate(static_cast<float>(dx), static_cast<float>(dy));
}

void scrollCallback(GLFWwindow* window, double, double yOffset) {
    AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (yOffset > 0.0) {
        app->moveSpeed = std::min(app->moveSpeed * 1.15f, 50.0f);
    } else if (yOffset < 0.0) {
        app->moveSpeed = std::max(app->moveSpeed * 0.87f, 1.0f);
    }
}

void keyCallback(GLFWwindow* window, int key, int, int action, int) {
    if (action != GLFW_PRESS && action != GLFW_REPEAT) return;

    AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    auto& robot = app->scene.robot();

    // ---- Robot Direction Controls (Arrow keys & I/J/K/L) -------------------
    if (key == GLFW_KEY_UP || key == GLFW_KEY_I) {
        robot.setDirectionByIndex(8); // North / Forward (+X)
        std::printf("[DIRECTION] Set direction: %s | Distance: %.2fm\n",
                    robot.targetDirectionName(), robot.distanceToTarget());
        return;
    }
    if (key == GLFW_KEY_DOWN || key == GLFW_KEY_K) {
        robot.setDirectionByIndex(2); // South / Backward (-X)
        std::printf("[DIRECTION] Set direction: %s | Distance: %.2fm\n",
                    robot.targetDirectionName(), robot.distanceToTarget());
        return;
    }
    if (key == GLFW_KEY_LEFT || key == GLFW_KEY_J) {
        robot.adjustTargetAngle(-0.2618f); // Turn left by 15 degrees
        std::printf("[DIRECTION] Steered Left: %s (%.1f deg) | Distance: %.2fm\n",
                    robot.targetDirectionName(), robot.targetDirectionAngle() * 180.0f / math::PI, robot.distanceToTarget());
        return;
    }
    if (key == GLFW_KEY_RIGHT || key == GLFW_KEY_L) {
        robot.adjustTargetAngle(+0.2618f); // Turn right by 15 degrees
        std::printf("[DIRECTION] Steered Right: %s (%.1f deg) | Distance: %.2fm\n",
                    robot.targetDirectionName(), robot.targetDirectionAngle() * 180.0f / math::PI, robot.distanceToTarget());
        return;
    }

    // ---- Cardinal & Diagonal Presets (1-9 & Numpad) ------------------------
    int dirPreset = 0;
    if (key == GLFW_KEY_1 || key == GLFW_KEY_KP_1) dirPreset = 1;
    else if (key == GLFW_KEY_2 || key == GLFW_KEY_KP_2) dirPreset = 2;
    else if (key == GLFW_KEY_3 || key == GLFW_KEY_KP_3) dirPreset = 3;
    else if (key == GLFW_KEY_4 || key == GLFW_KEY_KP_4) dirPreset = 4;
    else if (key == GLFW_KEY_6 || key == GLFW_KEY_KP_6) dirPreset = 6;
    else if (key == GLFW_KEY_7 || key == GLFW_KEY_KP_7) dirPreset = 7;
    else if (key == GLFW_KEY_8 || key == GLFW_KEY_KP_8) dirPreset = 8;
    else if (key == GLFW_KEY_9 || key == GLFW_KEY_KP_9) dirPreset = 9;

    if (dirPreset > 0) {
        robot.setDirectionByIndex(dirPreset);
        std::printf("[DIRECTION PRESET] %s | Target: (%.1f, %.1f) | Distance: %.2fm\n",
                    robot.targetDirectionName(), robot.targetCoordinate().x,
                    robot.targetCoordinate().z, robot.distanceToTarget());
        return;
    }

    // Only process one-shot keys on GLFW_PRESS (not repeat)
    if (action != GLFW_PRESS) return;

    if (key == GLFW_KEY_G) {
        robot.toggleWalking();
        std::printf("[gait] Walking: %s\n", robot.walking() ? "ON" : "OFF");
    }
    // Push the robot to test the stabilizer
    constexpr float kPushStrength = 2.5f;
    if (key == GLFW_KEY_P) {
        robot.push(-kPushStrength, 0.0f);   // tip nose down
        const auto cp = robot.capturePoint();
        const bool inPoly = robot.isCapturePointInPolygon();
        std::printf("[key push] Nose down (pitch -%.1f rad/s) | Capture Point: (%.2f, %.2f) | in polygon: %s\n",
                    kPushStrength, cp.x, cp.z, inPoly ? "YES (stable)" : "NO (stepping)");
    }
    if (key == GLFW_KEY_M) {
        robot.toggleControlMode();
        const bool isRL = robot.isRLMode();
        std::printf("\n==============================================\n"
                    "  [MODE] Switched to: %s\n"
                    "==============================================\n\n",
                    isRL ? "RL NEURAL NETWORK (1 hidden layer: 30 -> 64 -> 8)"
                         : "CLASSICAL STABILIZER (LIPM + Capture Point + QP)");
    }
    if (key == GLFW_KEY_U || key == GLFW_KEY_F5) {
        const bool ok = robot.loadPolicyWeights("training/model_weights.json");
        std::printf("[RL] Reload weights from 'training/model_weights.json': %s\n",
                    ok ? "SUCCESS" : "FAILED (file not found or parse error)");
    }
    if (key == GLFW_KEY_R) {
        robot.stabilizer().reset();
        std::printf("[reset] Robot stabilizer reset to equilibrium\n");
    }
}

void handleMovement(GLFWwindow* window, AppState& app, float dt) {
    math::Vec3 direction{0.0f, 0.0f, 0.0f};

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) direction.z += 1.0f;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) direction.z -= 1.0f;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) direction.x += 1.0f;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) direction.x -= 1.0f;
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) direction.y += 1.0f;
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) direction.y -= 1.0f;

    if (direction.lengthSq() <= 0.0f) return;
    direction = direction.normalized();

    const bool sprinting = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    const float speed = app.moveSpeed * (sprinting ? 3.0f : 1.0f);
    app.camera.move(direction, speed * dt);
}

bool writeScreenshot(const char* path, int width, int height) {
    std::vector<unsigned char> pixels(static_cast<size_t>(width) * height * 3);

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    FILE* file = std::fopen(path, "wb");
    if (file == nullptr) return false;

    std::fprintf(file, "P6\n%d %d\n255\n", width, height);
    for (int y = height - 1; y >= 0; --y) {
        const unsigned char* row =
            pixels.data() + static_cast<size_t>(y) * width * 3;
        std::fwrite(row, 1, static_cast<size_t>(width) * 3, file);
    }
    std::fclose(file);
    return true;
}

void printBanner() {
    std::printf(
        "\n"
        "==================================================================\n"
        "  Robot Simulator - Choose Whichever Direction You Want\n"
        "==================================================================\n"
        "  Direction Controls:\n"
        "    Arrow Keys (Up/Down/Left/Right)  : Steer robot forward / back / left / right\n"
        "    I / K / J / L                    : Steer robot forward / back / left / right\n"
        "    Number Keys 1..9 (or Numpad)     : Instant 8-way cardinal & diagonal directions\n"
        "                                       (8: North, 6: East, 2: South, 4: West, etc.)\n"
        "    Left click on floor              : Click anywhere to set target coordinate\n"
        "\n"
        "  Stabilizer & Interaction:\n"
        "    Left click on robot              : Push robot (3D impulse raycast)\n"
        "    P key                            : Push robot (impulse disturbance)\n"
        "    M key                            : Toggle Mode (RL Neural Net <-> Classical)\n"
        "    U / F5 key                       : Reload weights from training/model_weights.json\n"
        "    G key                            : Toggle walking (classical mode)\n"
        "    R key                            : Reset stabilizer equilibrium\n"
        "\n"
        "  Camera Controls:\n"
        "    Right mouse (hold) + move mouse  : Look around\n"
        "    W / A / S / D                    : Move camera\n"
        "    E / Q                            : Up / down camera\n"
        "    Left Shift                       : Fast camera movement\n"
        "==================================================================\n\n");
}

} // namespace

int main(int argc, char** argv) {
    int frameLimit = -1;
    std::string screenshotPath;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            frameLimit = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--screenshot") == 0 && i + 1 < argc) {
            screenshotPath = argv[++i];
        }
    }

    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        std::fprintf(stderr, "[app] glfwInit failed\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window =
        glfwCreateWindow(1280, 720, "Robot Simulator - World", nullptr, nullptr);
    if (window == nullptr) {
        std::fprintf(stderr, "[app] window creation failed\n");
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(frameLimit > 0 ? 0 : 1);

    if (!gl_loadFunctions()) {
        std::fprintf(stderr, "[app] failed to load OpenGL functions\n");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    std::printf("[gl] %s | %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glEnable(GL_MULTISAMPLE);
    glClearColor(0.58f, 0.60f, 0.63f, 1.0f); // Sleek modern studio gray sky

    AppState app;
    app.camera.position = {6.5f, 5.2f, 9.5f};
    app.camera.lookAt({0.0f, 0.4f, 0.0f});

    if (!app.scene.init()) {
        std::fprintf(stderr, "[app] scene initialization failed\n");
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // Try loading trained RL weights from JSON if present
    if (app.scene.robot().loadPolicyWeights("training/model_weights.json")) {
        std::printf("[app] Loaded trained RL weights from 'training/model_weights.json'\n");
    }

    std::printf("[physics] fixed step %.4f s | gravity (%.2f, %.2f, %.2f) | bodies: %zu\n",
                kFixedTimeStep, app.scene.physicsWorld().gravity.x,
                app.scene.physicsWorld().gravity.y, app.scene.physicsWorld().gravity.z,
                app.scene.physicsWorld().bodies().size());

    glfwSetWindowUserPointer(window, &app);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);

    printBanner();

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    glViewport(0, 0, framebufferWidth, framebufferHeight);

    double accumulator = 0.0;
    double lastTime = glfwGetTime();
    int frameCounter = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        const double now = glfwGetTime();
        const double frameTime = std::min(now - lastTime, 0.1);
        lastTime = now;

        handleMovement(window, app, static_cast<float>(frameTime));

        if (frameLimit > 0) {
            // Deterministic test mode: exactly one fixed step per frame.
            app.scene.update(static_cast<float>(kFixedTimeStep));
        } else {
            accumulator += frameTime;
            while (accumulator >= kFixedTimeStep) {
                app.scene.update(static_cast<float>(kFixedTimeStep));
                accumulator -= kFixedTimeStep;
            }
        }

        glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
        const float aspect =
            (framebufferHeight > 0)
                ? static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight)
                : 1.0f;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        app.scene.render(app.camera, aspect);

        ++frameCounter;
        if (frameCounter % 15 == 0) {
            char titleBuf[256];
            const auto& r = app.scene.robot();
            std::snprintf(titleBuf, sizeof(titleBuf),
                          "Robot Simulator | %s | Dir: %s (%.0f deg) | Dist: %.1fm | Steps: %d | Goals: %d",
                          r.isRLMode() ? "Mode: RL Net" : "Mode: Classical",
                          r.targetDirectionName(),
                          r.targetDirectionAngle() * 180.0f / math::PI,
                          r.distanceToTarget(),
                          r.totalStepCount(),
                          r.targetsReached());
            glfwSetWindowTitle(window, titleBuf);
        }

        if (frameLimit > 0 && frameCounter >= frameLimit) {
            if (!screenshotPath.empty()) {
                if (writeScreenshot(screenshotPath.c_str(), framebufferWidth, framebufferHeight)) {
                    std::printf("[app] screenshot written to %s\n", screenshotPath.c_str());
                } else {
                    std::fprintf(stderr, "[app] failed to write screenshot\n");
                }
            }
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }

        glfwSwapBuffers(window);
    }

    if (frameLimit > 0) {
        std::printf("[app] simulated %.2f s\n", frameCounter * kFixedTimeStep);
        const auto& bodies = app.scene.physicsWorld().bodies();
        for (size_t i = 0; i < bodies.size(); ++i) {
            const physics::RigidBody& body = *bodies[i];
            std::printf("[body %zu] pos (%.3f, %.3f, %.3f) | v (%.3f, %.3f, %.3f) | "
                        "w (%.3f, %.3f, %.3f)\n",
                        i, body.position.x, body.position.y, body.position.z,
                        body.linearVelocity.x, body.linearVelocity.y, body.linearVelocity.z,
                        body.angularVelocity.x, body.angularVelocity.y, body.angularVelocity.z);
        }
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
