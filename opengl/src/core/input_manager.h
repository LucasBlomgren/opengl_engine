#pragma once
#include "input.h"

// Manages input state and routing
class InputManager {
public:
    // Current frame's input state and context
    InputFrame currentFrame;
    InputContext currentContext;
    InputRouter router;

    Consumed consumed;
    FrameWants wants;

    // Route input to registered receivers and apply mouse capture
    void route(GLFWwindow* window) {
        consumed = {};
        wants = {};

        router.route(currentFrame, currentContext, consumed, wants);
        applyMouseCapture(window, wants);
    }

    // --- Initialization and Frame Management ---
    void init(GLFWwindow* window);
    void beginFrame();
    void setCurrentContext(bool uiWantsMouse, bool uiWantsKeyboard, bool isPlayerMode);
    void applyMouseCapture(GLFWwindow* window, FrameWants& wants);
    void resetFirstMouse();

    // --- GLFW Callbacks ---
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void mouseMovementCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

private:
    // Mouse tracking
    bool firstMouse = true;
    float lastX = 0;
    float lastY = 0;
};