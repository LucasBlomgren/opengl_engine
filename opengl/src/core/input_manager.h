#pragma once
#include "input.h"
#include "engine_state.h"
#include "camera.h"

class InputManager {
public:
    InputFrame currentFrame;
    InputContext currentContext;
    InputRouter router;

    void route() {
        Consumed consumed{};
        router.route(currentFrame, currentContext, consumed);
    }

    void init(GLFWwindow* window);
    void setPointers(EngineState* state, Camera* camera);

    void beginFrame();
    void setCurrentContext(bool uiWantsMouse, bool uiWantsKeyboard);
    void resetFirstMouse();

    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void mouseMovementCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

private:
    EngineState* engineState = nullptr;
    Camera* camera = nullptr;
    bool firstMouse = true;
    float lastX = 0;
    float lastY = 0;
};