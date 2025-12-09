#pragma once

#include <GLFW/glfw3.h>

#include "engine_state.h"
#include "camera.h"

class InputManager 
{
public:
    static void init(GLFWwindow* window);
    void setPointers(EngineState* state, Camera* camera);

    static void processInput(GLFWwindow* window, float deltaTime);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void mouseMovementCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    void resetFirstMouse();

private:
    static EngineState* engineState;

    static Camera* camera;
    static bool firstMouse;
    static float lastX;
    static float lastY;
};