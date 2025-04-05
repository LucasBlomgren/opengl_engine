#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "engineState.h"
#include "camera.h"

class InputManager 
{
public:
    static void Init(GLFWwindow* window);
    void SetPointers(EngineState* state, Camera* camera);

    static void ProcessInput(GLFWwindow* window, float deltaTime);
    static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void MouseMovementCallback(GLFWwindow* window, double xpos, double ypos);
    static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);

private:
    static EngineState* engineState;

    static Camera* camera;
    static bool firstMouse;
    static float lastX;
    static float lastY;
};