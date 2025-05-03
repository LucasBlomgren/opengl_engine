#include "input_manager.h"

EngineState* InputManager::engineState = nullptr;
Camera* InputManager::camera = nullptr;

bool InputManager::firstMouse;
float InputManager::lastX;
float InputManager::lastY;

void InputManager::init(GLFWwindow* window) 
{
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, mouseMovementCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    lastX = width / 2.0f;
    lastY = height / 2.0f;
    firstMouse = true;
}

void InputManager::setPointers(EngineState* state, Camera* camera) {
    this->engineState = state;
    this->camera = camera;
}

void InputManager::mouseMovementCallback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera->ProcessMouseMovement(xoffset, yoffset);
}

void InputManager::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    camera->ProcessMouseScroll(static_cast<float>(yoffset));
}

void InputManager::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) 
{
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_1)
            engineState->toggleShowNormals();
        if (key == GLFW_KEY_2)
            engineState->toggleShowAABB();
        if (key == GLFW_KEY_3)
            engineState->toggleShowOOBB();
        if (key == GLFW_KEY_4)
            engineState->toggleShowContactPoints();
        if (key == GLFW_KEY_5)
            engineState->toggleShowBVH();
        if (key == GLFW_KEY_0)
            engineState->toggleShowFPS();

        if (key == GLFW_KEY_G)
            engineState->togglePause();
        if (key == GLFW_KEY_H)
            engineState->setPressedKey("H");
        if (key == GLFW_KEY_K)
            engineState->setPressedKey("K");
        if (key == GLFW_KEY_6)
            engineState->setPressedKey("6");
    }
}

void InputManager::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) 
{
    if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS)
        engineState->setPressedKey("Mouse1");
    if (button == GLFW_MOUSE_BUTTON_2 && action == GLFW_PRESS)
        engineState->setPressedKey("Mouse2");
    if (button == GLFW_MOUSE_BUTTON_3 && action == GLFW_PRESS)
        engineState->setPressedKey("Mouse3");
}

void InputManager::processInput(GLFWwindow* window, float deltaTime)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera->ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera->ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera->ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera->ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        camera->ProcessKeyboard(UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
        camera->ProcessKeyboard(DOWN, deltaTime);
}