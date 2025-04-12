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
    engineState = state;
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
        if (key == GLFW_KEY_Q)
            engineState->toggleShowAABB();
        if (key == GLFW_KEY_E)
            engineState->toggleShowContactPoints();
        if (key == GLFW_KEY_R)
            engineState->toggleShowNormals();
        if (key == GLFW_KEY_T)
            engineState->toggleShowCollisionNormal();
        if (key == GLFW_KEY_G)
            engineState->togglePause();
        if (key == GLFW_KEY_H)
            engineState->setPressedKey("H");
    }
}

void InputManager::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) 
{
    if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS)
        engineState->setPressedKey("Mouse1");
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

    //Mesh& obj = meshList[1];
    //Mesh& obj = *light;
    //float speed = 750;
    //if(!paused)
    //{
    //    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(0, 0, 1) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(0, 0, -1) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(1, 0, 0) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(-1, 0, 0) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(0, 1, 0) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
    //        obj.addForce(glm::vec3(0, -1, 0) * speed);
    //    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
    //        obj.linearVelocity = glm::vec3();
    //        obj.angularVelocity = glm::vec3();
    //    }
    //}
}