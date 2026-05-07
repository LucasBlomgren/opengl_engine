#include "pch.h"
#include "input_manager.h"

// Static callback wrappers
void InputManager::init(GLFWwindow* window) {
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, mouseMovementCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    lastX = width / 2.0f;
    lastY = height / 2.0f;
    firstMouse = true;
}

// Call at the start of each frame to reset transient input states
void InputManager::beginFrame() {
    currentFrame.mouseDelta = { 0,0 };
    currentFrame.scrollDelta = 0.0f;

    std::fill(std::begin(currentFrame.keyPressed), std::end(currentFrame.keyPressed), false);
    std::fill(std::begin(currentFrame.keyReleased), std::end(currentFrame.keyReleased), false);

    std::fill(std::begin(currentFrame.mousePressed), std::end(currentFrame.mousePressed), false);
    std::fill(std::begin(currentFrame.mouseReleased), std::end(currentFrame.mouseReleased), false);
}

// Set the current input context
void InputManager::setCurrentContext(bool uiWantsMouse, bool uiWantsKeyboard, bool isPlayerMode) {
    currentContext.uiWantsMouse = uiWantsMouse;
    currentContext.uiWantsKeyboard = uiWantsKeyboard;
    currentContext.isPlayerMode = isPlayerMode;
}

// Apply mouse capture based on frame wants from input receivers
void InputManager::applyMouseCapture(GLFWwindow* window, FrameWants& wants) {
    if (wants.captureMouse) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }
    else {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
}

// Reset mouse tracking (e.g., when switching to player mode)
void InputManager::resetFirstMouse() {
    firstMouse = true;
}

// --- GLFW Callbacks ---
void InputManager::mouseMovementCallback(GLFWwindow* window, double xposIn, double yposIn) {
    auto* self = static_cast<InputManager*>(glfwGetWindowUserPointer(window));

    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (self->firstMouse) {
        self->lastX = xpos;
        self->lastY = ypos;
        self->firstMouse = false;
    }

    float xoffset = xpos - self->lastX;
    float yoffset = self->lastY - ypos; // reversed since y-coordinates go from bottom to top

    self->lastX = xpos;
    self->lastY = ypos;

    self->currentFrame.mouseDelta += glm::vec2(xoffset, yoffset);
    self->currentFrame.mousePos = glm::vec2(xpos, ypos);
}

void InputManager::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    auto* self = static_cast<InputManager*>(glfwGetWindowUserPointer(window));
    self->currentFrame.scrollDelta += static_cast<float>(yoffset);
}

void InputManager::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    auto* self = static_cast<InputManager*>(glfwGetWindowUserPointer(window));

    if (action == GLFW_PRESS)   self->currentFrame.keyPressed[key] = true;
    if (action == GLFW_RELEASE) self->currentFrame.keyReleased[key] = true;
    if (action == GLFW_PRESS or action == GLFW_REPEAT) self->currentFrame.keyDown[key] = true;
    if (action == GLFW_RELEASE) self->currentFrame.keyDown[key] = false;
}

void InputManager::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    auto* self = static_cast<InputManager*>(glfwGetWindowUserPointer(window));

    if (action == GLFW_PRESS)   self->currentFrame.mousePressed[button] = true;
    if (action == GLFW_RELEASE) self->currentFrame.mouseReleased[button] = true;
    if (action == GLFW_PRESS or action == GLFW_REPEAT) self->currentFrame.mouseDown[button] = true;
    if (action == GLFW_RELEASE) self->currentFrame.mouseDown[button] = false;
}