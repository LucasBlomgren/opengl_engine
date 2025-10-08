#include "pch.h"
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
        engineState->SetKeyState(key, true);

        if (key == GLFW_KEY_1) 
            engineState->toggleShowNormals();       // Toggle the visibility of normals in the scene
        if (key == GLFW_KEY_2)
            engineState->toggleShowAABB();          // Toggle the visibility of axis-aligned bounding boxes in the scene
        if (key == GLFW_KEY_3)
            engineState->toggleShowColliders();     // Toggle the visibility of oriented bounding boxes in the scene
        if (key == GLFW_KEY_4)
            engineState->toggleShowContactPoints(); // Toggle the visibility of contact points in the scene
        if (key == GLFW_KEY_0)
            engineState->toggleShowFPS();           // Toggle std::cout of metrics in the console

        if (key == GLFW_KEY_G)
            engineState->togglePause();             // Toggle pause/resume of the physics simulation
        if (key == GLFW_KEY_F)
            engineState->setAdvanceStep(true);      // Advance the physics simulation by one step

        if (key == GLFW_KEY_F5)
            engineState->toggleShowBVH_awake();           // Toggle the visibility of the BVH tree in the scene
        if (key == GLFW_KEY_F6)
            engineState->toggleShowBVH_asleep();
        if (key == GLFW_KEY_F7)
            engineState->toggleShowBVH_static();
        if (key == GLFW_KEY_F8)
            engineState->toggleShowBVH_terrain();

        const char* name = glfwGetKeyName(key, scancode);
        if (name) {
            engineState->setPressedKey(name);
        }
        else {
            std::string special;
            switch (key) {
            case GLFW_KEY_SPACE:        special = "SPACE";        break;
            case GLFW_KEY_ENTER:        special = "ENTER";        break;
            case GLFW_KEY_ESCAPE:       special = "ESCAPE";       break;
            case GLFW_KEY_LEFT:         special = "LEFT";         break;
            case GLFW_KEY_RIGHT:        special = "RIGHT";        break;
            case GLFW_KEY_UP:           special = "UP";           break;
            case GLFW_KEY_DOWN:         special = "DOWN";         break;

            case GLFW_KEY_F1:           special = "F1";           break;
            case GLFW_KEY_F2:           special = "F2";           break;
            case GLFW_KEY_F3:           special = "F3";           break;
            case GLFW_KEY_F4:           special = "F4";           break;
            case GLFW_KEY_F5:           special = "F5";           break;
            case GLFW_KEY_F6:           special = "F6";           break;
            case GLFW_KEY_F7:           special = "F7";           break;
            case GLFW_KEY_F8:           special = "F8";           break;
            case GLFW_KEY_F9:           special = "F9";           break;
            case GLFW_KEY_F10:          special = "F10";          break;
            case GLFW_KEY_F11:          special = "F11";          break;
            case GLFW_KEY_F12:          special = "F12";          break;

            case GLFW_KEY_LEFT_SHIFT:   special = "LEFT_SHIFT";   break;
            case GLFW_KEY_LEFT_CONTROL: special = "LEFT_CONTROL"; break;
            case GLFW_KEY_LEFT_ALT:     special = "LEFT_ALT";     break;

            default:                    special = "UNKNOWN";      break;
            }

            engineState->setPressedKey(special);
        }
    }
    else if (action == GLFW_RELEASE) {
        engineState->SetKeyState(key, false);
    }
}

void InputManager::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) 
{
    // PRESS
    if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS)
        engineState->setPressedKey("M1_PRESS");                 // Place an object in the scene
    if (button == GLFW_MOUSE_BUTTON_2 && action == GLFW_PRESS)
        engineState->setPressedKey("M2_PRESS");                 // Select an object in the scene
    if (button == GLFW_MOUSE_BUTTON_3 && action == GLFW_PRESS)
        engineState->setPressedKey("M3_PRESS");                 // Create a new object in the scene with high velocity

    // RELEASE
    if (button == GLFW_MOUSE_BUTTON_2 && action == GLFW_RELEASE)
        engineState->setPressedKey("M2_RELEASE");               // Drop the selected object in the scene
}

void InputManager::processInput(GLFWwindow* window, float deltaTime)
{
    if (!engineState->isPlayerMode()) {
        // Camera controls
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
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
            camera->ProcessKeyboard(DOWN, deltaTime);
    }
    else {
        // Player controls
        engineState->SetKeyState(GLFW_KEY_W, glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS);
        engineState->SetKeyState(GLFW_KEY_S, glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS);
        engineState->SetKeyState(GLFW_KEY_A, glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS);
        engineState->SetKeyState(GLFW_KEY_D, glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS);
    }

    // Hold shift to move faster
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) 
        camera->movementSpeed = camera->fastSpeed;
    else
        camera->movementSpeed = camera->standardSpeed;

    // Exit 
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}