#include "pch.h"
#include "camera.h"

glm::mat4 Camera::GetViewMatrix() {
    return glm::lookAt(position, position + front, up);
}

void Camera::updateDeltaTime(float dt) {
    deltaTime = dt;
}

void Camera::addInputRouter(InputRouter& router) {
    router.add(this);
}

void Camera::handleInput(const InputFrame& in, const InputContext& ctx, Consumed& consumed, FrameWants& wants) {
    // --- keyboard ---
    if (!consumed.keyboard and !ctx.isPlayerMode)
    {
        // movement speed
        if (in.keyDown[GLFW_KEY_LEFT_SHIFT]) {
            movementSpeed = fastSpeed;
        } else {
            movementSpeed = standardSpeed;
        }

        // process keyboard input
        float velocity = movementSpeed * deltaTime;
        if (in.keyDown[GLFW_KEY_W]) ProcessKeyboard(FORWARD, velocity);
        if (in.keyDown[GLFW_KEY_S]) ProcessKeyboard(BACKWARD, velocity);
        if (in.keyDown[GLFW_KEY_A]) ProcessKeyboard(LEFT, velocity);
        if (in.keyDown[GLFW_KEY_D]) ProcessKeyboard(RIGHT, velocity);
        if (in.keyDown[GLFW_KEY_E]) ProcessKeyboard(UP, velocity);
        if (in.keyDown[GLFW_KEY_Q]) ProcessKeyboard(DOWN, velocity);

        consumed.keyboard = true;
    }

    // --- mouse ---
    if (!wants.cameraLook) {
        return;
    }

    ProcessMouseMovement(in.mouseDelta.x, in.mouseDelta.y);
    ProcessMouseScroll(in.scrollDelta);
    consumed.mouse = true;
}

// processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
void Camera::ProcessKeyboard(Camera_Movement direction, float vel) 
{
    if (direction == FORWARD)   position += front * vel;
    if (direction == BACKWARD)  position -= front * vel;
    if (direction == LEFT)      position -= right * vel;
    if (direction == RIGHT)     position += right * vel;
    if (direction == UP)        position += worldUp * vel;
    if (direction == DOWN)      position += worldUp * -vel;
}

void Camera::ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch) {
    xoffset *= mouseSensitivity;
    yoffset *= mouseSensitivity;
    yaw += xoffset;
    pitch += yoffset;
    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (constrainPitch)
    {
        if (pitch > 89.0f)
            pitch = 89.0f;
        if (pitch < -89.0f)
            pitch = -89.0f;
    }
    // update front, right and Up Vectors using the updated Euler angles
    updateCameraVectors();
}

void Camera::ProcessMouseScroll(float yoffset) {
    zoom -= (float)yoffset;
    if (zoom < 1.0f)
        zoom = 1.0f;
    if (zoom > 45.0f)
        zoom = 45.0f;
}

void Camera::updateCameraVectors() {
    // calculate the new Front vector
    glm::vec3 temp;
    temp.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    temp.y = sin(glm::radians(pitch));
    temp.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front = glm::normalize(temp);
    // also re-calculate the right and Up vector
    right = glm::normalize(glm::cross(front, worldUp));  // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
    up = glm::normalize(glm::cross(right, front));
}