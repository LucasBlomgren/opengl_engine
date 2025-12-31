#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "input.h"

enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

// Default camera values
const float YAW = 40.0f;
const float PITCH = -30.0f;
const float SPEED = 30.0f;
const float SENSITIVITY = 0.05f;
const float ZOOM = 45.0f;

class Camera : public IInputReceiver {
public:
    // camera Attributes
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 right;
    glm::vec3 worldUp;

    // euler Angles
    float yaw;
    float pitch;

    // camera options
    float standardSpeed = 40.0f;
    float fastSpeed = 1000.0f;

    float movementSpeed = standardSpeed;
    float mouseSensitivity;
    float zoom;

    // constructor with vectors
    Camera(
        glm::vec3 pos = glm::vec3(0.0f, 0.0f, 0.0f), 
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), 
        float yaw = YAW, 
        float pitch = PITCH) : 
        front(glm::vec3(0.0f, 0.0f, -1.0f)), 
        mouseSensitivity(SENSITIVITY), 
        zoom(ZOOM)
    {
        position = pos;
        worldUp = up;
        this->yaw = yaw;
        this->pitch = pitch;
        updateCameraVectors();
    }

    // update delta time
    void updateDeltaTime(float dt);
    
    // add this input receiver to the input router
    void addInputRouter(InputRouter& router);
    // process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
    void handleInput(const InputFrame& in, const InputContext& ctx, Consumed& c, FrameWants& wants);

    // returns the view matrix calculated using Euler Angles and the LookAt Matrix
    glm::mat4 GetViewMatrix();

    // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
    void ProcessKeyboard(Camera_Movement direction, float deltaTime);

    // processes input received from a mouse input system. Expects the offset value in both the x and y direction.
    void ProcessMouseMovement(float xoffset, float yoffset, GLboolean constrainPitch = true);

    // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset);

private:
    // calculates the front vector from the Camera's (updated) Euler Angles
    void updateCameraVectors();

    float deltaTime = 0.0f;
};