#include "rotateCube.h"

// Uppdaterar en kvaternion-baserad rotation baserat på knapptryck (1–6).
// deltaTime används för att göra rotationen tidsberoende (smoother).
glm::quat rotateCubeWithQuaternion(GLFWwindow* window, glm::quat currentOrientation, float deltaTime)
{
    // Hur fort vi vill rotera (grader per sekund) – justerbart efter behov
    float rotationSpeed = 1.0;
    const float degreesPerSecond = 90.0f; // ex. 90 grader/s
    float angle = glm::radians(degreesPerSecond * rotationSpeed * deltaTime);
    // angle är nu i radianer, och motsvarar "hur många radianer vi roterar detta frame".

    // För att slippa if-satser med +1 / -1, kan vi göra såhär:

    // Rotera runt X-axeln?
    if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
    {
        // KEY_1 => -1 kring X
        glm::quat delta = glm::angleAxis(-angle, glm::vec3(1.0f, 0.0f, 0.0f));
        currentOrientation = currentOrientation * delta;
    }
    else if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
    {
        // KEY_2 => +1 kring X
        glm::quat delta = glm::angleAxis(+angle, glm::vec3(1.0f, 0.0f, 0.0f));
        currentOrientation = currentOrientation * delta;
    }

    // Rotera runt Y-axeln?
    if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
    {
        glm::quat delta = glm::angleAxis(-angle, glm::vec3(0.0f, 1.0f, 0.0f));
        currentOrientation = currentOrientation * delta;
    }
    else if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
    {
        glm::quat delta = glm::angleAxis(+angle, glm::vec3(0.0f, 1.0f, 0.0f));
        currentOrientation = currentOrientation * delta;
    }

    // Rotera runt Z-axeln?
    if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS)
    {
        glm::quat delta = glm::angleAxis(-angle, glm::vec3(0.0f, 0.0f, 1.0f));
        currentOrientation = currentOrientation * delta;
    }
    else if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS)
    {
        glm::quat delta = glm::angleAxis(+angle, glm::vec3(0.0f, 0.0f, 1.0f));
        currentOrientation = currentOrientation * delta;
    }

    // Normalisera för säkerhets skull (undvik drift från enhetskvaternion)
    currentOrientation = glm::normalize(currentOrientation);

    // Returnera den nya orienteringen
    return currentOrientation;
}