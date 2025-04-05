#pragma once

#include <GLFW/glfw3.h>
#include <glm/gtc/quaternion.hpp>

// Uppdaterar en kvaternion-baserad rotation baserat på knapptryck (1–6).
// deltaTime används för att göra rotationen tidsberoende (smoother).
glm::quat rotateCubeWithQuaternion(GLFWwindow* window, glm::quat currentOrientation, float deltaTime);