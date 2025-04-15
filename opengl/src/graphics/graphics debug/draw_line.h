#pragma once

#include <glm/glm.hpp>
#include "shader.h"

// Deklarera setupLine-funktionen
unsigned int setupLine();

// Deklarera drawLine-funktionen
void drawLine(const Shader& shader, unsigned int& VAO, const glm::vec3& lineStart, const glm::vec3& lineEnd, const glm::vec3& color);