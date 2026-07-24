#pragma once

#include "graphics/shaders/shader.h"

unsigned int setupContactPoint();
void renderContactPoint(const Shader& shader, const unsigned int& VAO, const glm::vec3& contactPoint);