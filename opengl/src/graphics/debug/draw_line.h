#pragma once

#include "graphics/shaders/shader.h"

void initLineRenderer(Shader* shader);

glm::vec3 colorFromIndex(uint32_t index);

void drawLine(
   const glm::vec3& lineStart, 
   const glm::vec3& lineEnd, 
   uint32_t index
);