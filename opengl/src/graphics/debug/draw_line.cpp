#include "pch.h"
#include "draw_line.h"

namespace {
   Shader* lineShader = nullptr;
   GLuint lineVAO = 0;
}

void initLineRenderer(Shader* shader) {
   lineShader = shader;

   glGenVertexArrays(1, &lineVAO); glcount::incVAO();
   glBindVertexArray(lineVAO);
   glBindVertexArray(0);
}

glm::vec3 colorFromIndex(uint32_t index) {
   uint32_t h = index + 1;
   h ^= h >> 16;
   h *= 0x7feb352d;
   h ^= h >> 15;
   h *= 0x846ca68b;
   h ^= h >> 16;

   return glm::vec3(
      (h & 255) / 255.0f,
      ((h >> 8) & 255) / 255.0f,
      ((h >> 16) & 255) / 255.0f
   );
}

void drawLine(
   const glm::vec3& start,
   const glm::vec3& end,
   uint32_t index)
{
   lineShader->use();

   lineShader->setInt("debug.objectType", 3);
   lineShader->setVec3("debug.lineStart", start);
   lineShader->setVec3("debug.lineEnd", end);
   lineShader->setBool("debug.useUniformColor", true);
   lineShader->setVec3("debug.uColor", colorFromIndex(index));

   glLineWidth(2.0);
   glBindVertexArray(lineVAO);
   glDrawArrays(GL_LINES, 0, 2);
   glBindVertexArray(0);
}