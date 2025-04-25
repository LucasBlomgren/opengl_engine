#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "shader.h"
#include "aabb.h"

class AABBRenderer
{
public:
    void setupWireframeBox(const AABB& box);
    void drawBox(const AABB& box, Shader& shader, const bool asleep);

private:
    unsigned int VAO, VBO;
    glm::mat4 model{ 1.0f };
    glm::vec3 color{ 0.9f, 0.7f, 0.2f };
};