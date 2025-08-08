#pragma once

#include "shader.h"

class SphereOutlineRenderer {
public:
    void init();
    void render(Shader& shader, glm::vec3& cameraPos, glm::vec3& sphereCenter, float radius, const bool asleep, const bool raycastHit);

private:
    unsigned int VAO, VBO;
    std::vector<glm::vec3> unitCircle;
};