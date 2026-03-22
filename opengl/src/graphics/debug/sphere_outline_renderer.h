#pragma once

#include "shaders/shader.h"

class SphereOutlineRenderer {
public:
    void init();
    void render(
        Shader& shader, 
        const glm::vec3& cameraPos,
        const glm::vec3& sphereCenter, 
        float radius, 
        const bool asleep, 
        const bool isStatic, 
        const bool selected,
        const bool hovered
    );

    void destroy();

private:
    unsigned int VAO, VBO;
    std::vector<glm::vec3> unitCircle;
};