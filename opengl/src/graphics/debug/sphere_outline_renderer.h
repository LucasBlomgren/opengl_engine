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
    ) const;

    void destroy();

private:
    unsigned int VAO = 0;
    unsigned int VBO = 0;
    std::vector<glm::vec3> unitCircle{};
};