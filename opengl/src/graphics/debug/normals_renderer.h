#pragma once
#include <glm/glm.hpp>

class NormalsRenderer {
public:
    void init();

    glm::mat4 modelX{ 1.0f };
    glm::mat4 modelY{ 1.0f };
    glm::mat4 modelZ{ 1.0f };

private:
    static bool sInitialized;
};