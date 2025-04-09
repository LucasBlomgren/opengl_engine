#pragma once
#include <glm/glm.hpp>

struct Light {
    glm::vec3 position;
    glm::vec3 color;
    float intensity;

    Light(glm::vec3 pos, glm::vec3 col, float intensity = 1.0f)
        : position(pos), color(col), intensity(intensity) {}
};