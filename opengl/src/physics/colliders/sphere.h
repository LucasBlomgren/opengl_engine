#pragma once
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

class Sphere {
public:
    glm::vec3 lCenter{ 0.0f };
    glm::vec3 wCenter;
    float     radius;

    void update(const glm::mat4& modelMatrix) {
        wCenter = glm::vec3(modelMatrix * glm::vec4(lCenter, 1.0f));
    }
};