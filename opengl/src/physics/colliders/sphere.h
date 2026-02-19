#pragma once
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <vector>

class Sphere {
public:
    glm::vec3 lCenter{ 0.0f };
    glm::vec3 wCenter;
    float     radius;

    Sphere(const glm::mat4& modelMatrix, const float radi) {
        wCenter = glm::vec3(modelMatrix * glm::vec4(lCenter, 1.0f));
        radius = radi;
    }

    void update(const glm::mat4& modelMatrix, float newRadius) {
        wCenter = glm::vec3(modelMatrix * glm::vec4(lCenter, 1.0f));
        radius = newRadius;
    }
};