#pragma once
#include <vector>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include "game/transform.h"

class Sphere {
public:
    glm::vec3 lCenter{ 0.0f };
    glm::vec3 wCenter;
    float     radius;

    Sphere(const Transform& t) {
        wCenter = glm::vec3(t.modelMatrix * glm::vec4(lCenter, 1.0f));
        radius = t.scale.x * 0.5f;
    }

    void update(const Transform& t) {
        wCenter = glm::vec3(t.modelMatrix * glm::vec4(lCenter, 1.0f));
        radius = t.scale.x * 0.5f;
    }
};