#pragma once
#include <vector>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include "game/transform_utils.h"

class Sphere {
public:
    glm::vec3 centerWorld{ 0.0f };
    glm::vec3 centerLocal{ 0.0f };
    float     radiusWorld = 0.0f;

    Sphere(const Transform& t) {
        centerWorld = glm::vec3(t.modelMatrix * glm::vec4(centerLocal, 1.0f));
        radiusWorld = t.scale.x; // sphere mesh has diameter = 2 [-1.0f, 1.0f] in local space
    }

    void update(const Transform& t) {
        centerWorld = glm::vec3(t.modelMatrix * glm::vec4(centerLocal, 1.0f));
        radiusWorld = t.scale.x;
    }
};