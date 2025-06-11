#pragma once
#include <glm/vec3.hpp>

#include "aabb.h"

class Tri {
public:
    glm::vec3 v0, v1, v2;

    // constructor
    Tri(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
        : v0(v0), v1(v1), v2(v2) {}

    AABB getAABB() const;
};