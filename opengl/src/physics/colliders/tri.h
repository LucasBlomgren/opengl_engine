#pragma once
#include <glm/vec3.hpp>

#include "aabb.h"

class Tri {
    glm::vec3 v0, v1, v2;

public:
    AABB getAABB() const;
};