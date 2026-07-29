#pragma once

#include <array>

#include <glm/vec3.hpp>

#include "physics/public/physics_types.h"

namespace physics {

struct Triangle {
    int id = -1;
    std::array<glm::vec3, 3> vertices{};
    AABB bounds{};

    Triangle() = default;

    Triangle(
        int id,
        const glm::vec3& v0,
        const glm::vec3& v1,
        const glm::vec3& v2)
        : id(id),
        vertices{ v0, v1, v2 }
    {
        bounds.worldMin = glm::min(v0, glm::min(v1, v2));
        bounds.worldMax = glm::max(v0, glm::max(v1, v2));
        bounds.worldCenter = (bounds.worldMin + bounds.worldMax) * 0.5f;
        bounds.worldHalfExtents = (bounds.worldMax - bounds.worldMin) * 0.5f;
    }
};

}
