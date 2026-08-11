#pragma once

#include <limits>
#include <glm/vec3.hpp>

#include "physics/public/handles.h"

namespace physics {

enum class BodySet {
    Awake,
    Asleep,
    Static
};

struct Ray {
    float length = 0.0f;
    glm::vec3 direction{ 0.0f };
    glm::vec3 start{ 0.0f };
    glm::vec3 end{ 0.0f };

    Ray(
        const glm::vec3& start,
        const glm::vec3& direction,
        float length)
        : length(length),
        direction(direction),
        start(start),
        end(start + direction * length)
    {}
};

struct RaycastHit {
    bool hit = false;
    BodyHandle bodyHandle;
    glm::vec3 point{ 0.0f };
    glm::vec3 normal{ 0.0f };
    float t = std::numeric_limits<float>::max();
};

}
