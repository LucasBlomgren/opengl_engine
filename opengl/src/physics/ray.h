#pragma once
#include <glm/glm.hpp>

namespace Physics {
    struct Ray {
        float length;
        glm::vec3 direction;
        glm::vec3 start;
        glm::vec3 end;

        Ray(const glm::vec3 start, const glm::vec3 direction, float length)
            : start(start),
            direction(direction),
            length(length)
        {
            end = start + direction * length;
        }
    };
}
