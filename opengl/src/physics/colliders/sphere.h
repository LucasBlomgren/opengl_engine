#pragma once

#include <algorithm>

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/public/physics_types.h"

namespace physics::internal {

class Sphere {
public:
    glm::vec3 centerLocal{ 0.0f };
    glm::vec3 centerWorld{ 0.0f };

    float radiusLocal = 0.5f;
    float radiusWorld = 0.5f;

    Sphere() = default;

    explicit Sphere(float radius, const glm::vec3& centerLocal) 
        : radiusLocal(radius), 
        radiusWorld(radius),
        centerLocal(centerLocal)
    {}

    void update(const Pose& worldPose, const glm::vec3& worldScale) {
        glm::mat3 rotation = glm::mat3_cast(worldPose.orientation);
        centerWorld = worldPose.position + rotation * (worldScale * centerLocal);

        const glm::vec3 absScale = glm::abs(worldScale);
        radiusWorld = radiusLocal * (std::max)({ absScale.x, absScale.y, absScale.z });
    }
};

}
