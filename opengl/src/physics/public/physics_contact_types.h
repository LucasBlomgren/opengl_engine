#pragma once

#include <glm/vec3.hpp>

#include "physics/public/physics_handles.h"

namespace physics {

struct ExternalMotionContact {
    BodyHandle bodyA{};
    BodyHandle bodyB{};
    glm::vec3 normal{ 0.0f };
    float penetration = 0.0f;
    bool terrainContact = false;

    ExternalMotionContact() = default;

    ExternalMotionContact(
        const BodyHandle& bodyA,
        const BodyHandle& bodyB,
        const glm::vec3& normal,
        float penetration,
        bool terrainContact = false)
        : bodyA(bodyA),
        bodyB(bodyB),
        normal(normal),
        penetration(penetration),
        terrainContact(terrainContact)
    {}
};

}
