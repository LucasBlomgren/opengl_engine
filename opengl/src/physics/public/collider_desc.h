#pragma once

#include <cstdint>
#include <variant>

#include "physics/public/physics_types.h"

struct BoxShapeDesc {
    glm::vec3 center{ 0.0f };
    glm::vec3 halfExtents{ 0.5f };
};

struct SphereShapeDesc {
    glm::vec3 center{ 0.0f };
    float radius = 0.5f;
};

using ColliderShapeDesc = std::variant<BoxShapeDesc, SphereShapeDesc>;

struct PhysicsMaterial {
    float friction = 0.6f;
    float restitution = 0.0f;
};

struct ColliderDesc {
    PhysicsPose localPose;
    glm::vec3 localScale{ 1.0f };
    ColliderShapeDesc shape;
    PhysicsMaterial material;

    bool enabled = true;
    bool isTrigger = false;

    uint32_t userTag = 0;
};
