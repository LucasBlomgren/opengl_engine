#pragma once

#include <cstdint>

#include "physics/public/physics_handles.h"
#include "physics/public/physics_types.h"

struct ColliderState {
    RigidBodyHandle body;
    PhysicsPose localPose;
    glm::vec3 localScale{ 1.0f };
    PhysicsPose worldPose;
    glm::vec3 worldScale{ 1.0f };

    bool enabled = true;
    bool isTrigger = false;

    uint32_t userTag = 0;
};
