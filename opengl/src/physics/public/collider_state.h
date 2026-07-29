#pragma once

#include <cstdint>
#include <variant>

#include "physics/public/physics_handles.h"
#include "physics/public/physics_types.h"

namespace physics {

//==================================================
// For use with Engine::getColliderState()
// to retrieve the current state of a collider.
//==================================================
struct ColliderState {
    BodyHandle body;
    Pose localPose;
    glm::vec3 localScale{ 1.0f };
    Pose worldPose;
    glm::vec3 worldScale{ 1.0f };

    ColliderType type = ColliderType::CUBOID;
    std::variant<BoxGeometry, SphereGeometry> shape;
    AABB bounds;

    bool enabled = true;
    bool isTrigger = false;

    uint32_t userTag = 0;
};

}
