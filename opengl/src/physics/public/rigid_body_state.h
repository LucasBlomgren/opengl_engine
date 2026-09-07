#pragma once

#include <vector>

#include "physics/public/handles.h"
#include "physics/public/physics_types.h"

namespace physics {

//=====================================================
// For use with Engine::getRigidBodyState() to
// retrieve the current state of a rigid body.
//=====================================================
struct BodyState {
    Pose pose;
    glm::vec3 scale{ 1.0f };

    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };

    BodyType type = BodyType::Dynamic;
    bool reportContacts = false;

    float mass = 0.0f;

    bool asleep = false;
    bool allowSleep = true;
    bool allowGravity = true;

    AABB bounds;
    std::vector<ColliderHandle> colliders;
};

}
