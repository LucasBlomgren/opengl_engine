#pragma once

#include "physics/public/physics_types.h"

//=====================================================
// For use with PhysicsEngine::getRigidBodyState() to 
// retrieve the current state of a rigid body.
//=====================================================
struct RigidBodyState {
    PhysicsPose pose;

    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };

    BodyType type = BodyType::Dynamic;

    bool asleep = false;
};