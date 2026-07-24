#pragma once

#include <vector>

#include "physics/public/collider_desc.h"
#include "physics/public/physics_types.h"

struct RigidBodyDesc {
    PhysicsPose pose;
    glm::vec3 scale{ 1.0f };

    BodyType type = BodyType::Dynamic;
    MotionControl motionControl = MotionControl::Physics;
    ContactResponseMode responseMode = ContactResponseMode::Normal;

    float mass = 1.0f;

    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };

    bool allowGravity = true;
    bool allowSleep = true;
    bool startAsleep = false;

    float sleepCounterThreshold = 1.5f;

    std::vector<ColliderDesc> colliders;
};