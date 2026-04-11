#pragma once

#include <variant>

#include "aabb.h"
#include "oobb.h"
#include "sphere.h"
#include "core/slot_map.h"
#include "broadphase/broadphase_types.h"
#include "game/transform_utils.h"

using ColliderShape = std::variant<OOBB, Sphere>;

enum class ColliderType {
    CUBOID,
    SPHERE,
};

// #TODO: integrate and use instead of globalTransform in physics
// 1. update simple pos/ori/scale before broadphase
// 2. then compute modelMatrix and invModelMatrix for narrowphase and collision response if necessary
struct ColliderPose {
    glm::vec3 position{ 0.0f };
    glm::quat orientation{ 1,0,0,0 };
    glm::vec3 scale{ 1.0f };

    glm::mat3 rotationMatrix{ 1.0f };
    glm::mat3 invRotationMatrix{ 1.0f };

    bool narrowphaseReady = false;
    glm::mat4 modelMatrix{ 1.0f };
    glm::mat4 invModelMatrix{ 1.0f };
};

// #TODO: decide what is private/public in Collider
struct Collider {
    int id = -1;
    ColliderType type = ColliderType::CUBOID;
    ColliderShape shape{};
    AABB aabb{};
    bool aabbDirty = true;

    ColliderPose pose{};

    RigidBodyHandle rigidBodyHandle{};
    TransformHandle localTransformHandle{};
    Transform globalTransform;

    AABB& getAABB();
    void updateCollider(const Transform& t);
    void updateAABB(const Transform& t);
};