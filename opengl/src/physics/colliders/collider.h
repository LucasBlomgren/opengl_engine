#pragma once

#include <variant>

#include "aabb.h"
#include "oobb.h"
#include "sphere.h"
#include "core/slot_map.h"
#include "broadphase/broadphase_types.h"
#include "collider_pose.h"

using ColliderShape = std::variant<OOBB, Sphere>;

enum class ColliderType {
    CUBOID,
    SPHERE,
};

// #TODO: decide what is private/public in Collider
struct Collider {
    int id = -1;
    ColliderType type = ColliderType::CUBOID;
    ColliderShape shape{};
    AABB aabb{};
    bool aabbDirty = true;

    ColliderPose pose{}; // physics runtime cache for global pose

    RigidBodyHandle rigidBodyHandle{};
    TransformHandle localTransformHandle{};

    AABB& getAABB();
    void updateCollider(ColliderPose& pose);
    void updateAABB(const ColliderPose& pose);
};