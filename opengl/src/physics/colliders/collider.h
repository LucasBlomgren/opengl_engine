#pragma once

#include <variant>

#include "aabb.h"
#include "oobb.h"
#include "sphere.h"
#include "core/slot_map.h"
#include "broadphase/broadphase_types.h"

class GameObject;
class TriMesh;

enum class ColliderType {
    CUBOID,
    SPHERE,
};

using ColliderShape = std::variant<OOBB, Sphere>;

// #TODO: decide what is private/public in Collider
struct Collider {
    int id;
    ColliderType type;
    ColliderShape shape;
    AABB aabb;
    bool aabbDirty = true;

    GameObjectHandle gameObjectHandle;
    RigidBodyHandle rigidBodyHandle;

    AABB& getAABB();
    void updateCollider(const Transform& t);
    void updateAABB(const Transform& t);
};