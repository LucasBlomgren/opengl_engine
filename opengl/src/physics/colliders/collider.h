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

struct Collider {
    ColliderType type;
    ColliderShape shape;
    AABB aabb;
    bool aabbDirty = true;

    GameObjectHandle ownerHandle;    
    RigidBodyHandle rigidBodyHandle;
    BroadphaseHandle broadphaseHandle;

    GameObject* owner;    // #TODO: krasch här pga reallokering i slotmap. Kommer fixas när physics istället kommer jobba med PhysicsWorld och Collider handles 

    Collider(GameObject* o) 
        : owner(o) {}

    AABB& getAABB() const;
    void updateCollider(const Transform& t);
    void updateAABB(const Transform& t);
};