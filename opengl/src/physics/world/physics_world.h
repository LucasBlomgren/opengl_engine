#pragma once

#include <unordered_map>

#include "core/slot_map.h"

#include "physics/public/handles.h"
#include "physics/bodies/rigidbody.h"
#include "physics/colliders/collider.h"

namespace physics::internal {

class PhysicsWorld {
public:
    void clear();

    // Handle reservation
    BodyHandle reserveBodyHandle();
    ColliderHandle reserveColliderHandle();

    void releaseBodyReservation(BodyHandle handle);
    void releaseColliderReservation(ColliderHandle handle);

    // Commit to active storage
    RigidBody* commitBody(
        BodyHandle handle,
        RigidBody&& body);

    Collider* commitCollider(
        ColliderHandle handle,
        Collider&& collider);

    // Active deletion
    void destroyBody(BodyHandle handle);
    void destroyCollider(ColliderHandle handle);

    // Active lookup only
    RigidBody& getBody(BodyHandle handle);
    RigidBody* tryGetBody(BodyHandle handle);
    const RigidBody* tryGetBody(BodyHandle handle) const;

    Collider& getCollider(ColliderHandle handle);
    Collider* tryGetCollider(ColliderHandle handle);
    const Collider* tryGetCollider(ColliderHandle handle) const;

    // Needed by caches, iteration and queries
    SlotMap<RigidBody, BodyHandle>& bodyStorage();
    const SlotMap<RigidBody, BodyHandle>& bodyStorage() const;

    SlotMap<Collider, ColliderHandle>& colliderStorage();
    const SlotMap<Collider, ColliderHandle>& colliderStorage() const;

    AABB computeBodyAABB(const RigidBody& body);

private:
    int colliderId = 0;
    int rigidBodyId = 0;
    SlotMap<RigidBody, BodyHandle> bodies{"RigidBody"};
    SlotMap<Collider, ColliderHandle> colliders{"Collider"};
};
}