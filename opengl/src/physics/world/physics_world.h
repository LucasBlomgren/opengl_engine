#pragma once

#include <unordered_map>

#include "core/slot_map.h"

#include "physics/public/physics_handles.h"
#include "physics/bodies/rigidbody.h"
#include "physics/colliders/collider.h"

class PhysicsEngine;

class PhysicsWorld {
public:
    void clear();

    // getters
    RigidBody* getRigidBody(RigidBodyHandle handle);
    const RigidBody* getRigidBody(RigidBodyHandle handle) const;

    Collider* getCollider(ColliderHandle handle);
    const Collider* getCollider(ColliderHandle handle) const;

    AABB computeBodyAABB(const RigidBody& body);

    SlotMap<Collider, ColliderHandle>& getCollidersMap();
    SlotMap<RigidBody, RigidBodyHandle>& getRigidBodiesMap();

    const SlotMap<Collider, ColliderHandle>& getCollidersMap() const;
    const SlotMap<RigidBody, RigidBodyHandle>& getRigidBodiesMap() const;

    // creation and deletion
    RigidBodyHandle createRigidBody();
    ColliderHandle createCollider();

    bool activateRigidBody(RigidBodyHandle handle);
    bool activateCollider(ColliderHandle handle);

    void discardPendingRigidBody(RigidBodyHandle handle);
    void discardPendingCollider(ColliderHandle handle);

    void deleteRigidBody(RigidBodyHandle handle);
    void deleteCollider(ColliderHandle handle);

    bool isRigidBodyActive(RigidBodyHandle handle) const;
    bool isColliderActive(ColliderHandle handle) const;
    bool isRigidBodyPending(RigidBodyHandle handle) const;
    bool isColliderPending(ColliderHandle handle) const;

private:
    int colliderId = 0;
    int rigidBodyId = 0;
    SlotMap<Collider, ColliderHandle> colliders;
    SlotMap<RigidBody, RigidBodyHandle> rigidBodies;
    std::unordered_map<ColliderHandle, Collider> pendingColliders;
    std::unordered_map<RigidBodyHandle, RigidBody> pendingRigidBodies;
};
