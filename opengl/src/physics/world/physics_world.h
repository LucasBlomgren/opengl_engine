#pragma once

#include <unordered_map>

#include "core/slot_map.h"

#include "physics/public/physics_handles.h"
#include "physics/bodies/rigidbody.h"
#include "physics/colliders/collider.h"

namespace physics::internal {

class PhysicsWorld {
public:
    void clear();

    // getters
    RigidBody* getRigidBody(BodyHandle handle);
    const RigidBody* getRigidBody(BodyHandle handle) const;

    Collider* getCollider(ColliderHandle handle);
    const Collider* getCollider(ColliderHandle handle) const;

    AABB computeBodyAABB(const RigidBody& body);

    SlotMap<Collider, ColliderHandle>& getCollidersMap();
    SlotMap<RigidBody, BodyHandle>& getRigidBodiesMap();

    const SlotMap<Collider, ColliderHandle>& getCollidersMap() const;
    const SlotMap<RigidBody, BodyHandle>& getRigidBodiesMap() const;

    // creation and deletion
    BodyHandle createPendingRigidBody();
    ColliderHandle createPendingCollider();

    bool activateRigidBody(BodyHandle handle);
    bool activateCollider(ColliderHandle handle);

    void discardPendingRigidBody(BodyHandle handle);
    void discardPendingCollider(ColliderHandle handle);

    void deleteRigidBody(BodyHandle handle);
    void deleteCollider(ColliderHandle handle);

    bool isRigidBodyActive(BodyHandle handle) const;
    bool isColliderActive(ColliderHandle handle) const;
    bool isRigidBodyPending(BodyHandle handle) const;
    bool isColliderPending(ColliderHandle handle) const;

private:
    int colliderId = 0;
    int rigidBodyId = 0;
    SlotMap<Collider, ColliderHandle> colliders;
    SlotMap<RigidBody, BodyHandle> rigidBodies;
    std::unordered_map<ColliderHandle, Collider> pendingColliders;
    std::unordered_map<BodyHandle, RigidBody> pendingRigidBodies;
};

}
