#pragma once

#include "core/slot_map.h"
#include "game/game_object.h"
#include "physics/rigid_body.h"
#include "physics/colliders/collider.h"

class PhysicsEngine;

class PhysicsWorld {
public:
    void clear();

    // getters
    RigidBody* getRigidBody(RigidBodyHandle& handle);
    Collider* getCollider(ColliderHandle& handle);
    AABB computeBodyAABB(const RigidBody& body);

    SlotMap<Collider, ColliderHandle>& getCollidersMap();
    SlotMap<RigidBody, RigidBodyHandle>& getRigidBodiesMap();

    // creation and deletion
    RigidBodyHandle createRigidBody();
    ColliderHandle createCollider();

    void deleteRigidBody(RigidBodyHandle handle);
    void deleteCollider(ColliderHandle handle);

private:
    int colliderId = 0;
    int rigidBodyId = 0;
    SlotMap<Collider, ColliderHandle> colliders;
    SlotMap<RigidBody, RigidBodyHandle> rigidBodies;
};