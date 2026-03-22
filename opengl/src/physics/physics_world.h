#pragma once

#include "core/slot_map.h"
#include "game/game_object.h"
#include "physics/rigid_body.h"
#include "physics/colliders/collider.h"

class PhysicsEngine;

class PhysicsWorld {
public:
    void clear();

    SlotMap<Collider, ColliderHandle>& getColliders() { return m_colliders; }
    SlotMap<RigidBody, RigidBodyHandle>& getRigidBodies() { return m_rigidBodies; }

    RigidBodyHandle createRigidBody();
    ColliderHandle createCollider();

    void deleteRigidBody(RigidBodyHandle handle);
    void deleteCollider(ColliderHandle handle);

private:
    int colliderId = 0;
    int rigidBodyId = 0;
    SlotMap<Collider, ColliderHandle> m_colliders;
    SlotMap<RigidBody, RigidBodyHandle> m_rigidBodies;
};