#include "pch.h"

#include "physics_world.h"

void PhysicsWorld::clear() {
    m_colliders = SlotMap<Collider, ColliderHandle>();
    m_rigidBodies = SlotMap<RigidBody, RigidBodyHandle>();
    colliderId = 0;
    rigidBodyId = 0;
}

RigidBodyHandle PhysicsWorld::createRigidBody() {
    return m_rigidBodies.create();
}

ColliderHandle PhysicsWorld::createCollider() {
    GameObject* owner = nullptr;
    return m_colliders.create(owner);
}

void PhysicsWorld::deleteRigidBody(RigidBodyHandle handle) {
    m_rigidBodies.destroy(handle);
}

void PhysicsWorld::deleteCollider(ColliderHandle handle) {
    m_colliders.destroy(handle);
}