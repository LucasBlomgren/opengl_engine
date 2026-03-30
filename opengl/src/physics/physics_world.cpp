#include "pch.h"

#include "physics_world.h"

// getters
RigidBody* PhysicsWorld::getRigidBody(RigidBodyHandle& handle) {
    return m_rigidBodies.try_get(handle); 
}
Collider* PhysicsWorld::getCollider(ColliderHandle& handle) { 
    return m_colliders.try_get(handle); 
}
SlotMap<Collider, ColliderHandle>& PhysicsWorld::getCollidersMap() { 
    return m_colliders; 
}
SlotMap<RigidBody, RigidBodyHandle>& PhysicsWorld::getRigidBodiesMap() { 
    return m_rigidBodies; 
}

void PhysicsWorld::clear() {
    m_colliders = SlotMap<Collider, ColliderHandle>();
    m_rigidBodies = SlotMap<RigidBody, RigidBodyHandle>();
    colliderId = 0;
    rigidBodyId = 0;
}

RigidBodyHandle PhysicsWorld::createRigidBody() {
    RigidBodyHandle handle = m_rigidBodies.create();
    RigidBody* body = m_rigidBodies.try_get(handle);
    body->id = rigidBodyId++;
    return handle;
}

ColliderHandle PhysicsWorld::createCollider() {
    ColliderHandle handle = m_colliders.create();
    Collider* collider = m_colliders.try_get(handle);
    collider->id = colliderId++;
    return handle;
}

void PhysicsWorld::deleteRigidBody(RigidBodyHandle handle) {
    m_rigidBodies.destroy(handle);
}

void PhysicsWorld::deleteCollider(ColliderHandle handle) {
    m_colliders.destroy(handle);
}