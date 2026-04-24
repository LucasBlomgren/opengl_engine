#include "pch.h"

#include "physics_world.h"

//-------------------------------------------
// Clear all data
//-------------------------------------------
void PhysicsWorld::clear() {
    colliders.clear();
    rigidBodies.clear();
    colliderId = 0;
    rigidBodyId = 0;
}

//-------------------------------------------
// Getters
//-------------------------------------------
RigidBody* PhysicsWorld::getRigidBody(RigidBodyHandle& handle) {
    return rigidBodies.try_get(handle); 
}
Collider* PhysicsWorld::getCollider(ColliderHandle& handle) { 
    return colliders.try_get(handle); 
}
SlotMap<Collider, ColliderHandle>& PhysicsWorld::getCollidersMap() { 
    return colliders; 
}
SlotMap<RigidBody, RigidBodyHandle>& PhysicsWorld::getRigidBodiesMap() { 
    return rigidBodies; 
}

AABB PhysicsWorld::computeBodyAABB(const RigidBody& body) {
    if (body.colliderHandles.empty()) {
        std::cout << "[PhysicsWorld] Warning: RigidBody with id " << body.id << " has no colliders. Returning empty AABB." << std::endl;
        return AABB{};
    }

    if (body.colliderHandles.size() == 1) {
        return colliders.try_get(body.colliderHandles[0])->getAABB();
    }

    AABB merged = colliders.try_get(body.colliderHandles[0])->getAABB();
    for (size_t i = 1; i < body.colliderHandles.size(); ++i) {
        Collider* c = colliders.try_get(body.colliderHandles[i]);
        merged.growToInclude(c->getAABB().worldMin);
        merged.growToInclude(c->getAABB().worldMax);
    }
    return merged;
}

//-------------------------------------------
// Creation
//-------------------------------------------
RigidBodyHandle PhysicsWorld::createRigidBody() {
    RigidBodyHandle handle = rigidBodies.create();
    RigidBody* body = rigidBodies.try_get(handle);
    body->id = rigidBodyId++;
    return handle;
}

ColliderHandle PhysicsWorld::createCollider() {
    ColliderHandle handle = colliders.create();
    Collider* collider = colliders.try_get(handle);
    collider->id = colliderId++;
    return handle;
}

//-------------------------------------------
// Deletion
//-------------------------------------------
void PhysicsWorld::deleteRigidBody(RigidBodyHandle handle) {
    rigidBodies.destroy(handle);
}

void PhysicsWorld::deleteCollider(ColliderHandle handle) {
    colliders.destroy(handle);
}