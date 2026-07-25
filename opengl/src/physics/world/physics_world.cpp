#include "pch.h"

#include "physics_world.h"

//===========================================
// Clear all data
//===========================================
void PhysicsWorld::clear() {
    pendingColliders.clear();
    pendingRigidBodies.clear();
    colliders.clear();
    rigidBodies.clear();
    colliderId = 0;
    rigidBodyId = 0;
}

//===========================================
// Getters
//===========================================
RigidBody* PhysicsWorld::getRigidBody(RigidBodyHandle handle) {
    if (RigidBody* body = rigidBodies.try_get(handle)) {
        return body;
    }

    auto pending = pendingRigidBodies.find(handle);
    return pending != pendingRigidBodies.end() ? &pending->second : nullptr;
}

const RigidBody* PhysicsWorld::getRigidBody(RigidBodyHandle handle) const {
    if (const RigidBody* body = rigidBodies.try_get(handle)) {
        return body;
    }

    auto pending = pendingRigidBodies.find(handle);
    return pending != pendingRigidBodies.end() ? &pending->second : nullptr;
}

Collider* PhysicsWorld::getCollider(ColliderHandle handle) {
    if (Collider* collider = colliders.try_get(handle)) {
        return collider;
    }

    auto pending = pendingColliders.find(handle);
    return pending != pendingColliders.end() ? &pending->second : nullptr;
}

const Collider* PhysicsWorld::getCollider(ColliderHandle handle) const {
    if (const Collider* collider = colliders.try_get(handle)) {
        return collider;
    }

    auto pending = pendingColliders.find(handle);
    return pending != pendingColliders.end() ? &pending->second : nullptr;
}

SlotMap<Collider, ColliderHandle>& PhysicsWorld::getCollidersMap() { 
    return colliders; 
}
SlotMap<RigidBody, RigidBodyHandle>& PhysicsWorld::getRigidBodiesMap() { 
    return rigidBodies; 
}
const SlotMap<Collider, ColliderHandle>& PhysicsWorld::getCollidersMap() const {
    return colliders;
}

const SlotMap<RigidBody, RigidBodyHandle>& PhysicsWorld::getRigidBodiesMap() const {
    return rigidBodies;
}

AABB PhysicsWorld::computeBodyAABB(const RigidBody& body) {
    if (body.colliderHandles.empty()) {
        std::cout << "[PhysicsWorld] Warning: RigidBody with id " << body.id << " has no colliders. Returning empty AABB." << std::endl;
        return AABB{};
    }

    if (body.colliderHandles.size() == 1) {
        Collider* collider = getCollider(body.colliderHandles[0]);
        return collider ? collider->getAABB() : AABB{};
    }

    Collider* first = getCollider(body.colliderHandles[0]);
    if (!first) {
        return AABB{};
    }

    AABB merged = first->getAABB();
    for (size_t i = 1; i < body.colliderHandles.size(); ++i) {
        Collider* c = getCollider(body.colliderHandles[i]);
        if (!c) {
            continue;
        }
        merged.growToInclude(c->getAABB().worldMin);
        merged.growToInclude(c->getAABB().worldMax);
    }
    return merged;
}

//===========================================
// Creation
//===========================================
RigidBodyHandle PhysicsWorld::createRigidBody() {
    RigidBodyHandle handle = rigidBodies.reserve();
    RigidBody& body = pendingRigidBodies[handle];
    body.id = rigidBodyId++;
    return handle;
}

ColliderHandle PhysicsWorld::createCollider() {
    ColliderHandle handle = colliders.reserve();
    Collider& collider = pendingColliders[handle];
    collider.id = colliderId++;
    return handle;
}

bool PhysicsWorld::activateRigidBody(RigidBodyHandle handle) {
    auto pending = pendingRigidBodies.find(handle);
    if (pending == pendingRigidBodies.end()) {
        return false;
    }

    RigidBody* body = rigidBodies.create_reserved(handle, std::move(pending->second));
    pendingRigidBodies.erase(pending);
    return body != nullptr;
}

bool PhysicsWorld::activateCollider(ColliderHandle handle) {
    auto pending = pendingColliders.find(handle);
    if (pending == pendingColliders.end()) {
        return false;
    }

    Collider* collider = colliders.create_reserved(handle, std::move(pending->second));
    pendingColliders.erase(pending);
    return collider != nullptr;
}

void PhysicsWorld::discardPendingRigidBody(RigidBodyHandle handle) {
    if (pendingRigidBodies.erase(handle) > 0) {
        rigidBodies.release_reserved(handle);
    }
}

void PhysicsWorld::discardPendingCollider(ColliderHandle handle) {
    if (pendingColliders.erase(handle) > 0) {
        colliders.release_reserved(handle);
    }
}

//===========================================
// Deletion
//===========================================
void PhysicsWorld::deleteRigidBody(RigidBodyHandle handle) {
    rigidBodies.destroy(handle);
}

void PhysicsWorld::deleteCollider(ColliderHandle handle) {
    colliders.destroy(handle);
}

bool PhysicsWorld::isRigidBodyActive(RigidBodyHandle handle) const {
    return rigidBodies.alive(handle);
}

bool PhysicsWorld::isColliderActive(ColliderHandle handle) const {
    return colliders.alive(handle);
}

bool PhysicsWorld::isRigidBodyPending(RigidBodyHandle handle) const {
    return pendingRigidBodies.contains(handle);
}

bool PhysicsWorld::isColliderPending(ColliderHandle handle) const {
    return pendingColliders.contains(handle);
}
