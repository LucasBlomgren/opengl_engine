#include "pch.h"

#include "physics_world.h"

namespace physics::internal {

//===========================================
// Clear all data
//===========================================
void PhysicsWorld::clear() {
    colliders.clear();
    bodies.clear();
    colliderId = 0;
    rigidBodyId = 0;
}

//===========================================
// Getters
//===========================================
RigidBody& PhysicsWorld::getBody(BodyHandle handle) {
    return bodies.get(handle);
}
RigidBody* PhysicsWorld::tryGetBody(BodyHandle handle) {
    return bodies.try_get(handle, FUNC_NAME);
}
const RigidBody* PhysicsWorld::tryGetBody(BodyHandle handle) const {
    return bodies.try_get(handle, FUNC_NAME);
}

Collider& PhysicsWorld::getCollider(ColliderHandle handle) {
    return colliders.get(handle);
}
Collider* PhysicsWorld::tryGetCollider(ColliderHandle handle) {
    return colliders.try_get(handle, FUNC_NAME);
}
const Collider* PhysicsWorld::tryGetCollider(ColliderHandle handle) const {
    return colliders.try_get(handle, FUNC_NAME);
}

SlotMap<RigidBody, BodyHandle>& PhysicsWorld::bodyStorage() {
    return bodies;
}
const SlotMap<RigidBody, BodyHandle>& PhysicsWorld::bodyStorage() const {
    return bodies;
}

SlotMap<Collider, ColliderHandle>& PhysicsWorld::colliderStorage() {
    return colliders;
}
const SlotMap<Collider, ColliderHandle>& PhysicsWorld::colliderStorage() const {
    return colliders;
}

AABB PhysicsWorld::computeBodyAABB(const RigidBody& body) {
    if (body.colliderHandles.empty()) {
        std::cout
            << "[PhysicsWorld] Warning: RigidBody with id "
            << body.id
            << " has no colliders. Returning empty AABB."
            << std::endl;
        return AABB{};
    }

    if (body.colliderHandles.size() == 1) {
        Collider& collider = getCollider(body.colliderHandles[0]);
        return collider.getAABB();
    }

    Collider& first = getCollider(body.colliderHandles[0]);

    AABB merged = first.getAABB();
    for (size_t i = 1; i < body.colliderHandles.size(); ++i) {
        Collider* c = tryGetCollider(body.colliderHandles[i]);
        if (!c) {
            continue;
        }
        merged.growToInclude(c->getAABB().worldMin);
        merged.growToInclude(c->getAABB().worldMax);
    }
    return merged;
}

//===========================================
// Reservation
//===========================================
BodyHandle PhysicsWorld::reserveBodyHandle() {
    return bodies.reserve();
}
ColliderHandle PhysicsWorld::reserveColliderHandle() {
    return colliders.reserve();
}

void PhysicsWorld::releaseBodyReservation(BodyHandle handle) {
    bodies.release_reserved(handle);
}
void PhysicsWorld::releaseColliderReservation(ColliderHandle handle) {
    colliders.release_reserved(handle);
}


//===========================================
// Creation
//===========================================
RigidBody* PhysicsWorld::commitBody(
    BodyHandle handle,
    RigidBody&& body) {
    body.id = rigidBodyId++;
    return bodies.create_reserved(handle, std::move(body));
}
Collider* PhysicsWorld::commitCollider(
    ColliderHandle handle,
    Collider&& collider) {
    collider.id = colliderId++;
    return colliders.create_reserved(handle, std::move(collider));
}

//===========================================
// Deletion
//===========================================
void PhysicsWorld::destroyBody(BodyHandle handle) {
    bodies.destroy(handle);
}
void PhysicsWorld::destroyCollider(ColliderHandle handle) {
    colliders.destroy(handle);
}
}
