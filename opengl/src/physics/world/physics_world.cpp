#include "pch.h"

#include "physics_world.h"

namespace physics::internal {

//===========================================
// Clear all data
//===========================================
void PhysicsWorld::clear() {
    colliders.clear();
    bodies.clear();
    motionStates.clear();
    sleepStates.clear();
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

MotionState& PhysicsWorld::getMotionState(MotionStateHandle handle) {
    return motionStates.get(handle);
}
const MotionState& PhysicsWorld::getMotionState(MotionStateHandle handle) const {
    return motionStates.get(handle);
}
SleepState& PhysicsWorld::getSleepState(SleepStateHandle handle) {
    return sleepStates.get(handle);
}
const SleepState& PhysicsWorld::getSleepState(SleepStateHandle handle) const {
    return sleepStates.get(handle);
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

SlotMap<MotionState, MotionStateHandle> PhysicsWorld::motionStatesStorage() {
    return motionStates;
}
const SlotMap<MotionState, MotionStateHandle> PhysicsWorld::motionStatesStorage() const {
    return motionStates;
}
SlotMap<SleepState, SleepStateHandle> PhysicsWorld::sleepStatesStorage() {
    return sleepStates;
}
const SlotMap<SleepState, SleepStateHandle> PhysicsWorld::sleepStatesStorage() const {
    return sleepStates;
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
MotionStateHandle PhysicsWorld::commitMotionState(
    MotionState&& motionState) {
    return motionStates.create(std::move(motionState));
}
SleepStateHandle PhysicsWorld::commitSleepState(
    SleepState&& sleepState) {
    return sleepStates.create(std::move(sleepState));
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
void PhysicsWorld::destroyMotionState(MotionStateHandle handle) {
    motionStates.destroy(handle);
}
void PhysicsWorld::destroySleepState(SleepStateHandle handle) {
    sleepStates.destroy(handle);
}
}