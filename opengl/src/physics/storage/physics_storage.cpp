#include "pch.h"

#include "physics_storage.h"

namespace physics::internal {

using BodyH = BodyHandle;
using CollH = ColliderHandle;
using MotionH = MotionStateHandle;
using SleepH = SleepStateHandle;

using BodyMap = SlotMap<RigidBody, BodyHandle>;
using MotionMap = SlotMap<MotionState, MotionStateHandle>;
using SleepMap = SlotMap<SleepState, SleepStateHandle>;
using ColliderMap = SlotMap<Collider, ColliderHandle>;

using PS = PhysicsStorage;

//===========================================
// Clear all data
//===========================================
void PS::clear() {
    colliders.clear();
    bodies.clear();
    motionStates.clear();
    sleepStates.clear();
    colliderId = 0;
    rigidBodyId = 0;
}

//===========================================
// Get data by handle
//===========================================
RigidBody& PS::getBody(BodyH h) { return bodies.get(h); }
RigidBody* PS::tryGetBody(BodyH h) { return bodies.try_get(h, FUNC_NAME); }
const RigidBody* PS::tryGetBody(BodyH h) const { return bodies.try_get(h, FUNC_NAME); }

MotionState& PS::getMotionState(MotionH h) { return motionStates.get(h); }
const MotionState& PS::getMotionState(MotionH h) const { return motionStates.get(h); }

SleepState& PS::getSleepState(SleepH h) { return sleepStates.get(h); }
const SleepState& PS::getSleepState(SleepH h) const { return sleepStates.get(h); }
SleepState* PS::tryGetSleepState(SleepH h) { return sleepStates.try_get(h, FUNC_NAME); }

Collider& PS::getCollider(CollH h) { return colliders.get(h); }
Collider* PS::tryGetCollider(CollH h) { return colliders.try_get(h, FUNC_NAME); }
const Collider* PS::tryGetCollider(CollH h) const { return colliders.try_get(h, FUNC_NAME); }

//===========================================
// Get storage
//===========================================
BodyMap& PS::bodyStorage() { return bodies; }
const BodyMap& PS::bodyStorage() const { return bodies; }

MotionMap PS::motionStatesStorage() { return motionStates; }
const MotionMap PS::motionStatesStorage() const { return motionStates; }

SleepMap PS::sleepStatesStorage() { return sleepStates; }
const SleepMap PS::sleepStatesStorage() const { return sleepStates; }

ColliderMap& PS::colliderStorage() { return colliders; }
const ColliderMap& PS::colliderStorage() const { return colliders; }

//===========================================
// Get AABB for a RigidBody
//===========================================
AABB PS::computeBodyAABB(const RigidBody& body) {
    if (body.colliderHandles.empty()) {
        std::cout
            << "[PS] Warning: RigidBody with id "
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
BodyHandle PS::reserveBodyHandle() {
    return bodies.reserve();
}
ColliderHandle PS::reserveColliderHandle() {
    return colliders.reserve();
}

void PS::releaseBodyReservation(BodyHandle handle) {
    bodies.release_reserved(handle);
}
void PS::releaseColliderReservation(ColliderHandle handle) {
    colliders.release_reserved(handle);
}


//===========================================
// Creation
//===========================================
RigidBody* PS::commitBody(BodyHandle h, RigidBody&& body) {
    body.id = rigidBodyId++;
    return bodies.create_reserved(h, std::move(body));
}
Collider* PS::commitCollider(ColliderHandle h, Collider&& collider) {
    collider.id = colliderId++;
    return colliders.create_reserved(h, std::move(collider));
}
MotionH PS::commitMotionState(MotionState&& motionState) {
    return motionStates.create(std::move(motionState));
}
SleepStateHandle PS::commitSleepState(SleepState&& sleepState) {
    return sleepStates.create(std::move(sleepState));
}

//===========================================
// Deletion
//===========================================
void PS::destroyBody(BodyHandle h) {
    bodies.destroy(h);
}
void PS::destroyCollider(ColliderHandle h) {
    colliders.destroy(h);
}
void PS::destroyMotionState(MotionH h) {
    motionStates.destroy(h);
}
void PS::destroySleepState(SleepStateHandle h) {
    sleepStates.destroy(h);
}
}