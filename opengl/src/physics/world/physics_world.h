#pragma once

#include <unordered_map>

#include "core/slot_map.h"

#include "physics/public/handles.h"
#include "physics/bodies/rigidbody.h"
#include "physics/bodies/motion_state.h"
#include "physics/sleep/sleep_state.h"
#include "physics/colliders/collider.h"

namespace physics::internal {

class PhysicsWorld {
public:
    void clear();

    // Handle reservation
    BodyHandle reserveBodyHandle();
    ColliderHandle reserveColliderHandle();

    void releaseBodyReservation(BodyHandle handle);
    void releaseColliderReservation(ColliderHandle handle);

    // Commit creation
    RigidBody* commitBody(
        BodyHandle handle,
        RigidBody&& body);

    Collider* commitCollider(
        ColliderHandle handle,
        Collider&& collider);

    MotionStateHandle commitMotionState(
        MotionState&& motionState);

    SleepStateHandle commitSleepState(
        SleepState&& sleepState);

    // Active deletion
    void destroyBody(BodyHandle handle);
    void destroyMotionState(MotionStateHandle handle);
    void destroySleepState(SleepStateHandle handle);
    void destroyCollider(ColliderHandle handle);

    // Active lookup only
    RigidBody& getBody(BodyHandle handle);
    RigidBody* tryGetBody(BodyHandle handle);
    const RigidBody* tryGetBody(BodyHandle handle) const;

    MotionState& getMotionState(MotionStateHandle handle);
    const MotionState& getMotionState(MotionStateHandle handle) const;
    SleepState& getSleepState(SleepStateHandle handle);
    const SleepState& getSleepState(SleepStateHandle handle) const;

    Collider& getCollider(ColliderHandle handle);
    Collider* tryGetCollider(ColliderHandle handle);
    const Collider* tryGetCollider(ColliderHandle handle) const;

    // Needed by caches, iteration and queries
    SlotMap<RigidBody, BodyHandle>& bodyStorage();
    const SlotMap<RigidBody, BodyHandle>& bodyStorage() const;

    SlotMap<MotionState, MotionStateHandle> motionStatesStorage();
    const SlotMap<MotionState, MotionStateHandle> motionStatesStorage() const;

    SlotMap<SleepState, SleepStateHandle> sleepStatesStorage();
    const SlotMap<SleepState, SleepStateHandle> sleepStatesStorage() const;

    SlotMap<Collider, ColliderHandle>& colliderStorage();
    const SlotMap<Collider, ColliderHandle>& colliderStorage() const;

    AABB computeBodyAABB(const RigidBody& body);

private:
    int colliderId = 0;
    int rigidBodyId = 0;
    SlotMap<RigidBody, BodyHandle> bodies{ "RigidBody" };
    SlotMap<MotionState, MotionStateHandle> motionStates{ "MotionState" };
    SlotMap<SleepState, SleepStateHandle> sleepStates{ "SleepState" };
    SlotMap<Collider, ColliderHandle> colliders{ "Collider" };
};
}