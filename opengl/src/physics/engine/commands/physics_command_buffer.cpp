#include "pch.h"

#include "physics/engine/commands/physics_command_buffer.h"

#include <algorithm>
#include <iterator>
#include <utility>

namespace {
    template<class T>
    bool contains(
        const std::vector<T>& values, 
        const T& value) {
        return std::find(
            values.begin(), 
            values.end(), 
            value) 
            != values.end();
    }

    template<class T>
    bool addUnique(
        std::vector<T>& values, 
        const T& value) 
    {
        if (contains(values, value)) {
            return false;
        }

        values.push_back(value);
        return true;
    }

    template<class T>
    void moveAndClear(
        std::vector<T>& source, 
        std::vector<T>& destination) 
    {
        destination.reserve(source.size());

        std::move(
            source.begin(),
            source.end(),
            std::back_inserter(destination)
        );

        source.clear();
    }
}

//=========================================
// Storage management
//=========================================
void PhysicsCommandBuffer::reserve(
    size_t bodyCount, 
    size_t colliderCount, 
    size_t mutationCount) 
{
    bodyCreates.reserve(bodyCount);
    bodyDestroys.reserve(bodyCount);

    colliderCreates.reserve(colliderCount);
    colliderDestroys.reserve(colliderCount);

    mutations.reserve(mutationCount);
}

void PhysicsCommandBuffer::clear() {
    bodyCreates.clear();
    colliderCreates.clear();
    bodyDestroys.clear();
    colliderDestroys.clear();
    mutations.clear();
}

bool PhysicsCommandBuffer::empty() const noexcept {
    return bodyCreates.empty() &&
        colliderCreates.empty() &&
        bodyDestroys.empty() &&
        colliderDestroys.empty() &&
        mutations.empty();
}

PhysicsCommandBuffer::Batch PhysicsCommandBuffer::take() {
    Batch batch;

    moveAndClear(bodyCreates, batch.bodyCreates);
    moveAndClear(colliderCreates, batch.colliderCreates);
    moveAndClear(bodyDestroys, batch.bodyDestroys);
    moveAndClear(colliderDestroys, batch.colliderDestroys);
    moveAndClear(mutations, batch.mutations);

    return batch;
}

//=========================================
// Lifecycle recording
//=========================================
void PhysicsCommandBuffer::recordBodyCreate(
    RigidBodyHandle body) {
    bodyCreates.push_back(body);
}

void PhysicsCommandBuffer::recordColliderCreate(
    ColliderHandle collider) {
    colliderCreates.push_back(collider);
}

bool PhysicsCommandBuffer::recordBodyDestroy(
    RigidBodyHandle body) {
    return addUnique(bodyDestroys, body);
}

bool PhysicsCommandBuffer::recordColliderDestroy(
    ColliderHandle collider) {
    return addUnique(colliderDestroys, collider);
}

bool PhysicsCommandBuffer::isBodyPendingDestroy(
    RigidBodyHandle body) const {
    return contains(bodyDestroys, body);
}

bool PhysicsCommandBuffer::isColliderPendingDestroy(
    ColliderHandle collider) const {
    return contains(colliderDestroys, collider);
}

//=========================================
// Rigid body command recording
//=========================================
void PhysicsCommandBuffer::recordApplyLinearImpulse(
    RigidBodyHandle body, 
    const glm::vec3& impulse) {
    mutations.emplace_back(ApplyLinearImpulse{ body, impulse });
}

void PhysicsCommandBuffer::recordSetLinearVelocity(
    RigidBodyHandle body, 
    const glm::vec3& velocity) {
    mutations.emplace_back(SetLinearVelocity{ body, velocity });
}

void PhysicsCommandBuffer::recordSetAngularVelocity(
    RigidBodyHandle body, 
    const glm::vec3& velocity) {
    mutations.emplace_back(SetAngularVelocity{ body, velocity });
}

void PhysicsCommandBuffer::recordSetKinematicTarget(
    RigidBodyHandle body, 
    const PhysicsPose& target) {
    mutations.emplace_back(SetKinematicTarget{ body, target });
}

void PhysicsCommandBuffer::recordSetRigidBodySleepState(
    RigidBodyHandle body,
    bool asleep) {
    mutations.emplace_back(SetRigidBodySleepState{ body, asleep });
}

void PhysicsCommandBuffer::recordSetRigidBodyType(
    RigidBodyHandle body, 
    BodyType type) {
    mutations.emplace_back(SetRigidBodyType{ body, type });
}

void PhysicsCommandBuffer::recordSetRigidBodyMotionControl(
    RigidBodyHandle body, 
    MotionControl motionControl) {
    mutations.emplace_back(SetRigidBodyMotionControl{ body, motionControl });
}

//=========================================
// Collider command recording
//=========================================
void PhysicsCommandBuffer::recordSetColliderLocalPose(
    ColliderHandle collider, 
    const PhysicsPose& localPose) {
    mutations.emplace_back(SetColliderLocalPose{ collider, localPose });
}

void PhysicsCommandBuffer::recordSetColliderEnabled(
    ColliderHandle collider, 
    bool enabled) {
    mutations.emplace_back(SetColliderEnabled{ collider, enabled });
}

void PhysicsCommandBuffer::recordSetColliderTrigger(
    ColliderHandle collider, 
    bool isTrigger) {
    mutations.emplace_back(SetColliderTrigger{ collider, isTrigger });
}

//=========================================
// Scene-wide command recording
//=========================================
void PhysicsCommandBuffer::recordSleepAllObjects() {
    mutations.emplace_back(SleepAllObjects{});
}

void PhysicsCommandBuffer::recordAwakenAllObjects() {
    mutations.emplace_back(AwakenAllObjects{});
}

void PhysicsCommandBuffer::recordSyncBodyFromTransform(
    RigidBodyHandle body) {
    mutations.emplace_back(SyncBodyFromTransform{ body });
}
