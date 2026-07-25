#include "pch.h"

#include "physics/engine/commands/physics_external_command_buffer.h"

#include <algorithm>
#include <iterator>
#include <utility>

namespace {
    template<class T>
    bool contains(const std::vector<T>& values, const T& value) {
        return std::find(values.begin(), values.end(), value) != values.end();
    }

    template<class T>
    bool addUnique(std::vector<T>& values, const T& value) {
        if (contains(values, value)) {
            return false;
        }

        values.push_back(value);
        return true;
    }

    template<class T>
    void moveAndClear(std::vector<T>& source, std::vector<T>& destination) {
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
void PhysicsExternalCommandBuffer::reserve(size_t bodyCount, size_t colliderCount, size_t mutationCount) {
    bodyCreates.reserve(bodyCount);
    bodyDestroys.reserve(bodyCount);

    colliderCreates.reserve(colliderCount);
    colliderDestroys.reserve(colliderCount);

    mutations.reserve(mutationCount);
}

void PhysicsExternalCommandBuffer::clear() {
    bodyCreates.clear();
    colliderCreates.clear();
    bodyDestroys.clear();
    colliderDestroys.clear();
    mutations.clear();
}

bool PhysicsExternalCommandBuffer::empty() const noexcept {
    return bodyCreates.empty() &&
        colliderCreates.empty() &&
        bodyDestroys.empty() &&
        colliderDestroys.empty() &&
        mutations.empty();
}

PhysicsExternalCommandBuffer::Batch PhysicsExternalCommandBuffer::take() {
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
void PhysicsExternalCommandBuffer::recordBodyCreate(RigidBodyHandle body) {
    bodyCreates.push_back(body);
}

void PhysicsExternalCommandBuffer::recordColliderCreate(ColliderHandle collider) {
    colliderCreates.push_back(collider);
}

bool PhysicsExternalCommandBuffer::recordBodyDestroy(RigidBodyHandle body) {
    return addUnique(bodyDestroys, body);
}

bool PhysicsExternalCommandBuffer::recordColliderDestroy(ColliderHandle collider) {
    return addUnique(colliderDestroys, collider);
}

bool PhysicsExternalCommandBuffer::isBodyPendingDestroy(RigidBodyHandle body) const {
    return contains(bodyDestroys, body);
}

bool PhysicsExternalCommandBuffer::isColliderPendingDestroy(ColliderHandle collider) const {
    return contains(colliderDestroys, collider);
}

//=========================================
// Rigid body command recording
//=========================================
void PhysicsExternalCommandBuffer::queueApplyLinearImpulse(RigidBodyHandle body, const glm::vec3& impulse) {
    mutations.emplace_back(ApplyLinearImpulse{ body, impulse });
}

void PhysicsExternalCommandBuffer::queueSetLinearVelocity(RigidBodyHandle body, const glm::vec3& velocity) {
    mutations.emplace_back(SetLinearVelocity{ body, velocity });
}

void PhysicsExternalCommandBuffer::queueSetAngularVelocity(RigidBodyHandle body, const glm::vec3& velocity) {
    mutations.emplace_back(SetAngularVelocity{ body, velocity });
}

void PhysicsExternalCommandBuffer::queueSetKinematicTarget(RigidBodyHandle body, const PhysicsPose& target) {
    mutations.emplace_back(SetKinematicTarget{ body, target });
}

void PhysicsExternalCommandBuffer::queueSetRigidBodyAwake(RigidBodyHandle body) {
    mutations.emplace_back(SetRigidBodyAwake{ body });
}

void PhysicsExternalCommandBuffer::queueSetRigidBodyAsleep(RigidBodyHandle body) {
    mutations.emplace_back(SetRigidBodyAsleep{ body });
}

void PhysicsExternalCommandBuffer::queueSetRigidBodyType(RigidBodyHandle body, BodyType type) {
    mutations.emplace_back(SetRigidBodyType{ body, type });
}

void PhysicsExternalCommandBuffer::queueSetRigidBodyMotionControl(RigidBodyHandle body, MotionControl motionControl) {
    mutations.emplace_back(SetRigidBodyMotionControl{ body, motionControl });
}

//=========================================
// Collider command recording
//=========================================
void PhysicsExternalCommandBuffer::queueSetColliderLocalPose(ColliderHandle collider, const PhysicsPose& localPose) {
    mutations.emplace_back(SetColliderLocalPose{ collider, localPose });
}

void PhysicsExternalCommandBuffer::queueSetColliderEnabled(ColliderHandle collider, bool enabled) {
    mutations.emplace_back(SetColliderEnabled{ collider, enabled });
}

void PhysicsExternalCommandBuffer::queueSetColliderTrigger(ColliderHandle collider, bool isTrigger) {
    mutations.emplace_back(SetColliderTrigger{ collider, isTrigger });
}

//=========================================
// Scene-wide command recording
//=========================================
void PhysicsExternalCommandBuffer::queueSleepAllObjects() {
    mutations.emplace_back(SleepAllObjects{});
}

void PhysicsExternalCommandBuffer::queueAwakenAllObjects() {
    mutations.emplace_back(AwakenAllObjects{});
}

void PhysicsExternalCommandBuffer::queueSyncBodyFromTransform(RigidBodyHandle body) {
    mutations.emplace_back(SyncBodyFromTransform{ body });
}