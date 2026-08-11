#include "pch.h"

#include "physics/engine/cmd_processor/cmd_buffer.h"

#include <algorithm>
#include <iterator>
#include <utility>

namespace physics::internal {

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
void CommandBuffer::reserve(
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

void CommandBuffer::clear() {
    bodyCreates.clear();
    colliderCreates.clear();
    bodyDestroys.clear();
    colliderDestroys.clear();
    mutations.clear();
}

bool CommandBuffer::empty() const noexcept {
    return bodyCreates.empty() &&
        colliderCreates.empty() &&
        bodyDestroys.empty() &&
        colliderDestroys.empty() &&
        mutations.empty();
}

CommandBuffer::Batch CommandBuffer::take() {
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
void CommandBuffer::recordBodyCreate(
    BodyHandle body) {
    bodyCreates.push_back(body);
}

void CommandBuffer::recordColliderCreate(
    ColliderHandle collider) {
    colliderCreates.push_back(collider);
}

bool CommandBuffer::recordBodyDestroy(
    BodyHandle body) {
    return addUnique(bodyDestroys, body);
}

bool CommandBuffer::recordColliderDestroy(
    ColliderHandle collider) {
    return addUnique(colliderDestroys, collider);
}

bool CommandBuffer::isBodyPendingDestroy(
    BodyHandle body) const {
    return contains(bodyDestroys, body);
}

bool CommandBuffer::isColliderPendingDestroy(
    ColliderHandle collider) const {
    return contains(colliderDestroys, collider);
}

//=========================================
// Rigid body command recording
//=========================================
void CommandBuffer::recordApplyLinearImpulse(
    BodyHandle body,
    const glm::vec3& impulse) {
    mutations.emplace_back(ApplyLinearImpulse{ body, impulse });
}

void CommandBuffer::recordSetLinearVelocity(
    BodyHandle body,
    const glm::vec3& velocity) {
    mutations.emplace_back(SetLinearVelocity{ body, velocity });
}

void CommandBuffer::recordSetAngularVelocity(
    BodyHandle body,
    const glm::vec3& velocity) {
    mutations.emplace_back(SetAngularVelocity{ body, velocity });
}

void CommandBuffer::recordSetKinematicTarget(
    BodyHandle body,
    const Pose& target) {
    mutations.emplace_back(SetKinematicTarget{ body, target });
}

void CommandBuffer::recordSetRigidBodyTransform(
    BodyHandle body,
    const Pose& pose,
    const glm::vec3& scale) {
    mutations.emplace_back(SetRigidBodyTransform{ body, pose, scale });
}

void CommandBuffer::recordSetRigidBodySleepState(
    BodyHandle body,
    bool asleep) {
    mutations.emplace_back(SetRigidBodySleepState{ body, asleep });
}

void CommandBuffer::recordSetRigidBodyType(
    BodyHandle body,
    BodyType type) {
    mutations.emplace_back(SetRigidBodyType{ body, type });
}

void CommandBuffer::recordSetRigidBodyMotionControl(
    BodyHandle body,
    MotionControl motionControl) {
    mutations.emplace_back(SetRigidBodyMotionControl{ body, motionControl });
}

void CommandBuffer::recordSetRigidBodyResponseMode(
    BodyHandle body,
    ResponseMode responseMode) {
    mutations.emplace_back(SetRigidBodyResponseMode{ body, responseMode });
}

void CommandBuffer::recordSetRigidBodyMass(
    BodyHandle body,
    float mass) {
    mutations.emplace_back(SetRigidBodyMass{ body, mass });
}

void CommandBuffer::recordSetRigidBodyAllowGravity(
    BodyHandle body,
    bool allowGravity) {
    mutations.emplace_back(SetRigidBodyAllowGravity{ body, allowGravity });
}

void CommandBuffer::recordSetRigidBodyAllowSleep(
    BodyHandle body,
    bool allowSleep) {
    mutations.emplace_back(SetRigidBodyAllowSleep{ body, allowSleep });
}

void CommandBuffer::recordSetRigidBodyCanMoveLinearly(
    BodyHandle body,
    bool canMoveLinearly) {
    mutations.emplace_back(SetRigidBodyCanMoveLinearly{
        body,
        canMoveLinearly
    });
}

//=========================================
// Collider command recording
//=========================================
void CommandBuffer::recordSetColliderLocalPose(
    ColliderHandle collider,
    const Pose& localPose) {
    mutations.emplace_back(SetColliderLocalPose{ collider, localPose });
}

void CommandBuffer::recordSetColliderLocalTransform(
    ColliderHandle collider,
    const Pose& localPose,
    const glm::vec3& localScale) {
    mutations.emplace_back(SetColliderLocalTransform{
        collider,
        localPose,
        localScale
    });
}

void CommandBuffer::recordSetColliderShape(
    ColliderHandle collider,
    const ColliderShapeDesc& shape) {
    mutations.emplace_back(SetColliderShape{ collider, shape });
}

void CommandBuffer::recordSetColliderEnabled(
    ColliderHandle collider,
    bool enabled) {
    mutations.emplace_back(SetColliderEnabled{ collider, enabled });
}

void CommandBuffer::recordSetColliderTrigger(
    ColliderHandle collider,
    bool isTrigger) {
    mutations.emplace_back(SetColliderTrigger{ collider, isTrigger });
}

//=========================================
// Scene-wide command recording
//=========================================
void CommandBuffer::recordSleepAllObjects() {
    mutations.emplace_back(SleepAllObjects{});
}

void CommandBuffer::recordAwakenAllObjects() {
    mutations.emplace_back(AwakenAllObjects{});
}

}
