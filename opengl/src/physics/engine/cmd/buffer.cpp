#include "pch.h"

#include "physics/engine/cmd/buffer.h"

#include <algorithm>
#include <iterator>
#include <utility>

namespace physics::internal::cmd {

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
void Buffer::reserve(
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

void Buffer::clear() {
    bodyCreates.clear();
    colliderCreates.clear();
    bodyDestroys.clear();
    colliderDestroys.clear();
    mutations.clear();
}

bool Buffer::empty() const noexcept {
    return bodyCreates.empty() &&
        colliderCreates.empty() &&
        bodyDestroys.empty() &&
        colliderDestroys.empty() &&
        mutations.empty();
}

Buffer::Batch Buffer::take() {
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
void Buffer::recordBodyCreate(
    BodyHandle body) {
    bodyCreates.push_back(body);
}

void Buffer::recordColliderCreate(
    ColliderHandle collider) {
    colliderCreates.push_back(collider);
}

bool Buffer::recordBodyDestroy(
    BodyHandle body) {
    return addUnique(bodyDestroys, body);
}

bool Buffer::recordColliderDestroy(
    ColliderHandle collider) {
    return addUnique(colliderDestroys, collider);
}

bool Buffer::isBodyPendingDestroy(
    BodyHandle body) const {
    return contains(bodyDestroys, body);
}

bool Buffer::isColliderPendingDestroy(
    ColliderHandle collider) const {
    return contains(colliderDestroys, collider);
}

//=========================================
// Rigid body command recording
//=========================================
void Buffer::recordApplyLinearImpulse(
    BodyHandle body,
    const glm::vec3& impulse) {
    mutations.emplace_back(ApplyLinearImpulse{ body, impulse });
}

void Buffer::recordSetLinearVelocity(
    BodyHandle body,
    const glm::vec3& velocity) {
    mutations.emplace_back(SetLinearVelocity{ body, velocity });
}

void Buffer::recordSetAngularVelocity(
    BodyHandle body,
    const glm::vec3& velocity) {
    mutations.emplace_back(SetAngularVelocity{ body, velocity });
}

void Buffer::recordSetKinematicTarget(
    BodyHandle body,
    const Pose& target) {
    mutations.emplace_back(SetKinematicTarget{ body, target });
}

void Buffer::recordSetRigidBodyTransform(
    BodyHandle body,
    const Pose& pose,
    const glm::vec3& scale) {
    mutations.emplace_back(SetRigidBodyTransform{ body, pose, scale });
}

void Buffer::recordSetRigidBodySleepState(
    BodyHandle body,
    bool asleep) {
    mutations.emplace_back(SetRigidBodySleepState{ body, asleep });
}

void Buffer::recordSetRigidBodyType(
    BodyHandle body,
    BodyType type) {
    mutations.emplace_back(SetRigidBodyType{ body, type });
}

void Buffer::recordSetRigidBodyMotionControl(
    BodyHandle body,
    MotionControl motionControl) {
    mutations.emplace_back(SetRigidBodyMotionControl{ body, motionControl });
}

void Buffer::recordSetRigidBodyResponseMode(
    BodyHandle body,
    ResponseMode responseMode) {
    mutations.emplace_back(SetRigidBodyResponseMode{ body, responseMode });
}

void Buffer::recordSetRigidBodyMass(
    BodyHandle body,
    float mass) {
    mutations.emplace_back(SetRigidBodyMass{ body, mass });
}

void Buffer::recordSetRigidBodyAllowGravity(
    BodyHandle body,
    bool allowGravity) {
    mutations.emplace_back(SetRigidBodyAllowGravity{ body, allowGravity });
}

void Buffer::recordSetRigidBodyAllowSleep(
    BodyHandle body,
    bool allowSleep) {
    mutations.emplace_back(SetRigidBodyAllowSleep{ body, allowSleep });
}

void Buffer::recordSetRigidBodyCanMoveLinearly(
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
void Buffer::recordSetColliderLocalPose(
    ColliderHandle collider,
    const Pose& localPose) {
    mutations.emplace_back(SetColliderLocalPose{ collider, localPose });
}

void Buffer::recordSetColliderLocalTransform(
    ColliderHandle collider,
    const Pose& localPose,
    const glm::vec3& localScale) {
    mutations.emplace_back(SetColliderLocalTransform{
        collider,
        localPose,
        localScale
    });
}

void Buffer::recordSetColliderShape(
    ColliderHandle collider,
    const ColliderShapeDesc& shape) {
    mutations.emplace_back(SetColliderShape{ collider, shape });
}

void Buffer::recordSetColliderEnabled(
    ColliderHandle collider,
    bool enabled) {
    mutations.emplace_back(SetColliderEnabled{ collider, enabled });
}

void Buffer::recordSetColliderTrigger(
    ColliderHandle collider,
    bool isTrigger) {
    mutations.emplace_back(SetColliderTrigger{ collider, isTrigger });
}

//=========================================
// Scene-wide command recording
//=========================================
void Buffer::recordSleepAllObjects() {
    mutations.emplace_back(SleepAllObjects{});
}

void Buffer::recordAwakenAllObjects() {
    mutations.emplace_back(AwakenAllObjects{});
}

}
