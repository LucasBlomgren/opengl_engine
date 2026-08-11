#include "pch.h"
#include "physics/physics_engine.h"

namespace physics {

using namespace internal;

//=========================================
// Rigid body creation and destruction
//=========================================
BodyHandle Engine::createRigidBody(
    const BodyDesc& desc)
{
    if (desc.type == BodyType::Dynamic &&
        desc.mass <= 0.0f) {
        return {};
    }

    BodyHandle bodyHandle =
        physicsWorld.createPendingRigidBody();

    RigidBody* body =
        physicsWorld.getRigidBody(bodyHandle);

    if (!body) {
        return {};
    }

    commandBuffer.recordBodyCreate(bodyHandle);

    body->pose = desc.pose;
    body->scale = desc.scale;
    body->type = desc.type;
    body->motionControl = desc.motionControl;
    body->responseMode = desc.responseMode;
    body->linearVelocity = desc.linearVelocity;
    body->angularVelocity = desc.angularVelocity;
    body->allowGravity = desc.allowGravity;
    body->allowSleep = desc.allowSleep;
    body->canMoveLinearly = desc.canMoveLinearly;
    body->asleep = desc.startAsleep;
    body->sleepCounterThreshold = desc.sleepCounterThreshold;
    body->anchorPoint = desc.pose.position;

    const float radius = 0.5f * glm::length(desc.scale);
    body->invRadius = radius > 0.0f ? 1.0f / radius : 0.0f;

    if (desc.type == BodyType::Dynamic) {
        body->mass = desc.mass;
        body->invMass = 1.0f / desc.mass;
    }
    else {
        body->mass = 0.0f;
        body->invMass = 0.0f;
    }

    for (const ColliderDesc& colliderDesc : desc.colliders) {
        ColliderHandle colliderHandle =
            createCollider(bodyHandle, colliderDesc);

        if (!colliderHandle.isValid()) {
            destroyRigidBody(bodyHandle);
            return {};
        }
    }

    return bodyHandle;
}

bool Engine::destroyRigidBody(
    BodyHandle bodyHandle)
{
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(bodyHandle)) {
        return false;
    }

    commandBuffer.recordBodyDestroy(bodyHandle);
    return true;
}

//=========================================
// Rigid body commands
//=========================================
bool Engine::applyLinearImpulse(
    BodyHandle handle,
    const glm::vec3& impulse)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Dynamic) {
        return false;
    }

    commandBuffer.recordApplyLinearImpulse(handle, impulse);
    return true;
}

bool Engine::setLinearVelocity(
    BodyHandle handle,
    const glm::vec3& velocity)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle) ||
        body->type == BodyType::Static) {
        return false;
    }

    commandBuffer.recordSetLinearVelocity(handle, velocity);
    return true;
}

bool Engine::setAngularVelocity(
    BodyHandle handle,
    const glm::vec3& velocity)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle) ||
        body->type == BodyType::Static) {
        return false;
    }

    commandBuffer.recordSetAngularVelocity(handle, velocity);
    return true;
}

bool Engine::setKinematicTarget(
    BodyHandle handle,
    const Pose& target)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Kinematic) {
        return false;
    }

    commandBuffer.recordSetKinematicTarget(handle, target);
    return true;
}

bool Engine::setRigidBodyTransform(
    BodyHandle handle,
    const Pose& pose,
    const glm::vec3& scale)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || commandBuffer.isBodyPendingDestroy(handle)) {
        return false;
    }

    commandBuffer.recordSetRigidBodyTransform(handle, pose, scale);
    return true;
}

bool Engine::setRigidBodySleepState(
    BodyHandle handle,
    bool asleep)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Dynamic) {
        return false;
    }

    if (!body->allowSleep /*||
        body->motionControl == MotionControl::External*/) {
        return false;
    }

    commandBuffer.recordSetRigidBodySleepState(handle, asleep);
    return true;
}

bool Engine::setRigidBodyType(
    BodyHandle handle,
    BodyType type)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle)) {
        return false;
    }

    commandBuffer.recordSetRigidBodyType(handle, type);
    return true;
}

bool Engine::setRigidBodyMotionControl(
    BodyHandle handle,
    MotionControl motionControl)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle) ||
        body->type == BodyType::Static) {
        return false;
    }

    commandBuffer.recordSetRigidBodyMotionControl(
        handle,
        motionControl
    );

    return true;
}

bool Engine::setRigidBodyResponseMode(
    BodyHandle handle,
    ResponseMode responseMode)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || commandBuffer.isBodyPendingDestroy(handle)) {
        return false;
    }

    commandBuffer.recordSetRigidBodyResponseMode(handle, responseMode);
    return true;
}

bool Engine::setRigidBodyMass(
    BodyHandle handle,
    float mass)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Dynamic ||
        mass <= 0.0f) {
        return false;
    }

    commandBuffer.recordSetRigidBodyMass(handle, mass);
    return true;
}

bool Engine::setRigidBodyAllowGravity(
    BodyHandle handle,
    bool allowGravity)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || commandBuffer.isBodyPendingDestroy(handle)) {
        return false;
    }

    commandBuffer.recordSetRigidBodyAllowGravity(handle, allowGravity);
    return true;
}

bool Engine::setRigidBodyAllowSleep(
    BodyHandle handle,
    bool allowSleep)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Dynamic) {
        return false;
    }

    commandBuffer.recordSetRigidBodyAllowSleep(handle, allowSleep);
    return true;
}

bool Engine::setRigidBodyCanMoveLinearly(
    BodyHandle handle,
    bool canMoveLinearly)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || commandBuffer.isBodyPendingDestroy(handle)) {
        return false;
    }

    commandBuffer.recordSetRigidBodyCanMoveLinearly(
        handle,
        canMoveLinearly
    );
    return true;
}

}
