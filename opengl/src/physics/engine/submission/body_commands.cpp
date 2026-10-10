#include "pch.h"

#include <utility>

#include "physics/public/engine.h"

namespace physics {

using namespace internal;

namespace {

const RigidBody* resolveBody(
    const cmd::Buffer& commandBuffer,
    const PhysicsStorage& physicsStorage,
    BodyHandle handle)
{
    if (commandBuffer.isBodyPendingDestroy(handle)) {
        return nullptr;
    }

    if (const RigidBody* body = commandBuffer.tryGetPendingBodyCreate(handle)) {
        return body;
    }

    return physicsStorage.tryGetBody(handle);
}

}

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

    BodyHandle bodyHandle = physicsStorage.reserveBodyHandle();

    RigidBody body;
    body.pose = desc.pose;
    body.scale = desc.scale;
    body.type = desc.type;
    body.reportContacts = desc.reportContacts;

    body.linearVelocity = desc.linearVelocity;
    body.angularVelocity = desc.angularVelocity;
    body.allowGravity = desc.allowGravity;

    const float radius = 0.5f * glm::length(desc.scale);
    body.invRadius = radius > 0.0f ? 1.0f / radius : 0.0f;

    if (desc.type == BodyType::Dynamic) {
        body.mass = desc.mass;
        body.invMass = 1.0f / desc.mass;
    }
    else {
        body.mass = 0.0f;
        body.invMass = 0.0f;
    }

    // Commit motion and sleep states for dynamic bodies
    if (desc.type == BodyType::Dynamic) {
        MotionState motionState;
        motionState.mass = body.mass;
        motionState.radius = radius;
        motionState.invRadius = body.invRadius;
        motionState.linearVelocity = desc.linearVelocity;
        motionState.angularVelocity = desc.angularVelocity;
        motionState.allowGravity = desc.allowGravity;

        SleepState sleepState {
            desc.allowSleep, 
            desc.startAsleep, 
            desc.pose.position
        };

        body.motionStateHandle = 
            physicsStorage.commitMotionState(std::move(motionState));
        body.sleepStateHandle = 
            physicsStorage.commitSleepState(std::move(sleepState));
    }

    commandBuffer.recordBodyCreate(bodyHandle, std::move(body));

    // Stage the body before its colliders so createCollider() can resolve the
    // pending parent through the command buffer.
    // recordBodyCreate() must therefore stay before this loop.
    for (const ColliderDesc& colliderDesc : desc.colliders) {
        ColliderHandle colliderHandle =
            createCollider(bodyHandle, colliderDesc);

        if (!colliderHandle.isValid()) {
            commandBuffer.recordBodyDestroy(bodyHandle, physicsStorage);
            return {};
        }
    }

    return bodyHandle;
}

bool Engine::destroyRigidBody(
    BodyHandle bodyHandle)
{
    if (!resolveBody(commandBuffer, physicsStorage, bodyHandle)) {
        return false;
    }

    return commandBuffer.recordBodyDestroy(bodyHandle, physicsStorage);
}

//=========================================
// Rigid body commands
//=========================================
bool Engine::applyLinearImpulse(
    BodyHandle handle,
    const glm::vec3& impulse)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body || body->type != BodyType::Dynamic) {
        return false;
    }

    commandBuffer.recordApplyLinearImpulse(handle, impulse);
    return true;
}

bool Engine::setLinearVelocity(
    BodyHandle handle,
    const glm::vec3& velocity)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body || body->type == BodyType::Static) {
        return false;
    }

    commandBuffer.recordSetLinearVelocity(handle, velocity);
    return true;
}

bool Engine::setAngularVelocity(
    BodyHandle handle,
    const glm::vec3& velocity)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body || body->type == BodyType::Static) {
        return false;
    }

    commandBuffer.recordSetAngularVelocity(handle, velocity);
    return true;
}

bool Engine::setKinematicTarget(
    BodyHandle handle,
    const Pose& target)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body || body->type != BodyType::Kinematic) {
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
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body) {
        return false;
    }

    commandBuffer.recordSetBodyTransform(handle, pose, scale);
    return true;
}

bool Engine::setRigidBodySleepState(
    BodyHandle handle,
    bool asleep)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body || body->type != BodyType::Dynamic) {
        return false;
    }

    const SleepState& sleepState = 
        physicsStorage.getSleepState(body->sleepStateHandle);

    if (!sleepState.canSleep()) {
        return false;
    }

    commandBuffer.recordSetBodySleepState(handle, asleep);
    return true;
}

bool Engine::setRigidBodyType(
    BodyHandle handle,
    BodyType type)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body) {
        return false;
    }

    commandBuffer.recordSetBodyType(handle, type);
    return true;
}

bool Engine::setRigidBodyReportContacts(
    BodyHandle handle,
    bool reportContacts)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body) {
        return false;
    }

    commandBuffer.recordSetBodyReportContacts(handle, reportContacts);
    return true;
}

bool Engine::setRigidBodyMass(
    BodyHandle handle,
    float mass)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body ||
        body->type != BodyType::Dynamic ||
        mass <= 0.0f) {
        return false;
    }

    commandBuffer.recordSetBodyMass(handle, mass);
    return true;
}

bool Engine::setRigidBodyAllowGravity(
    BodyHandle handle,
    bool allowGravity)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body) {
        return false;
    }

    commandBuffer.recordSetBodyAllowGravity(handle, allowGravity);
    return true;
}

bool Engine::setRigidBodyAllowSleep(
    BodyHandle handle,
    bool allowSleep)
{
    const RigidBody* body = resolveBody(
        commandBuffer,
        physicsStorage,
        handle
    );

    if (!body || body->type != BodyType::Dynamic) {
        return false;
    }

    commandBuffer.recordSetBodyAllowSleep(handle, allowSleep);
    return true;
}

}
