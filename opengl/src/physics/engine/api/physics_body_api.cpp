#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=========================================
// Public facade and submission: rigid bodies
//=========================================
RigidBodyHandle PhysicsEngine::createRigidBody(
    const RigidBodyDesc& desc) {
    return impl->submitCreateRigidBody(desc);
}

bool PhysicsEngine::destroyRigidBody(
    RigidBodyHandle body) {
    return impl->submitDestroyRigidBody(body);
}

RigidBodyHandle PhysicsEngine::Impl::submitCreateRigidBody(
    const RigidBodyDesc& desc) 
{
    if (desc.type == BodyType::Dynamic && desc.mass <= 0.0f) {
        return {};
    }

    RigidBodyHandle bodyHandle = physicsWorld.createPendingRigidBody();
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

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
            submitCreateCollider(bodyHandle, colliderDesc);

        if (!colliderHandle.isValid()) {
            submitDestroyRigidBody(bodyHandle);
            return {};
        }
    }

    return bodyHandle;
}

bool PhysicsEngine::Impl::submitDestroyRigidBody(
    RigidBodyHandle bodyHandle) 
{
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body || commandBuffer.isBodyPendingDestroy(bodyHandle)) {
        return false;
    }

    commandBuffer.recordBodyDestroy(bodyHandle);

    return true;
}

//=========================================
// Rigid body commands
//=========================================
bool PhysicsEngine::applyLinearImpulse(
    RigidBodyHandle body, 
    const glm::vec3& impulse) {
    return impl->submitApplyLinearImpulse(body, impulse);
}

bool PhysicsEngine::setLinearVelocity(
    RigidBodyHandle body,
    const glm::vec3& velocity) {
    return impl->submitSetLinearVelocity(body, velocity);
}

bool PhysicsEngine::setAngularVelocity(
    RigidBodyHandle body, 
    const glm::vec3& velocity) {
    return impl->submitSetAngularVelocity(body, velocity);
}

bool PhysicsEngine::setKinematicTarget(
    RigidBodyHandle body, 
    const PhysicsPose& target) {
    return impl->submitSetKinematicTarget(body, target);
}

bool PhysicsEngine::setRigidBodyAwake(
    RigidBodyHandle body) {
    return impl->submitSetRigidBodyAwake(body, true);
}

bool PhysicsEngine::setRigidBodyAsleep(
    RigidBodyHandle body) {
    return impl->submitSetRigidBodyAwake(body, false);
}

bool PhysicsEngine::setRigidBodyType(
    RigidBodyHandle body, 
    BodyType type) {
    return impl->submitSetRigidBodyType(body, type);
}

bool PhysicsEngine::setRigidBodyMotionControl(
    RigidBodyHandle body, 
    MotionControl motionControl) {
    return impl->submitSetRigidBodyMotionControl(body, motionControl);
}

bool PhysicsEngine::Impl::submitApplyLinearImpulse(
    RigidBodyHandle handle, 
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

bool PhysicsEngine::Impl::submitSetLinearVelocity(
    RigidBodyHandle handle, 
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

bool PhysicsEngine::Impl::submitSetAngularVelocity(
    RigidBodyHandle handle, 
    const glm::vec3& velocity) 
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || commandBuffer.isBodyPendingDestroy(handle) ||
        body->type == BodyType::Static) {
        return false;
    }

    commandBuffer.recordSetAngularVelocity(handle, velocity);
    return true;
}

bool PhysicsEngine::Impl::submitSetKinematicTarget(
    RigidBodyHandle handle,
    const PhysicsPose& target) 
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

bool PhysicsEngine::Impl::submitSetRigidBodyAwake(
    RigidBodyHandle handle,
    bool awake)
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        commandBuffer.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Dynamic) {
        return false;
    }

    if (!body->allowSleep ||
        body->motionControl == MotionControl::External) {
        return false;
    }

    commandBuffer.recordSetRigidBodyAwake(handle, awake);
    return true;
}

bool PhysicsEngine::Impl::submitSetRigidBodyType(
    RigidBodyHandle handle, 
    BodyType type) 
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || commandBuffer.isBodyPendingDestroy(handle)) {
        return false;
    }

    commandBuffer.recordSetRigidBodyType(handle, type);
    return true;
}

bool PhysicsEngine::Impl::submitSetRigidBodyMotionControl(
    RigidBodyHandle handle, 
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
