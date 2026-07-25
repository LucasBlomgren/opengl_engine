#include "pch.h"

#include <algorithm>

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=========================================
// Rigid body creation and destruction
//=========================================
RigidBodyHandle PhysicsEngine::createRigidBody(const RigidBodyDesc& desc) {
    return impl->createRigidBody(desc);
}

bool PhysicsEngine::destroyRigidBody(RigidBodyHandle body) {
    return impl->destroyRigidBody(body);
}

RigidBodyHandle PhysicsEngine::Impl::createRigidBody(const RigidBodyDesc& desc) {
    if (desc.type == BodyType::Dynamic && desc.mass <= 0.0f) {
        return {};
    }

    RigidBodyHandle bodyHandle = physicsWorld.createRigidBody();
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body) {
        return {};
    }

    externalCommands.recordBodyCreate(bodyHandle);

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
        ColliderHandle colliderHandle = createCollider(bodyHandle, colliderDesc);

        if (!colliderHandle.isValid()) {
            destroyRigidBody(bodyHandle);
            return {};
        }
    }

    return bodyHandle;
}

bool PhysicsEngine::Impl::destroyRigidBody(RigidBodyHandle bodyHandle) {
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body || externalCommands.isBodyPendingDestroy(bodyHandle)) {
        return false;
    }

    externalCommands.recordBodyDestroy(bodyHandle);

    return true;
}

//=========================================
// Rigid body commands
//=========================================
bool PhysicsEngine::applyLinearImpulse(RigidBodyHandle body, const glm::vec3& impulse) {
    return impl->applyLinearImpulse(body, impulse);
}

bool PhysicsEngine::setLinearVelocity(RigidBodyHandle body, const glm::vec3& velocity) {
    return impl->setLinearVelocity(body, velocity);
}

bool PhysicsEngine::setAngularVelocity(RigidBodyHandle body, const glm::vec3& velocity) {
    return impl->setAngularVelocity(body, velocity);
}

bool PhysicsEngine::setKinematicTarget(RigidBodyHandle body, const PhysicsPose& target) {
    return impl->setKinematicTarget(body, target);
}

bool PhysicsEngine::setRigidBodyAwake(RigidBodyHandle body) {
    return impl->setRigidBodyAwake(body);
}

bool PhysicsEngine::setRigidBodyAsleep(RigidBodyHandle body) {
    return impl->setRigidBodyAsleep(body);
}

bool PhysicsEngine::setRigidBodyType(RigidBodyHandle body, BodyType type) {
    return impl->setRigidBodyType(body, type);
}

bool PhysicsEngine::setRigidBodyMotionControl(RigidBodyHandle body, MotionControl motionControl) {
    return impl->setRigidBodyMotionControl(body, motionControl);
}

bool PhysicsEngine::Impl::applyLinearImpulse(RigidBodyHandle handle, const glm::vec3& impulse) {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        externalCommands.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Dynamic) {
        return false;
    }

    externalCommands.queueApplyLinearImpulse(handle, impulse);
    return true;
}

bool PhysicsEngine::Impl::setLinearVelocity(RigidBodyHandle handle, const glm::vec3& velocity) {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        externalCommands.isBodyPendingDestroy(handle) ||
        body->type == BodyType::Static) {
        return false;
    }

    externalCommands.queueSetLinearVelocity(handle, velocity);
    return true;
}

bool PhysicsEngine::Impl::setAngularVelocity(RigidBodyHandle handle, const glm::vec3& velocity) {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || externalCommands.isBodyPendingDestroy(handle) || body->type == BodyType::Static) {
        return false;
    }

    externalCommands.queueSetAngularVelocity(handle, velocity);
    return true;
}

bool PhysicsEngine::Impl::setKinematicTarget(RigidBodyHandle handle, const PhysicsPose& target) {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        externalCommands.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Kinematic) {
        return false;
    }

    externalCommands.queueSetKinematicTarget(handle, target);
    return true;
}

bool PhysicsEngine::Impl::setRigidBodyAwake(RigidBodyHandle handle) {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        externalCommands.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Dynamic) {
        return false;
    }

    if (!body->allowSleep || body->motionControl == MotionControl::External) {
        return false;
    }

    externalCommands.queueSetRigidBodyAwake(handle);
    return true;
}

bool PhysicsEngine::Impl::setRigidBodyAsleep(RigidBodyHandle handle) {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        externalCommands.isBodyPendingDestroy(handle) ||
        body->type != BodyType::Dynamic ||
        !body->allowSleep ||
        body->motionControl == MotionControl::External) {
        return false;
    }

    externalCommands.queueSetRigidBodyAsleep(handle);
    return true;
}

bool PhysicsEngine::Impl::setRigidBodyType(RigidBodyHandle handle, BodyType type) {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || externalCommands.isBodyPendingDestroy(handle)) {
        return false;
    }

    externalCommands.queueSetRigidBodyType(handle, type);
    return true;
}

bool PhysicsEngine::Impl::setRigidBodyMotionControl(RigidBodyHandle handle, MotionControl motionControl) {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body ||
        externalCommands.isBodyPendingDestroy(handle) ||
        body->type == BodyType::Static) {
        return false;
    }

    externalCommands.queueSetRigidBodyMotionControl(handle, motionControl);
    return true;
}

//=========================================
// Rigid body state queries
//=========================================
std::optional<RigidBodyState> PhysicsEngine::getRigidBodyState(RigidBodyHandle body) const {
    return impl->getRigidBodyState(body);
}

std::optional<RigidBodyState> PhysicsEngine::Impl::getRigidBodyState(RigidBodyHandle handle) const {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || externalCommands.isBodyPendingDestroy(handle)) {
        return std::nullopt;
    }

    RigidBodyState state;
    state.pose = body->pose;
    state.linearVelocity = body->linearVelocity;
    state.angularVelocity = body->angularVelocity;
    state.type = body->type;
    state.asleep = body->asleep;

    return state;
}
