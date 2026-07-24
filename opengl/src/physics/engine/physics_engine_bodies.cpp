#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

RigidBodyHandle PhysicsEngine::createRigidBody(const RigidBodyDesc& desc) {
    return impl->createRigidBody(desc);
}

bool PhysicsEngine::destroyRigidBody(RigidBodyHandle body) {
    return impl->destroyRigidBody(body);
}

std::optional<RigidBodyState> PhysicsEngine::getRigidBodyState(RigidBodyHandle body) const {
    return impl->getRigidBodyState(body);
}

bool PhysicsEngine::applyLinearImpulse(RigidBodyHandle body, const glm::vec3& impulse) {
    return impl->applyLinearImpulse(body, impulse);
}

bool PhysicsEngine::setLinearVelocity(RigidBodyHandle body, const glm::vec3& velocity) {
    return impl->setLinearVelocity(body, velocity);
}

bool PhysicsEngine::setKinematicTarget(RigidBodyHandle body, const PhysicsPose& target) {
    return impl->setKinematicTarget(body, target);
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
    body->invRadius = 1.0f / (0.5f * glm::length(desc.scale));

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

    if (!body->colliderHandles.empty()) {
        body->aabb = physicsWorld.computeBodyAABB(*body);
    }

    return bodyHandle;
}

bool PhysicsEngine::Impl::destroyRigidBody(RigidBodyHandle bodyHandle) {
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body) {
        return false;
    }

    pending.erase(
        std::remove_if(
            pending.begin(),
            pending.end(),
            [bodyHandle](const PhysCmd& cmd) { return cmd.handle == bodyHandle; }),
        pending.end());

    if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
        broadphaseManager.remove(bodyHandle);
    }
    contactCache.clear();

    std::vector<ColliderHandle> colliderHandles = body->colliderHandles;

    for (ColliderHandle colliderHandle : colliderHandles) {
        physicsWorld.deleteCollider(colliderHandle);
    }

    physicsWorld.deleteRigidBody(bodyHandle);

    return true;
}

std::optional<RigidBodyState> PhysicsEngine::Impl::getRigidBodyState(RigidBodyHandle handle) const {
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body) {
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

bool PhysicsEngine::Impl::applyLinearImpulse(RigidBodyHandle handle, const glm::vec3& impulse) {
    RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || body->type != BodyType::Dynamic) {
        return false;
    }

    if (body->asleep) {
        body->setAwake();
        broadphaseManager.moveToAwake(handle);
    }

    body->applyImpulseLinear(impulse);

    return true;
}

bool PhysicsEngine::Impl::setLinearVelocity(RigidBodyHandle handle, const glm::vec3& velocity) {
    RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || body->type == BodyType::Static) {
        return false;
    }

    body->linearVelocity = velocity;

    if (body->asleep) {
        body->setAwake();
        broadphaseManager.moveToAwake(handle);
    }

    return true;
}

bool PhysicsEngine::Impl::setKinematicTarget(RigidBodyHandle handle, const PhysicsPose& target) {
    RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || body->type != BodyType::Kinematic) {
        return false;
    }

    body->pose = target;

    for (ColliderHandle colliderHandle : body->colliderHandles) {
        Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (!collider || !collider->enabled) {
            continue;
        }

        collider->updateWorldPose(body->pose, body->scale);
        collider->updateShape();
        collider->updateAABB();
    }

    body->aabb = physicsWorld.computeBodyAABB(*body);
    setBVHDirty(handle);

    return true;
}
