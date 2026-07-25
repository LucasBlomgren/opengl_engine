#include "pch.h"

#include <algorithm>
#include <type_traits>

#include "physics/engine/physics_engine_impl.h"

namespace {
    template<class Handle>
    bool containsHandle(const std::vector<Handle>& handles, Handle handle) {
        return std::find(handles.begin(), handles.end(), handle) != handles.end();
    }

    template<class Handle>
    void addUniqueHandle(std::vector<Handle>& handles, Handle handle) {
        if (!containsHandle(handles, handle)) {
            handles.push_back(handle);
        }
    }

    template<class Handle>
    void eraseHandle(std::vector<Handle>& handles, Handle handle) {
        handles.erase(std::remove(handles.begin(), handles.end(), handle), handles.end());
    }

    BroadphaseBucket getBodyBucket(const RigidBody& body) {
        if (body.type == BodyType::Static) {
            return BroadphaseBucket::Static;
        }

        if (body.type == BodyType::Dynamic &&
            body.asleep &&
            body.motionControl != MotionControl::External) {
            return BroadphaseBucket::Asleep;
        }

        return BroadphaseBucket::Awake;
    }

    bool hasEnabledCollider(PhysicsWorld& physicsWorld, const RigidBody& body) {
        for (ColliderHandle colliderHandle : body.colliderHandles) {
            if (!physicsWorld.isColliderActive(colliderHandle)) {
                continue;
            }

            const Collider* collider = physicsWorld.getCollider(colliderHandle);

            if (collider && collider->enabled) {
                return true;
            }
        }

        return false;
    }

    void refreshBodyInertia(PhysicsWorld& physicsWorld, RigidBody& body) {
        if (body.type != BodyType::Dynamic || body.colliderHandles.empty()) {
            return;
        }

        Collider* collider = nullptr;

        for (ColliderHandle colliderHandle : body.colliderHandles) {
            if (!physicsWorld.isColliderActive(colliderHandle)) {
                continue;
            }

            Collider* candidate = physicsWorld.getCollider(colliderHandle);

            if (candidate && candidate->enabled) {
                collider = candidate;
                break;
            }
        }

        if (!collider) {
            return;
        }

        Transform inertiaTransform;

        if (collider->type == ColliderType::SPHERE) {
            const Sphere& sphere = std::get<Sphere>(collider->shape);
            inertiaTransform.scale = glm::vec3(sphere.radiusWorld * 2.0f);
        }
        else {
            inertiaTransform.scale = collider->worldScale;
        }

        body.calculateInverseInertia(collider->type, *collider, inertiaTransform);
        body.updateInertiaWorld();
    }
}

//=========================================
// External command processing
//=========================================
void PhysicsEngine::Impl::flushExternalCommands() {
    PhysicsExternalCommandBuffer::Batch batch = externalCommands.take();

    if (batch.empty()) {
        return;
    }

    processLifecycleCommands(batch);
    processMutationCommands(batch.mutations);
}

//=========================================
// Lifecycle command processing
//=========================================
void PhysicsEngine::Impl::processLifecycleCommands(const PhysicsExternalCommandBuffer::Batch& batch) {
    if (batch.bodyCreates.empty() &&
        batch.colliderCreates.empty() &&
        batch.bodyDestroys.empty() &&
        batch.colliderDestroys.empty()) {
        return;
    }

    std::vector<RigidBodyHandle> affectedBodies;

    //----------------------------------
    // Collect affected bodies
    //----------------------------------
    for (RigidBodyHandle bodyHandle : batch.bodyCreates) {
        addUniqueHandle(affectedBodies, bodyHandle);
    }

    for (RigidBodyHandle bodyHandle : batch.bodyDestroys) {
        addUniqueHandle(affectedBodies, bodyHandle);
    }

    for (ColliderHandle colliderHandle : batch.colliderCreates) {
        const Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (collider) {
            addUniqueHandle(affectedBodies, collider->rigidBodyHandle);
        }
    }

    for (ColliderHandle colliderHandle : batch.colliderDestroys) {
        const Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (collider) {
            addUniqueHandle(affectedBodies, collider->rigidBodyHandle);
        }
    }

    //----------------------------------
    // Remove affected active bodies from broadphase
    //----------------------------------
    for (RigidBodyHandle bodyHandle : affectedBodies) {
        if (!physicsWorld.isRigidBodyActive(bodyHandle)) {
            continue;
        }

        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (body && body->broadphaseHandle.bucket != BroadphaseBucket::None) {
            broadphaseManager.remove(bodyHandle);
        }
    }

    bool activeStorageChanged = false;

    //----------------------------------
    // Destroy individual colliders
    //----------------------------------
    for (ColliderHandle colliderHandle : batch.colliderDestroys) {
        Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (!collider) {
            continue;
        }

        RigidBodyHandle bodyHandle = collider->rigidBodyHandle;

        // The body-destroy phase owns all colliders belonging to a destroyed body.
        if (containsHandle(batch.bodyDestroys, bodyHandle)) {
            continue;
        }

        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (body) {
            eraseHandle(body->colliderHandles, colliderHandle);
        }

        if (physicsWorld.isColliderActive(colliderHandle)) {
            physicsWorld.deleteCollider(colliderHandle);
            activeStorageChanged = true;
        }
        else if (physicsWorld.isColliderPending(colliderHandle)) {
            physicsWorld.discardPendingCollider(colliderHandle);
        }
    }

    //----------------------------------
    // Destroy bodies and their colliders
    //----------------------------------
    for (RigidBodyHandle bodyHandle : batch.bodyDestroys) {
        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (!body) {
            continue;
        }

        const std::vector<ColliderHandle> activeColliders = body->colliderHandles;

        for (ColliderHandle colliderHandle : activeColliders) {
            if (physicsWorld.isColliderActive(colliderHandle)) {
                physicsWorld.deleteCollider(colliderHandle);
                activeStorageChanged = true;
            }
            else if (physicsWorld.isColliderPending(colliderHandle)) {
                physicsWorld.discardPendingCollider(colliderHandle);
            }
        }

        // Pending colliders are not necessarily present in body->colliderHandles yet.
        for (ColliderHandle colliderHandle : batch.colliderCreates) {
            Collider* collider = physicsWorld.getCollider(colliderHandle);

            if (!collider || collider->rigidBodyHandle != bodyHandle) {
                continue;
            }

            if (physicsWorld.isColliderPending(colliderHandle)) {
                physicsWorld.discardPendingCollider(colliderHandle);
            }
            else if (physicsWorld.isColliderActive(colliderHandle)) {
                physicsWorld.deleteCollider(colliderHandle);
                activeStorageChanged = true;
            }
        }

        if (physicsWorld.isRigidBodyActive(bodyHandle)) {
            physicsWorld.deleteRigidBody(bodyHandle);
            activeStorageChanged = true;
        }
        else if (physicsWorld.isRigidBodyPending(bodyHandle)) {
            physicsWorld.discardPendingRigidBody(bodyHandle);
        }
    }

    //----------------------------------
    // Activate pending bodies
    //----------------------------------
    for (RigidBodyHandle bodyHandle : batch.bodyCreates) {
        if (containsHandle(batch.bodyDestroys, bodyHandle)) {
            continue;
        }

        if (!physicsWorld.isRigidBodyPending(bodyHandle)) {
            continue;
        }

        if (!physicsWorld.activateRigidBody(bodyHandle)) {
            physicsWorld.discardPendingRigidBody(bodyHandle);
            continue;
        }

        activeStorageChanged = true;
    }

    //----------------------------------
    // Activate pending colliders
    //----------------------------------
    for (ColliderHandle colliderHandle : batch.colliderCreates) {
        if (containsHandle(batch.colliderDestroys, colliderHandle)) {
            if (physicsWorld.isColliderPending(colliderHandle)) {
                physicsWorld.discardPendingCollider(colliderHandle);
            }

            continue;
        }

        Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (!collider) {
            continue;
        }

        RigidBodyHandle bodyHandle = collider->rigidBodyHandle;

        if (containsHandle(batch.bodyDestroys, bodyHandle) ||
            !physicsWorld.isRigidBodyActive(bodyHandle)) {
            if (physicsWorld.isColliderPending(colliderHandle)) {
                physicsWorld.discardPendingCollider(colliderHandle);
            }

            continue;
        }

        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (!body) {
            physicsWorld.discardPendingCollider(colliderHandle);
            continue;
        }

        collider->updateWorldPose(body->pose, body->scale);
        collider->updateShape();
        collider->updateAABB();

        if (!physicsWorld.activateCollider(colliderHandle)) {
            physicsWorld.discardPendingCollider(colliderHandle);
            continue;
        }

        if (!containsHandle(body->colliderHandles, colliderHandle)) {
            body->colliderHandles.push_back(colliderHandle);
        }

        activeStorageChanged = true;
    }

    //----------------------------------
    // Clear pointer caches after dense storage changes
    //----------------------------------
    if (activeStorageChanged) {
        caches.clear();
    }

    //----------------------------------
    // Rebuild affected surviving bodies
    //----------------------------------
    for (RigidBodyHandle bodyHandle : affectedBodies) {
        if (!physicsWorld.isRigidBodyActive(bodyHandle)) {
            continue;
        }

        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (!body) {
            continue;
        }

        std::vector<ColliderHandle>& colliderHandles = body->colliderHandles;

        colliderHandles.erase(
            std::remove_if(
                colliderHandles.begin(),
                colliderHandles.end(),
                [this](ColliderHandle colliderHandle) {
                    return !physicsWorld.isColliderActive(colliderHandle);
                }),
            colliderHandles.end());

        if (!hasEnabledCollider(physicsWorld, *body)) {
            body->aabb = AABB{};
            continue;
        }

        body->aabb = physicsWorld.computeBodyAABB(*body);

        const float radius = 0.5f * glm::length(body->aabb.worldMax - body->aabb.worldMin);
        body->invRadius = radius > 0.0f ? 1.0f / radius : 0.0f;

        refreshBodyInertia(physicsWorld, *body);

        broadphaseManager.add(bodyHandle, getBodyBucket(*body));
    }

    // #TODO: Can't clear contact cache here because it may contain contacts for bodies that are still active. 
    // Need to implement a more selective contact cache clearing mechanism.
    //contactCache.clear();
}

//=========================================
// Mutation command processing
//=========================================
void PhysicsEngine::Impl::processMutationCommands(const std::vector<PhysicsExternalCommandBuffer::Mutation>& mutations) {
    for (const PhysicsExternalCommandBuffer::Mutation& mutation : mutations) {
        std::visit([this](const auto& command) {
            using CommandType = std::decay_t<decltype(command)>;

            if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::ApplyLinearImpulse>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetLinearVelocity>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetAngularVelocity>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetKinematicTarget>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetRigidBodyAwake>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetRigidBodyAsleep>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetRigidBodyType>) {
                RigidBody* body = physicsWorld.getRigidBody(command.body);

                if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
                    return;
                }

                body->type = command.type;

                switch (command.type) {
                case BodyType::Dynamic:
                    if (body->mass <= 0.0f) {
                        body->mass = 1.0f;
                    }

                    body->invMass = 1.0f / body->mass;

                    if (body->sleepCounterThreshold <= 0.0f) {
                        body->sleepCounterThreshold = 1.5f;
                    }

                    refreshBodyInertia(physicsWorld, *body);

                    if (body->allowSleep && body->asleep) {
                        body->setAsleep();
                        broadphaseManager.moveToAsleep(command.body);
                    }
                    else {
                        body->setAwake();
                        broadphaseManager.moveToAwake(command.body);
                    }

                    break;

                case BodyType::Kinematic:
                    body->invMass = 0.0f;
                    body->linearVelocity = glm::vec3(0.0f);
                    body->angularVelocity = glm::vec3(0.0f);
                    body->setAwake();
                    broadphaseManager.moveToAwake(command.body);
                    break;

                case BodyType::Static:
                    body->mass = 0.0f;
                    body->invMass = 0.0f;
                    body->linearVelocity = glm::vec3(0.0f);
                    body->angularVelocity = glm::vec3(0.0f);
                    broadphaseManager.moveToStatic(command.body);
                    break;
                }

                // #TODO: Can't clear contact cache here because it may contain contacts for bodies that are still active. 
                // Need to implement a more selective contact cache clearing mechanism.
                //contactCache.clear();
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetRigidBodyMotionControl>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetColliderLocalPose>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetColliderEnabled>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SetColliderTrigger>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SleepAllObjects>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::AwakenAllObjects>) {
                // Implementation.
            }
            else if constexpr (std::is_same_v<CommandType, PhysicsExternalCommandBuffer::SyncBodyFromTransform>) {
                // Temporary legacy implementation.
            }
            }, mutation);
    }
}