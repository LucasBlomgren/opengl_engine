#include "pch.h"

#include <algorithm>
#include "physics/engine/physics_engine_impl.h"

namespace physics::internal {

//=========================================
// Helper functions
//=========================================
namespace {
    template<class Handle>
    bool containsHandle(
        const std::vector<Handle>& handles,
        Handle handle) {
        return std::find(handles.begin(), handles.end(), handle) != handles.end();
    }

    template<class Handle>
    void addUniqueHandle(
        std::vector<Handle>& handles, 
        Handle handle) 
    {
        if (!containsHandle(handles, handle)) {
            handles.push_back(handle);
        }
    }

    template<class Handle>
    void eraseHandle(
        std::vector<Handle>& handles, 
        Handle handle) {
        handles.erase(std::remove(
            handles.begin(), 
            handles.end(), 
            handle), 
            handles.end()
        );
    }

    BroadphaseBucket getBodyBucket(
        const RigidBody& body) 
    {
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

    bool hasEnabledCollider(
        PhysicsWorld& physicsWorld, 
        const RigidBody& body) 
    {
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

}

//=========================================
// Lifecycle command processing
//=========================================
void EngineImpl::processLifecycleCommands(
    const PhysicsCommandBuffer::Batch& batch)
{
    if (batch.bodyCreates.empty() &&
        batch.colliderCreates.empty() &&
        batch.bodyDestroys.empty() &&
        batch.colliderDestroys.empty()) {
        return;
    }

    std::vector<BodyHandle> affectedBodies;

    //----------------------------------
    // Collect affected bodies
    //----------------------------------
    for (BodyHandle bodyHandle : batch.bodyCreates) {
        addUniqueHandle(affectedBodies, bodyHandle);
    }

    for (BodyHandle bodyHandle : batch.bodyDestroys) {
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
    for (BodyHandle bodyHandle : affectedBodies) {
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

        BodyHandle bodyHandle = collider->rigidBodyHandle;

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
    for (BodyHandle bodyHandle : batch.bodyDestroys) {
        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (!body) {
            continue;
        }

        const std::vector<ColliderHandle> activeColliders = body->colliderHandles;

        // Destroy all colliders belonging to the body being destroyed.
        for (ColliderHandle colliderHandle : activeColliders) {
            physicsWorld.deleteCollider(colliderHandle);
            activeStorageChanged = true;
        }

        // Pending colliders are not necessarily present in body->colliderHandles yet.
        for (ColliderHandle colliderHandle : batch.colliderCreates) {
            Collider* collider = physicsWorld.getCollider(colliderHandle);

            // Check if the collider belongs to the body being destroyed
            if (!collider || collider->rigidBodyHandle != bodyHandle) {
                continue;
            }

            // If we are here it means the collider is pending,
            // so discard it from the pending colliders list.
            physicsWorld.discardPendingCollider(colliderHandle);
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
    for (BodyHandle bodyHandle : batch.bodyCreates) {
        if (containsHandle(batch.bodyDestroys, bodyHandle)) {
            continue;
        }

        if (physicsWorld.activateRigidBody(bodyHandle)) {
            activeStorageChanged = true;
        }
    }

    //----------------------------------
    // Activate pending colliders
    //----------------------------------
    for (ColliderHandle colliderHandle : batch.colliderCreates) {
        if (containsHandle(batch.colliderDestroys, colliderHandle)) {
            continue;
        }

        Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (!collider) {
            continue;
        }

        BodyHandle bodyHandle = collider->rigidBodyHandle;

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
    for (BodyHandle bodyHandle : affectedBodies) {
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

        refreshBodyInertia(*body);

        broadphaseManager.add(bodyHandle, getBodyBucket(*body));
    }

    // #TODO: Can't clear contact cache here because it may contain contacts for bodies that are still active. 
    // Need to implement a more selective contact cache clearing mechanism.
    //contactCache.clear();
}

}
