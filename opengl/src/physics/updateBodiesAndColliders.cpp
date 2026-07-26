#include "pch.h"
#include "physics/engine/physics_engine_impl.h"

//==============================================================
//   Integrate Forces / Velocities
//==============================================================
void PhysicsEngine::Impl::integrateForcesAndVelocities(
    const std::vector<RigidBodyHandle>& bodies,
    float dt)
{
    for (const RigidBodyHandle& bodyH : bodies) {
        RigidBody* body = caches.bodies.get(bodyH, FUNC_NAME);
        if (!body || body->colliderHandles.empty()) {
            continue;
        }

        Collider* mainCollider =
            caches.colliders.get(body->colliderHandles[0], FUNC_NAME);
        if (!mainCollider) {
            continue;
        }

        // For solo spheres.
        if (body->colliderHandles.size() == 1) {
            body->applyRollingFriction(mainCollider->type, dt);
        }

        body->applyVelocityDamping(dt);
        body->applyGravity(dt);
        // body->applyAntistuckFriction(dt);
    }
}

//==============================================================
//   Integrate Positions and Update Colliders
//==============================================================
void PhysicsEngine::Impl::integratePositionsAndColliders(
    const std::vector<RigidBodyHandle>& bodies,
    float dt)
{
    for (const RigidBodyHandle& bodyH : bodies) {
        RigidBody* body = caches.bodies.get(bodyH, FUNC_NAME);
        if (!body) {
            continue;
        }

        Transform* rootTransform =
            caches.transforms.get(body->rootTransformHandle, FUNC_NAME);

        // This assumes integrateVelocities() means:
        // position/orientation += velocity/angularVelocity * dt
        body->integratePose(dt);

        if (rootTransform) {
            rootTransform->lastPosition = rootTransform->position;
            rootTransform->position = body->pose.position;
            rootTransform->orientation = body->pose.orientation;
            rootTransform->updateCache();
        }

        body->updateInertiaWorld();

        updateCollidersAndBodyAABB(body, rootTransform);
    }
}

//==============================================================
//   Update Collider Poses and Body AABB
//==============================================================
void PhysicsEngine::Impl::updateCollidersAndBodyAABB(
    RigidBody* body,
    Transform* rootTransform)
{
    bool hasAABB = false;
    AABB combinedAABB;

    for (const ColliderHandle& colH : body->colliderHandles) {
        Collider* collider = caches.colliders.get(colH, FUNC_NAME);
        if (!collider || !collider->enabled) {
            continue;
        }

        Transform* localTransform =
            caches.transforms.get(collider->localTransformHandle, FUNC_NAME);

        if (localTransform) {
            collider->localPose.position = localTransform->position;
            collider->localPose.orientation = localTransform->orientation;
            collider->localScale = localTransform->scale;
        }

        collider->updateWorldPose(body->pose, body->scale);
        collider->updateShape();
        collider->updateAABB();

        const AABB& colliderAABB = collider->getAABB();

        if (!hasAABB) {
            combinedAABB = colliderAABB;
            hasAABB = true;
        }
        else {
            combinedAABB.growToInclude(colliderAABB.worldMin);
            combinedAABB.growToInclude(colliderAABB.worldMax);
        }
    }

    if (!hasAABB) {
        return;
    }

    combinedAABB.worldCenter =
        (combinedAABB.worldMin + combinedAABB.worldMax) * 0.5f;

    combinedAABB.worldHalfExtents =
        (combinedAABB.worldMax - combinedAABB.worldMin) * 0.5f;

    body->aabb = combinedAABB;
}
