#include "pch.h"

#include "physics/physics_engine.h"

namespace physics {

using namespace internal;

//==============================================================
// Integrate Forces / Velocities
//==============================================================
void Engine::integrateForcesAndVelocities(
    const std::vector<BodyHandle>& bodies,
    float dt)
{
    for (const BodyHandle& bodyHandle : bodies) {
        RigidBody* body = caches.bodies.get(bodyHandle, FUNC_NAME);

        if (!body || body->colliderHandles.empty()) {
            continue;
        }

        Collider* mainCollider =
            caches.colliders.get(body->colliderHandles[0], FUNC_NAME);

        if (!mainCollider) {
            continue;
        }

        if (body->colliderHandles.size() == 1) {
            body->applyRollingFriction(mainCollider->type, dt);
        }

        body->applyVelocityDamping(dt);
        body->applyGravity(dt);
    }
}

//==============================================================
// Integrate Positions and Update Colliders
//==============================================================
void Engine::integratePositionsAndColliders(
    const std::vector<BodyHandle>& bodies,
    float dt)
{
    for (const BodyHandle& bodyHandle : bodies) {
        RigidBody* body = caches.bodies.get(bodyHandle, FUNC_NAME);

        if (!body) {
            continue;
        }

        body->integratePose(dt);
        body->updateInertiaWorld();
        updateCollidersAndBodyAABB(body);
    }
}

//==============================================================
// Update Collider Poses and Body AABB
//==============================================================
void Engine::updateCollidersAndBodyAABB(
    RigidBody* body)
{
    if (!body) {
        return;
    }

    bool hasAABB = false;
    internal::AABB combinedAABB;

    for (ColliderHandle colliderHandle : body->colliderHandles) {
        Collider* collider =
            caches.colliders.get(colliderHandle, FUNC_NAME);

        if (!collider || !collider->enabled) {
            continue;
        }

        collider->updateWorldPose(body->pose, body->scale);
        collider->updateShape();
        collider->updateAABB();

        const internal::AABB& colliderAABB = collider->getAABB();

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
        body->aabb = internal::AABB{};
        body->invRadius = 0.0f;
        return;
    }

    combinedAABB.worldCenter =
        (combinedAABB.worldMin + combinedAABB.worldMax) * 0.5f;

    combinedAABB.worldHalfExtents =
        (combinedAABB.worldMax - combinedAABB.worldMin) * 0.5f;

    body->aabb = combinedAABB;

    const float boundingRadius =
        glm::length(combinedAABB.worldHalfExtents);

    body->invRadius =
        boundingRadius > 0.0f
        ? 1.0f / boundingRadius
        : 0.0f;
}

}
