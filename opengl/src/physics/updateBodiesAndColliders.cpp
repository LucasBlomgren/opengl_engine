#include "pch.h"
#include "physics_engine.h"

//==============================================================
//   Integrate Forces / Velocities
//==============================================================
void PhysicsEngine::integrateForcesAndVelocities(
    const std::vector<RigidBodyHandle>& bodies,
    float dt)
{
    for (const RigidBodyHandle& bodyH : bodies) {
        RigidBody* body = caches.bodies.get(bodyH, FUNC_NAME);
        Collider* mainCollider =
            caches.colliders.get(body->colliderHandles[0], FUNC_NAME);

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
void PhysicsEngine::integratePositionsAndColliders(
    const std::vector<RigidBodyHandle>& bodies,
    float dt)
{
    for (const RigidBodyHandle& bodyH : bodies) {
        RigidBody* body = caches.bodies.get(bodyH, FUNC_NAME);
        Transform* rootTransform =
            caches.transforms.get(body->rootTransformHandle, FUNC_NAME);

        // This assumes integrateVelocities() means:
        // position/orientation += velocity/angularVelocity * dt
        body->integrateVelocities(*rootTransform, dt);

        rootTransform->updateCache();
        body->updateInertiaWorld(*rootTransform);

        updateCollidersAndBodyAABB(body, rootTransform);
    }
}

//==============================================================
//   Update Collider Poses and Body AABB
//==============================================================
void PhysicsEngine::updateCollidersAndBodyAABB(
    RigidBody* body,
    Transform* rootTransform)
{
    bool hasAABB = false;
    AABB combinedAABB;

    for (const ColliderHandle& colH : body->colliderHandles) {
        Collider* collider = caches.colliders.get(colH, FUNC_NAME);
        Transform* localTransform =
            caches.transforms.get(collider->localTransformHandle, FUNC_NAME);

        collider->pose.combineIntoColliderPose(
            *rootTransform,
            *localTransform
        );

        collider->pose.ensureModelMatrix();
        collider->updateAABB(collider->pose);
        collider->updateCollider(collider->pose);

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

    combinedAABB.setSurfaceArea();

    body->aabb = combinedAABB;
}