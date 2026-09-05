#include "pch.h"

#include "physics/public/engine.h"
#include "physics/engine/runtime/body_spatial_update.h"

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
        RigidBody& body = physicsWorld.getBody(bodyHandle);

        if (body.colliderHandles.empty()) {
            continue;
        }

        Collider& mainCollider =
            physicsWorld.getCollider(body.colliderHandles[0]);

        if (body.type == BodyType::Dynamic) 
        {
            SleepState& sleepState = physicsWorld.getSleepState(body.sleepStateHandle);
            if (sleepState.asleep || 
                body.motionControl == MotionControl::External) {
                continue;
            }

            if (body.colliderHandles.size() == 1) {
                body.applyRollingFriction(mainCollider.type, dt, sleepState);
            }

            body.applyVelocityDamping(dt, sleepState);
            body.applyGravity(dt);
        }
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
        RigidBody& body = physicsWorld.getBody(bodyHandle);

        if (body.type == BodyType::Dynamic) {
            SleepState& sleepState = physicsWorld.getSleepState(body.sleepStateHandle);
            if (sleepState.asleep ||
                body.motionControl == MotionControl::External) {
                continue;
            }
        }

        body.integratePose(dt);
        body.updateInertiaWorld();
        updateCollidersAndBodyAABB(physicsWorld, &body);
    }
}

//==============================================================
// Update Collider Poses and Body AABB
//==============================================================
void internal::updateCollidersAndBodyAABB(
    PhysicsWorld& world,
    RigidBody* body)
{
    if (!body) {
        return;
    }

    bool hasAABB = false;
    internal::AABB combinedAABB;

    for (ColliderHandle colliderHandle : body->colliderHandles) {
        Collider& collider =
            world.getCollider(colliderHandle);

        if (!collider.enabled) {
            continue;
        }

        collider.updateWorldPose(body->pose, body->scale);
        collider.updateShape();
        collider.updateAABB();

        const internal::AABB& colliderAABB = collider.getAABB();

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
