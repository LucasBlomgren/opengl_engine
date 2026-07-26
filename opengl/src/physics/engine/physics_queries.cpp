#include "pch.h"

#include "physics/engine/physics_engine_impl.h"
#include "game/world.h"

//=========================================
// Spatial queries
//=========================================
Raycast::RaycastHit PhysicsEngine::Impl::raycast(
    Raycast::Ray& ray) {
    SlotMap<RigidBody, RigidBodyHandle>* bodyMap =
        &physicsWorld.getRigidBodiesMap();

    SlotMap<Collider, ColliderHandle>* colliderMap =
        &physicsWorld.getCollidersMap();

    SlotMap<GameObject, GameObjectHandle>* gameObjectMap =
        &world->getGameObjectsMap();

    Raycast::RaycastHit awakeHit =
        Raycast::raycast(
            ray,
            broadphaseManager.getAwakeBVH(),
            bodyMap,
            colliderMap,
            gameObjectMap
        );

    Raycast::RaycastHit asleepHit =
        Raycast::raycast(
            ray,
            broadphaseManager.getAsleepBVH(),
            bodyMap,
            colliderMap,
            gameObjectMap
        );

    Raycast::RaycastHit staticHit =
        Raycast::raycast(
            ray,
            broadphaseManager.getStaticBVH(),
            bodyMap,
            colliderMap,
            gameObjectMap
        );

    Raycast::RaycastHit bestHit = awakeHit;

    if (asleepHit.hit && (!bestHit.hit || asleepHit.t < bestHit.t)) {
        bestHit = asleepHit;
    }

    if (staticHit.hit && (!bestHit.hit || staticHit.t < bestHit.t)) {
        bestHit = staticHit;
    }

    return bestHit;
}

//=========================================
// State queries
//=========================================
std::optional<RigidBodyState>
PhysicsEngine::Impl::getRigidBodyState(
    RigidBodyHandle handle) const
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || commandBuffer.isBodyPendingDestroy(handle)) {
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

std::optional<ColliderState>
PhysicsEngine::Impl::getColliderState(
    ColliderHandle handle) const
{
    const Collider* collider = physicsWorld.getCollider(handle);

    if (!collider ||
        commandBuffer.isColliderPendingDestroy(handle) ||
        commandBuffer.isBodyPendingDestroy(collider->rigidBodyHandle)) {
        return std::nullopt;
    }

    ColliderState state;
    state.body = collider->rigidBodyHandle;
    state.localPose = collider->localPose;
    state.localScale = collider->localScale;
    state.worldPose.position = collider->worldPose.position;
    state.worldPose.orientation = collider->worldPose.orientation;
    state.worldScale = collider->worldScale;
    state.enabled = collider->enabled;
    state.isTrigger = collider->isTrigger;
    state.userTag = collider->userTag;

    return state;
}

//=========================================
// Simulation output
//=========================================
std::vector<ExternalMotionContact>&
PhysicsEngine::Impl::getExternalMotionContacts() {
    return narrowphaseManager.getExternalContacts();
}
