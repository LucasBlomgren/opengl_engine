#include "pch.h"

#include "physics/engine/physics_engine_impl.h"
#include "physics/raycast/raycast.h"

namespace physics::internal {

namespace {
    physics::AABB toPublicBounds(const AABB& bounds) {
        physics::AABB result;
        result.worldMin = bounds.worldMin;
        result.worldMax = bounds.worldMax;
        result.worldCenter = bounds.worldCenter;
        result.worldHalfExtents = bounds.worldHalfExtents;
        return result;
    }

    AABB toInternalBounds(const physics::AABB& bounds) {
        AABB result;
        result.worldMin = bounds.worldMin;
        result.worldMax = bounds.worldMax;
        result.worldCenter = bounds.worldCenter;
        result.worldHalfExtents = bounds.worldHalfExtents;
        return result;
    }
}

//=========================================
// Spatial queries
//=========================================
RaycastHit EngineImpl::raycast(
    const Ray& ray,
    BodyHandle ignoredBody)
{
    const SlotMap<RigidBody, BodyHandle>& bodyMap =
        physicsWorld.getRigidBodiesMap();

    RaycastHit bestHit;

    const BVHTree* trees[] = {
        &broadphaseManager.getAwakeBVH(),
        &broadphaseManager.getAsleepBVH(),
        &broadphaseManager.getStaticBVH()
    };

    for (const BVHTree* tree : trees) {
        RaycastHit hit = raycast::raycastTree(
            ray,
            *tree,
            bodyMap,
            ignoredBody
        );

        if (hit.hit && (!bestHit.hit || hit.t < bestHit.t)) {
            bestHit = hit;
        }
    }

    return bestHit;
}

std::vector<BodyHandle> EngineImpl::queryBodies(
    const physics::AABB& bounds,
    BodySet bodySet) const
{
    const BVHTree* tree = nullptr;

    switch (bodySet) {
    case BodySet::Awake:
        tree = &broadphaseManager.getAwakeBVH();
        break;
    case BodySet::Asleep:
        tree = &broadphaseManager.getAsleepBVH();
        break;
    case BodySet::Static:
        tree = &broadphaseManager.getStaticBVH();
        break;
    }

    std::vector<BodyHandle> result;

    if (tree) {
        tree->singleQuery(toInternalBounds(bounds), result);
    }

    return result;
}

//=========================================
// State queries
//=========================================
std::optional<BodyState>
EngineImpl::getRigidBodyState(
    BodyHandle handle) const
{
    const RigidBody* body = physicsWorld.getRigidBody(handle);

    if (!body || commandBuffer.isBodyPendingDestroy(handle)) {
        return std::nullopt;
    }

    BodyState state;
    state.pose = body->pose;
    state.scale = body->scale;
    state.linearVelocity = body->linearVelocity;
    state.angularVelocity = body->angularVelocity;
    state.type = body->type;
    state.motionControl = body->motionControl;
    state.responseMode = body->responseMode;
    state.mass = body->mass;
    state.asleep = body->asleep;
    state.allowSleep = body->allowSleep;
    state.allowGravity = body->allowGravity;
    state.canMoveLinearly = body->canMoveLinearly;
    state.bounds = toPublicBounds(body->aabb);
    state.colliders = body->colliderHandles;
    return state;
}

std::optional<ColliderState>
EngineImpl::getColliderState(
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
    state.worldPose = collider->worldPose;
    state.worldScale = collider->worldScale;
    state.type = collider->type;
    state.bounds = toPublicBounds(collider->aabb);
    state.enabled = collider->enabled;
    state.isTrigger = collider->isTrigger;
    state.userTag = collider->userTag;

    if (collider->type == ColliderType::CUBOID) {
        const OOBB& box = std::get<OOBB>(collider->shape);
        state.shape = BoxGeometry{
            box.worldCenter,
            box.localHalfExtents,
            box.scale
        };
    }
    else {
        const Sphere& sphere = std::get<Sphere>(collider->shape);
        state.shape = SphereGeometry{
            sphere.centerWorld,
            sphere.radiusWorld
        };
    }

    return state;
}

//=========================================
// Simulation output
//=========================================
std::vector<ExternalMotionContact>&
EngineImpl::getExternalMotionContacts() {
    return narrowphaseManager.getExternalContacts();
}

}
