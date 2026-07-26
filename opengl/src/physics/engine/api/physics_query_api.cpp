#include "pch.h"

#include "physics/engine/physics_engine_impl.h"
#include "game/world.h"

//===============================
// Public facade
//===============================
Raycast::RaycastHit PhysicsEngine::raycast(Raycast::Ray& ray) {
    return impl->raycast(ray);
}

std::vector<ExternalMotionContact>& PhysicsEngine::getExternalMotionContacts() {
    return impl->getExternalMotionContacts();
}

DebugData PhysicsEngine::getDebugData() const {
    return impl->getDebugData();
}

PhysicsStepDebugPhase PhysicsEngine::getDebugPhase() const {
    return impl->debugPhase;
}

float PhysicsEngine::getPausedDt() const {
    return impl->pausedDt;
}

void PhysicsEngine::updateBVHRenderData(const BVHType& type, bool update) {
    impl->updateBVHRenderData(type, update);
}

const std::vector<AABB>& PhysicsEngine::getDebugSweeps() const {
    return impl->debugSweeps;
}

const std::vector<DebugSpeculativeContact>& PhysicsEngine::getDebugSpeculativeContacts() const {
    return impl->debugSpeculativeContacts;
}

const std::unordered_map<size_t, Contact>& PhysicsEngine::getContactCache() const {
    return impl->getContactCache();
}

const std::vector<RigidBodyHandle>& PhysicsEngine::getAwakeList() const {
    return impl->getAwakeList();
}

const BVHTree& PhysicsEngine::getDynamicAwakeBvh() const {
    return impl->getDynamicAwakeBvh();
}

const BVHTree& PhysicsEngine::getDynamicAsleepBvh() const {
    return impl->getDynamicAsleepBvh();
}

const BVHTree& PhysicsEngine::getStaticBvh() const {
    return impl->getStaticBvh();
}

const TerrainBVH& PhysicsEngine::getTerrainBvh() const {
    return impl->getTerrainBvh();
}

PhysicsWorld* PhysicsEngine::getPhysicsWorld() {
    return impl->getPhysicsWorld();
}

//===============================
// Public state queries
//===============================
std::optional<RigidBodyState> PhysicsEngine::getRigidBodyState(
    RigidBodyHandle body) const {
    return impl->getRigidBodyState(body);
}

std::optional<ColliderState> PhysicsEngine::getColliderState(
    ColliderHandle collider) const {
    return impl->getColliderState(collider);
}

//===============================
// Public facade and implementation: physics queries
//===============================
Raycast::RaycastHit PhysicsEngine::Impl::raycast(Raycast::Ray& ray) {
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

//===============================
// Simulation output
//===============================
std::vector<ExternalMotionContact>& PhysicsEngine::Impl::getExternalMotionContacts() {
    return narrowphaseManager.getExternalContacts();
}

//===============================
// State queries
//===============================
std::optional<RigidBodyState> PhysicsEngine::Impl::getRigidBodyState(
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

std::optional<ColliderState> PhysicsEngine::Impl::getColliderState(
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

//===============================
// Debug state
//===============================
DebugData PhysicsEngine::Impl::getDebugData() const {
    DebugData debugData;

    debugData.awake = broadphaseManager.getAwakeList().size();
    debugData.asleep = broadphaseManager.getAsleepList().size();
    debugData.staticBodies = broadphaseManager.getStaticList().size();
    debugData.colliders = physicsWorld.getCollidersMap().dense().size();
    debugData.terrainTris = terrainTriangles ? terrainTriangles->size() : 0;
    debugData.contacts = contactsGeneratedThisFrame;

    return debugData;
}

void PhysicsEngine::Impl::updateBVHRenderData(
    const BVHType& type,
    bool update) {
    broadphaseManager.updateBVHRenderData(type, update);
}

//===============================
// Debug spatial data
//===============================
const std::vector<RigidBodyHandle>& PhysicsEngine::Impl::getAwakeList() const {
    return broadphaseManager.getAwakeList();
}

const BVHTree& PhysicsEngine::Impl::getDynamicAwakeBvh() const {
    return broadphaseManager.getAwakeBVH();
}

const BVHTree& PhysicsEngine::Impl::getDynamicAsleepBvh() const {
    return broadphaseManager.getAsleepBVH();
}

const BVHTree& PhysicsEngine::Impl::getStaticBvh() const {
    return broadphaseManager.getStaticBVH();
}

const TerrainBVH& PhysicsEngine::Impl::getTerrainBvh() const {
    return broadphaseManager.getTerrainBVH();
}

const std::unordered_map<size_t, Contact>& PhysicsEngine::Impl::getContactCache() const {
    return contactCache;
}

//===============================
// Temporary legacy API
//===============================
PhysicsWorld* PhysicsEngine::Impl::getPhysicsWorld() {
    return &physicsWorld;
}
