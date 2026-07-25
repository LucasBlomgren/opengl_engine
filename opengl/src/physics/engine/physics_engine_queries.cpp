#include "pch.h"

#include "physics/engine/physics_engine_impl.h"
#include "game/world.h"

//===============================
// Physics queries
//===============================
Raycast::RaycastHit PhysicsEngine::Impl::raycast(Raycast::Ray& ray) {
    SlotMap<RigidBody, RigidBodyHandle>* bodyMap = &physicsWorld.getRigidBodiesMap();
    SlotMap<Collider, ColliderHandle>* colliderMap = &physicsWorld.getCollidersMap();
    SlotMap<GameObject, GameObjectHandle>* gameObjectMap = &world->getGameObjectsMap();

    Raycast::RaycastHit awakeHit = Raycast::raycast(ray, broadphaseManager.getAwakeBVH(), bodyMap, colliderMap, gameObjectMap);
    Raycast::RaycastHit asleepHit = Raycast::raycast(ray, broadphaseManager.getAsleepBVH(), bodyMap, colliderMap, gameObjectMap);
    Raycast::RaycastHit staticHit = Raycast::raycast(ray, broadphaseManager.getStaticBVH(), bodyMap, colliderMap, gameObjectMap);

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