#include "pch.h"

#include "physics/engine/physics_engine_impl.h"
#include "game/world.h"

PhysicsWorld* PhysicsEngine::Impl::getPhysicsWorld() {
    return &physicsWorld;
}

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

std::vector<ExternalMotionContact>& PhysicsEngine::Impl::getExternalMotionContacts() {
    return narrowphaseManager.getExternalContacts();
}

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

RaycastHit PhysicsEngine::Impl::performRaycast(Ray& ray) {
    SlotMap<RigidBody, RigidBodyHandle>* bodyMap = &physicsWorld.getRigidBodiesMap();
    SlotMap<Collider, ColliderHandle>* colliderMap = &physicsWorld.getCollidersMap();
    SlotMap<GameObject, GameObjectHandle>* gameObjectMap = &world->getGameObjectsMap();

    RaycastHit awakeHit = raycast(ray, broadphaseManager.getAwakeBVH(), bodyMap, colliderMap, gameObjectMap);
    RaycastHit asleepHit = raycast(ray, broadphaseManager.getAsleepBVH(), bodyMap, colliderMap, gameObjectMap);
    RaycastHit staticHit = raycast(ray, broadphaseManager.getStaticBVH(), bodyMap, colliderMap, gameObjectMap);

    RaycastHit bestHit = awakeHit;

    if (asleepHit.hit && (!bestHit.hit || asleepHit.t < bestHit.t)) {
        bestHit = asleepHit;
    }

    if (staticHit.hit && (!bestHit.hit || staticHit.t < bestHit.t)) {
        bestHit = staticHit;
    }

    return bestHit;
}