#include "pch.h"
#include "physics_engine.h"
#include "../game/world.h"

PhysicsWorld* PhysicsEngine::getPhysicsWorld() {
    return &physicsWorld;
}
const std::vector<RigidBodyHandle>& PhysicsEngine::getAwakeList() const {
    return broadphaseManager.getAwakeList();
}
const BVHTree& PhysicsEngine::getDynamicAwakeBvh() const {
    return broadphaseManager.getAwakeBVH();
}
const BVHTree& PhysicsEngine::getDynamicAsleepBvh() const {
    return broadphaseManager.getAsleepBVH();
}
const BVHTree& PhysicsEngine::getStaticBvh() const {
return broadphaseManager.getStaticBVH();
}
const TerrainBVH& PhysicsEngine::getTerrainBvh() const {
    return broadphaseManager.getTerrainBVH();
}
const std::unordered_map<size_t, Contact>& PhysicsEngine::GetContactCache() const {
    return contactCache;
}
std::vector<ExternalMotionContact>& PhysicsEngine::getExternalMotionContacts() {
    return narrowphaseManager.getExternalContacts();
}
const DebugData PhysicsEngine::getDebugData() {
    static DebugData debugData;
    debugData.awake = broadphaseManager.getAwakeList().size();
    debugData.asleep = broadphaseManager.getAsleepList().size();
    debugData.Static = broadphaseManager.getStaticList().size();
    debugData.colliders = physicsWorld.getCollidersMap().dense().size();
    debugData.terrainTris = terrainTriangles->size();
    debugData.contacts = contactsGeneratedThisFrame;
    return debugData;
}

RaycastHit PhysicsEngine::performRaycast(Ray& r) {
    SlotMap<RigidBody, RigidBodyHandle>* bodyMap = &physicsWorld.getRigidBodiesMap();
    SlotMap<Collider, ColliderHandle>* colMap = &physicsWorld.getCollidersMap();
    SlotMap<GameObject, GameObjectHandle>* goMap = &world->getGameObjectsMap();
    RaycastHit a = raycast(r, broadphaseManager.getAwakeBVH(), bodyMap, colMap, goMap);
    RaycastHit b = raycast(r, broadphaseManager.getAsleepBVH(), bodyMap, colMap, goMap);
    RaycastHit c = raycast(r, broadphaseManager.getStaticBVH(), bodyMap, colMap, goMap);

    RaycastHit bestHit = a;
    if (b.hit) {
        if (bestHit.hit == false || b.t < bestHit.t) {
            bestHit = b;
        }
    }
    if (c.hit) {
        if (bestHit.hit == false || c.t < bestHit.t) {
            bestHit = c;
        }
    }
    return bestHit;
}