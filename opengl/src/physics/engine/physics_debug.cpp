#include "pch.h"

#include "physics/engine/physics_engine_impl.h"

//=========================================
// Debug state
//=========================================
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

PhysicsStepDebugPhase
PhysicsEngine::Impl::getDebugPhase() const {
    return debugPhase;
}

//=========================================
// Debug visualization
//=========================================
void PhysicsEngine::Impl::updateBVHRenderData(
    const BVHType& type,
    bool update) {
    broadphaseManager.updateBVHRenderData(type, update);
}

const std::vector<AABB>&
PhysicsEngine::Impl::getDebugSweeps() const {
    return debugSweeps;
}

const std::vector<DebugSpeculativeContact>&
PhysicsEngine::Impl::getDebugSpeculativeContacts() const {
    return debugSpeculativeContacts;
}

//=========================================
// Debug spatial data
//=========================================
const std::vector<RigidBodyHandle>&
PhysicsEngine::Impl::getAwakeList() const {
    return broadphaseManager.getAwakeList();
}

const BVHTree&
PhysicsEngine::Impl::getDynamicAwakeBvh() const {
    return broadphaseManager.getAwakeBVH();
}

const BVHTree&
PhysicsEngine::Impl::getDynamicAsleepBvh() const {
    return broadphaseManager.getAsleepBVH();
}

const BVHTree&
PhysicsEngine::Impl::getStaticBvh() const {
    return broadphaseManager.getStaticBVH();
}

const TerrainBVH&
PhysicsEngine::Impl::getTerrainBvh() const {
    return broadphaseManager.getTerrainBVH();
}

const std::unordered_map<size_t, Contact>&
PhysicsEngine::Impl::getContactCache() const {
    return contactCache;
}
