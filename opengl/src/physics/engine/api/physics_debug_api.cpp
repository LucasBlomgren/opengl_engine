#include "pch.h"

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=========================================
// Debug state
//=========================================
DebugData PhysicsEngine::getDebugData() const {
    return impl->getDebugData();
}

PhysicsStepDebugPhase PhysicsEngine::getDebugPhase() const {
    return impl->getDebugPhase();
}

//=========================================
// Debug visualization
//=========================================
void PhysicsEngine::updateBVHRenderData(
    const BVHType& type,
    bool update) {
    impl->updateBVHRenderData(type, update);
}

const std::vector<AABB>&
PhysicsEngine::getDebugSweeps() const {
    return impl->getDebugSweeps();
}

const std::vector<DebugSpeculativeContact>&
PhysicsEngine::getDebugSpeculativeContacts() const {
    return impl->getDebugSpeculativeContacts();
}

const std::unordered_map<size_t, Contact>&
PhysicsEngine::getContactCache() const {
    return impl->getContactCache();
}

//=========================================
// Debug spatial data
//=========================================
const std::vector<RigidBodyHandle>&
PhysicsEngine::getAwakeList() const {
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
