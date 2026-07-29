#include "pch.h"

#include "physics/engine/physics_engine_impl.h"

namespace physics::internal {

//====================================
// Internal scene setup
//====================================
void EngineImpl::setupScene(
    const std::vector<Triangle>& terrainInput) {
    terrainTriangles.clear();
    terrainTriangles.reserve(terrainInput.size());

    for (const Triangle& triangle : terrainInput) {
        terrainTriangles.emplace_back(
            triangle.id,
            triangle.vertices[0],
            triangle.vertices[1],
            triangle.vertices[2]
        );
    }

    caches.colliders.init(physicsWorld.getCollidersMap(), "Collider");
    caches.bodies.init(physicsWorld.getRigidBodiesMap(), "RigidBody");

    uint32_t colliderSlotCapacity = 
        physicsWorld.getCollidersMap().slot_capacity();

    toWake.reserve(colliderSlotCapacity);
    toSleep.reserve(colliderSlotCapacity);

    broadphaseManager.init(
        &physicsWorld,
        &caches,
        &terrainTriangles
    );

    narrowphaseManager.init(
        std::make_unique<CollisionManifold>(), 
        &debugSpeculativeContacts, 
        &contactCache, 
        &caches, 
        &toWake
    );
}

//====================================
// Scene cleanup
//====================================
void EngineImpl::clear() {
    commandBuffer.clear();

    toWake.clear();
    toSleep.clear();

    contactCache.clear();
    debugSweeps.clear();
    debugSpeculativeContacts.clear();

    broadphaseManager.clear();
    narrowphaseManager.clear();
    pgsSolver.clear();

    physicsWorld.clear();
    caches.clear();
    terrainTriangles.clear();
}

//====================================
// Scene-wide command submission
//====================================
void EngineImpl::submitSleepAllObjects() {
    commandBuffer.recordSleepAllObjects();
}

void EngineImpl::submitAwakenAllObjects() {
    commandBuffer.recordAwakenAllObjects();
}

//====================================
// BVH management
//====================================
void EngineImpl::setBVHDirty(
    BodyHandle handle) {
    broadphaseManager.setBVHDirty(handle);
}

}
