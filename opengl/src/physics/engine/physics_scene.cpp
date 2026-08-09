#include "pch.h"

#include "physics/physics_engine.h"

namespace physics {

using namespace internal;

//====================================
// Internal scene setup
//====================================
void Engine::setupScene(
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
void Engine::clear() {
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
void Engine::sleepAllObjects() {
    commandBuffer.recordSleepAllObjects();
}

void Engine::awakenAllObjects() {
    commandBuffer.recordAwakenAllObjects();
}

//====================================
// BVH management
//====================================
void Engine::setBVHDirty(
    BodyHandle handle) {
    broadphaseManager.setBVHDirty(handle);
}

}
