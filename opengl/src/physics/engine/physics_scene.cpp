#include "pch.h"

#include "physics/engine/physics_engine_impl.h"
#include "game/world.h"

//====================================
// Internal scene setup
//====================================
void PhysicsEngine::Impl::setupScene(std::vector<Tri>* terrainTris) {
    terrainTriangles = terrainTris;

    caches.transforms.init(world->getTransformsMap(), "Transform");
    caches.colliders.init(physicsWorld.getCollidersMap(), "Collider");
    caches.bodies.init(physicsWorld.getRigidBodiesMap(), "RigidBody");

    uint32_t colliderSlotCapacity = 
        physicsWorld.getCollidersMap().slot_capacity();

    toWake.reserve(colliderSlotCapacity);
    toSleep.reserve(colliderSlotCapacity);

    broadphaseManager.init(&physicsWorld, &caches, terrainTris);

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
void PhysicsEngine::Impl::clear() {
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
}

//====================================
// Scene-wide command submission
//====================================
void PhysicsEngine::Impl::submitSleepAllObjects() {
    commandBuffer.recordSleepAllObjects();
}

void PhysicsEngine::Impl::submitAwakenAllObjects() {
    commandBuffer.recordAwakenAllObjects();
}

//====================================
// Temporary legacy submission
//====================================
void PhysicsEngine::Impl::submitSyncBodyFromTransform(
    RigidBodyHandle body) {
    commandBuffer.recordSyncBodyFromTransform(body);
}

//====================================
// BVH management
//====================================
void PhysicsEngine::Impl::setBVHDirty(
    RigidBodyHandle handle) {
    broadphaseManager.setBVHDirty(handle);
}
