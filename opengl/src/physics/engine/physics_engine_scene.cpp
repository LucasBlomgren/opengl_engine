#include "pch.h"

#include "physics/engine/physics_engine_impl.h"
#include "game/world.h"

void PhysicsEngine::Impl::setupScene(std::vector<Tri>* terrainTris) {
    terrainTriangles = terrainTris;

    caches.transforms.init(world->getTransformsMap(), "Transform");
    caches.colliders.init(physicsWorld.getCollidersMap(), "Collider");
    caches.bodies.init(physicsWorld.getRigidBodiesMap(), "RigidBody");

    uint32_t slotCap = physicsWorld.getCollidersMap().slot_capacity();

    toWake.reserve(slotCap);
    toSleep.reserve(slotCap);

    broadphaseManager.init(&physicsWorld, &caches, terrainTris);
    narrowphaseManager.init(std::make_unique<CollisionManifold>(), &debugSpeculativeContacts, &contactCache, &caches, &toWake);

    flushBroadphaseCommands();
}

void PhysicsEngine::Impl::clear() {
    toWake.clear();
    toSleep.clear();

    contactCache.clear();
    debugSweeps.clear();
    debugSpeculativeContacts.clear();

    broadphaseManager.clear();
    narrowphaseManager.clear();
    pgsSolver.clear();

    pending.clear();

    physicsWorld.clear();
}