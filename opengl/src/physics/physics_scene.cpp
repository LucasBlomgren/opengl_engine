#include "pch.h"
#include "physics_engine.h"
#include "game/world.h"

void PhysicsEngine::setupScene(std::vector<Tri>* terrainTris) {
    this->terrainTriangles = terrainTris;

    caches.transforms.init(world->getTransformsMap(), "Transform");
    caches.colliders.init(physicsWorld.getCollidersMap(), "Collider");
    caches.bodies.init(physicsWorld.getRigidBodiesMap(), "RigidBody");

    uint32_t slotCap = physicsWorld.getCollidersMap().slot_capacity();
    toWake.reserve(slotCap);
    toSleep.reserve(slotCap);

    broadphaseManager.init(&physicsWorld, &caches, terrainTris);
    narrowphaseManager.init(collisionManifold, &contactCache, &caches, &toWake);

    flushBroadphaseCommands();
}

void PhysicsEngine::clear() {
    toWake.clear();
    contactCache.clear();

    physicsWorld.clear();
    broadphaseManager.clear();
    narrowphaseManager.clear();
    contactsToSolve.clear();
    pending.clear();
    pending.reserve(50000);
}