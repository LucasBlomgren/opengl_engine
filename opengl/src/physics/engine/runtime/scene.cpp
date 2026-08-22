#include "pch.h"

#include "physics/public/engine.h"

namespace physics {

using namespace internal;

//====================================
// Internal scene setup
//====================================
void Engine::activateScene(
    const std::vector<Triangle>& terrainInput) 
{
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

    uint32_t colliderSlotCapacity = 
        physicsWorld.colliderStorage().slot_capacity();

    toWake.reserve(colliderSlotCapacity);
    toSleep.reserve(colliderSlotCapacity);

    broadphaseManager.init(
        &physicsWorld,
        &terrainTriangles
    );

    narrowphaseManager.init(
        &physicsWorld,
        &collisionManifold,
        &debugSpeculativeContacts, 
        &contactCache, 
        &toWake
    );

    pgsSolver.init(physicsWorld);

    processPendingCommands();
}

//====================================
// Scene cleanup
//====================================
void Engine::clear() {
    commandBuffer.clear(physicsWorld);

    toWake.clear();
    toSleep.clear();

    contactCache.clear();
    debugSweeps.clear();
    debugSpeculativeContacts.clear();

    broadphaseManager.clear();
    narrowphaseManager.clear();
    pgsSolver.clear();

    physicsWorld.clear();
    terrainTriangles.clear();
}

}
