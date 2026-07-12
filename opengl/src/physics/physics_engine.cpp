#include "pch.h"
#include "physics_engine.h"

#include "engine/engine_state.h"
#include "world.h"
#include "broadphase/broadphase_types.h"

//===================================
//    Initialization 
//===================================
void PhysicsEngine::init(World* world, FrameTimers* ft) {
    this->world = world;
    this->frameTimers = ft;
    collisionManifold = new CollisionManifold();
    collisionManifold->init(&caches);

    pending.reserve(50000);
}

//===================================
// Prepare for stepping loop 
//===================================
void PhysicsEngine::prepareStepLoop() {
    // only clear pointer caches before accumulator loop, since objects are not supposed to be added/removed during accumulator step loop,
    // and if cache is cleared multiple times during the loop, it will cause redundant lookups in the slot maps and thus worse performance.
    caches.clear();

    frameTimers->reset("Physics");
    frameTimers->reset("Pre step");
    frameTimers->reset("Sync");
    frameTimers->reset("BVH update");
    frameTimers->reset("Broadphase");
    frameTimers->reset("Narrowphase");
    frameTimers->reset("Contact collection");
    frameTimers->reset("Collision resolution");
    frameTimers->reset("Post step");
}

//====================================
//         Step
//====================================
void PhysicsEngine::step(float dt, EngineState& engine)
{
    double physicsStart = glfwGetTime() * 1000.0;

    beginPhysicsStep(dt);

    const std::vector<RigidBodyHandle>& awake =
        broadphaseManager.getAwakeList();

    // 1. Integrera/synca alla awake bodies
    double start = glfwGetTime() * 1000.0;
    updateBodiesAndColliders(awake, dt);
    frameTimers->submit(
        "Sync",
        frameTimers->get("Sync") + glfwGetTime() * 1000.0 - start
    );

    // 2. Uppdatera BVH efter sync
    start = glfwGetTime() * 1000.0;
    broadphaseManager.updateBVHs();
    frameTimers->submit(
        "BVH update",
        frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
    );

    // 3. Vanliga pairs + speculative pairs
    start = glfwGetTime() * 1000.0;
    PairBatch pairs;
    broadphaseManager.buildPairs(pairs);
    broadphaseManager.buildSpeculativePairs(dt, pairs, debugSweeps);
    frameTimers->submit(
        "Broadphase",
        frameTimers->get("Broadphase") + glfwGetTime() * 1000.0 - start
    );

    // 4. Narrowphase
    start = glfwGetTime() * 1000.0;
    ContactBatch contacts;
    narrowphaseManager.narrowPhase(pairs, contacts, dt);
    frameTimers->submit(
        "Narrowphase",
        frameTimers->get("Narrowphase") + glfwGetTime() * 1000.0 - start
    );

    // 5. Sort
    start = glfwGetTime() * 1000.0;
    contacts.sortByMinY();
    frameTimers->submit(
        "Contact collection",
        frameTimers->get("Contact collection") + glfwGetTime() * 1000.0 - start
    );

    // 6. Solver
    start = glfwGetTime() * 1000.0;
    pgsSolver.solve(
        contacts,
        caches,
        pgsIterations,
        dt
    );
    frameTimers->submit(
        "Collision resolution",
        frameTimers->get("Collision resolution") + glfwGetTime() * 1000.0 - start
    );

    endPhysicsStep(dt);

    frameTimers->submit(
        "Physics",
        frameTimers->get("Physics") + glfwGetTime() * 1000.0 - physicsStart
    );

    contactsGeneratedThisFrame += contacts.size();
}

//=====================================================================
//   Begin step: 
//   reset timers, prepare lists and contact cache for the new frame
//=====================================================================
void PhysicsEngine::beginPhysicsStep(float outerDt) {
    flushBroadphaseCommands();

    uint32_t bodiesSlotCap = physicsWorld.getRigidBodiesMap().slot_capacity();

    toWake.reserve(bodiesSlotCap);
    toSleep.reserve(bodiesSlotCap);

    toWake.clear();
    toSleep.clear();

    debugSweeps.clear();
    contactsGeneratedThisFrame = 0;

    for (auto& [key, contact] : contactCache) {
        contact.wasUsedThisFrame = false;

        for (size_t i = 0; i < contact.numPoints; ++i) {
            ContactPoint& cp = contact.points[i];
            cp.wasUsedThisFrame = false;
            cp.wasWarmStarted = false;
        }
    }

    double start = glfwGetTime() * 1000.0;
    broadphaseManager.updateBVHs();
    frameTimers->submit(
        "BVH update",
        frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
    );
}

//================================================================
//    End step: update sleep states, contact cache etc.
//================================================================
void PhysicsEngine::endPhysicsStep(float outerDt) {
    double start = glfwGetTime() * 1000.0;

    processWakeList();
    updateSleepThresholds();
    processSleepList(outerDt);
    addSleepDamping();
    updateContactCache();

    frameTimers->submit(
        "Post step",
        frameTimers->get("Post step") + glfwGetTime() * 1000.0 - start
    );
}