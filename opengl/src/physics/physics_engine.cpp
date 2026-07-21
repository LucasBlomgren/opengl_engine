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
    if (engine.isPaused() && !engine.getAdvanceStep())
        return;

    double physicsStart = glfwGetTime() * 1000.0;

    const std::vector<RigidBodyHandle>& awake =
        broadphaseManager.getAwakeList();

    if (debugPhase == PhysicsStepDebugPhase::PausedBeforePositionIntegration) {
        // Resume second half of the step.
        integratePositionsAndColliders(awake, dt);
        //broadphaseManager.updateBVHs();
        endPhysicsStep(dt);
        debugPhase = PhysicsStepDebugPhase::Ready;

        frameTimers->submit(
            "Physics",
            frameTimers->get("Physics") + glfwGetTime() * 1000.0 - physicsStart + savedPhysicsSurpassedTime
        );
    }

    // 1. Begin step: reset timers, prepare lists and contact cache for the new frame
    beginPhysicsStep(dt);

    // 2. Integrate forces and velocities
    double start = glfwGetTime() * 1000.0;
    integrateForcesAndVelocities(awake, dt);
    frameTimers->submit(
        "Sync",
        frameTimers->get("Sync") + glfwGetTime() * 1000.0 - start
    );

    // 3. Update BVHs
    start = glfwGetTime() * 1000.0;
    broadphaseManager.updateBVHs();
    frameTimers->submit(
        "BVH update",
        frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
    );

    // 4. Build pairs and speculative pairs
    start = glfwGetTime() * 1000.0;
    PairBatch pairs;
    double pairBuildStart = glfwGetTime() * 1000.0;
    broadphaseManager.buildPairs(pairs);
    double pairBuildEnd = glfwGetTime() * 1000.0;
    double pairBuildElapsed = pairBuildEnd - pairBuildStart;
    /*std::cout << "======================================================================" << std::endl;
    std::cout << "BROADPHASE DEBUG INFO" << std::endl;
    std::cout << "Dynamic pairs: " << pairs.dynamicPairs.size() << std::endl;
    std::cout << "buildPairs took " << pairBuildElapsed << " ms" << std::endl;*/
    double speculativePairBuildStart = glfwGetTime() * 1000.0;
    broadphaseManager.buildSpeculativePairs(dt, pairs, debugSweeps);
    double speculativePairBuildEnd = glfwGetTime() * 1000.0;
    double speculativePairBuildElapsed = speculativePairBuildEnd - speculativePairBuildStart;
    /*std::cout << "Speculative dynamic pairs: " << pairs.speculativeDynamicPairs.size() << std::endl;
    std::cout << "buildSpeculativePairs took " << speculativePairBuildElapsed << " ms" << std::endl;
    std::cout << "======================================================================" << std::endl;*/
    frameTimers->submit(
        "Broadphase",
        frameTimers->get("Broadphase") + glfwGetTime() * 1000.0 - start
    );

    // 5. Narrowphase
    start = glfwGetTime() * 1000.0;
    ContactBatch contacts;
    narrowphaseManager.narrowPhase(pairs, contacts, dt);
    contactsGeneratedThisFrame += contacts.contacts.size() + contacts.speculativeContacts.size();
    frameTimers->submit(
        "Narrowphase",
        frameTimers->get("Narrowphase") + glfwGetTime() * 1000.0 - start
    );

    // 6. Sort
    start = glfwGetTime() * 1000.0;
    contacts.sortByMinY();
    frameTimers->submit(
        "Contact collection",
        frameTimers->get("Contact collection") + glfwGetTime() * 1000.0 - start
    );

    // 7. Solver
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

    if (engine.isPaused()) {
        broadphaseManager.updateBVHs();
        debugPhase = PhysicsStepDebugPhase::PausedBeforePositionIntegration;
        savedPhysicsSurpassedTime = glfwGetTime() * 1000.0 - physicsStart;
        return;
    }

    // 8. Integrate positions
    integratePositionsAndColliders(awake, dt);

    // 9. End step: update sleep states, contact cache etc.
    endPhysicsStep(dt);

    frameTimers->submit(
        "Physics",
        frameTimers->get("Physics") + glfwGetTime() * 1000.0 - physicsStart
    );
}

//=====================================================================
//   Begin step: 
//   reset timers, prepare lists and contact cache for the new frame
//=====================================================================
void PhysicsEngine::beginPhysicsStep(float outerDt) {
    double start = glfwGetTime() * 1000.0;
    flushBroadphaseCommands();

    uint32_t bodiesSlotCap = physicsWorld.getRigidBodiesMap().slot_capacity();

    toWake.reserve(bodiesSlotCap);
    toSleep.reserve(bodiesSlotCap);

    toWake.clear();
    toSleep.clear();

    debugSweeps.clear();
    debugSpeculativeContacts.clear();
    contactsGeneratedThisFrame = 0;

    for (auto& [key, contact] : contactCache) {
        contact.wasUsedThisFrame = false;

        for (size_t i = 0; i < contact.numPoints; ++i) {
            ContactPoint& cp = contact.points[i];
            cp.wasUsedThisFrame = false;
            cp.wasWarmStarted = false;
        }
    }

    //double startBvh = glfwGetTime() * 1000.0;
    //broadphaseManager.updateBVHs();
    //frameTimers->submit(
    //    "BVH update",
    //    frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - startBvh
    //);

    frameTimers->submit(
        "Pre step",
        frameTimers->get("Pre step") + glfwGetTime() * 1000.0 - start
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