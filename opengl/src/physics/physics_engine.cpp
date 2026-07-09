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
void PhysicsEngine::step(float dt, EngineState& engine) {
    double physicsStart = glfwGetTime() * 1000.0;

    if (!physicsFrameActive) {
        ScopedTimer t(*frameTimers, "Pre step");
        physicsFrameActive = true;
        schedulerSubstep = 0;
        highestSubstepIslandCount = 0;

        beginPhysicsStep(dt);
        createPredictedIslandsMVP(dt);

        for (const Island& island : predictedIslands) {
            highestSubstepIslandCount = std::max(highestSubstepIslandCount, island.substeps);
        }

        if (engine.isPaused()) {
            std::cout << "Physics frame started | highest substep count=" << highestSubstepIslandCount << " | islands=" << predictedIslands.size() << std::endl;
            schedulerSubstep = -1; // special value to indicate that we are paused and waiting for user input to advance
        }
    }

    if (engine.isPaused() && !engine.getAdvanceStep()) {
        frameTimers->submit(
            "Physics",
            frameTimers->get("Physics") + glfwGetTime() * 1000.0 - physicsStart
        );
        return;
    }

    // If we are paused and the user has requested to advance one step, run one substep and then pause again
    if (schedulerSubstep == -1) {
        schedulerSubstep = 0;
        return;
    }

    bool keepRunning = true;
    while (keepRunning && physicsFrameActive) {
        // Run one substep for each island that has not yet completed its substeps
        if (schedulerSubstep < highestSubstepIslandCount) {
            for (const Island& island : predictedIslands) {
                if (schedulerSubstep >= island.substeps)
                    continue;

                StepScope islandScope;
                islandScope.type = StepScopeType::Island;
                islandScope.bodies = &island.bodies;
                islandScope.islandId = island.id;
                islandScope.islandBroadphaseMode = IslandBroadphaseMode::SAP;

                float islandH = dt / static_cast<float>(island.substeps);

                stepScope(islandScope, islandH);
            }

            float start = glfwGetTime() * 1000.0;
            broadphaseManager.updateBVHs();
            frameTimers->submit(
                "BVH update",
                frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
            );

            schedulerSubstep++;

            if (engine.isPaused())
                std::cout << "Scheduler substep " << schedulerSubstep << "/" << highestSubstepIslandCount << " completed." << std::endl;
        }

        // All islands have completed their substeps, 
        // now do the rest step for all remaining bodies
        if (schedulerSubstep >= highestSubstepIslandCount) {
            StepScope restScope;
            restScope.type = StepScopeType::Rest;
            restScope.bodies = &restBodies;

            stepScope(restScope, dt);

            float start = glfwGetTime() * 1000.0;
            broadphaseManager.updateBVHs();
            frameTimers->submit(
                "BVH update",
                frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
            );

            endPhysicsStep(dt);

            physicsFrameActive = false;
            schedulerSubstep = 0;
            highestSubstepIslandCount = 0;
        }

        if (engine.isPaused()) {
            keepRunning = false;
        }
    }

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
    flushBroadphaseCommands();

    uint32_t bodiesSlotCap = physicsWorld.getRigidBodiesMap().slot_capacity();

    toWake.reserve(bodiesSlotCap);
    toSleep.reserve(bodiesSlotCap);

    toWake.clear();
    toSleep.clear();

    for (auto& [key, contact] : contactCache) {
        contact.wasUsedThisFrame = false;

        for (ContactPoint& cp : contact.points) {
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

    debugSweeps.clear();
}

//===================================
//    Step scope
//===================================
void PhysicsEngine::stepScope(StepScope& scope, float h) {
    this->dt = h;

    auto updateTimer = [&](const std::string& name, double start) {
        frameTimers->submit(name, frameTimers->get(name) + glfwGetTime() * 1000.0 - start);
        };

    double start = 0.0f;

    // sync bodies and colliders
    start = glfwGetTime() * 1000.0;
    updateBodiesAndColliders(*scope.bodies, h);
    updateTimer("Sync", start);

    broadphaseManager.updateBVHs();
    frameTimers->submit(
        "BVH update",
        frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
    );

    // broadphase
    start = glfwGetTime() * 1000.0;
    PairBatch pairs;
    pairs.dynamicPairs.reserve(scope.bodies->size() * 2);
    buildPairsForScope(scope, pairs);
    updateTimer("Broadphase", start);

    // narrowphase
    start = glfwGetTime() * 1000.0;
    ContactBatch contacts;
    narrowphaseManager.narrowPhase(pairs, contacts);
    updateTimer("Narrowphase", start);

    // sort contacts by minY for better stability of resting contacts
    start = glfwGetTime() * 1000.0;
    contacts.sortByMinY();
    updateTimer("Contact collection", start);

    // solve contacts
    start = glfwGetTime() * 1000.0;
    pgsSolver.solve(scope, contacts, caches, pgsIterations, h);
    updateTimer("Collision resolution", start);

    processWakeList();
}

//================================================================
//    End step: update sleep states, contact cache etc.
//================================================================
void PhysicsEngine::endPhysicsStep(float outerDt) {
    double start = glfwGetTime() * 1000.0;

    //processWakeList(); // extra säkerhet om något ligger kvar

    updateSleepThresholds();
    processSleepList(outerDt);
    addSleepDamping();
    updateContactCache();

    frameTimers->submit(
        "Post step",
        frameTimers->get("Post step") + glfwGetTime() * 1000.0 - start
    );
}