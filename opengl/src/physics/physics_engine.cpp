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

    // Initialize one new outer physics frame.
    if (!physicsFrameActive) {
        ScopedTimer t(*frameTimers, "Pre step");

        physicsFrameActive = true;
        schedulerSubstep = 0;
        highestSubstepIslandCount = 0;

        beginPhysicsStep(dt);

        if (stepMode == StepMode::Global) {
            currentSubstepAmount = computeGlobalSubsteps(dt);

            debugSweeps.clear();
            islandScopes.clear();

            restScope = PhysicsScope{};
            restScope.type = PhysicsScopeType::Rest;
            restScope.substeps = currentSubstepAmount;
            //restScope.bodies = broadphaseManager.getAwakeList();

            highestSubstepIslandCount = currentSubstepAmount;
        }
        else {
            createPredictedIslandsMVP(dt);

            for (const PhysicsScope& scope : islandScopes) {
                highestSubstepIslandCount = std::max(
                    highestSubstepIslandCount,
                    scope.substeps
                );
            }
        }

        if (engine.isPaused()) {
            std::cout
                << "Physics frame started | highest substep count="
                << highestSubstepIslandCount
                << " | islands="
                << islandScopes.size()
                << '\n';

            schedulerSubstep = -1;
        }
    }

    if (engine.isPaused() && !engine.getAdvanceStep()) {
        frameTimers->submit(
            "Physics",
            frameTimers->get("Physics") +
            glfwGetTime() * 1000.0 -
            physicsStart
        );

        return;
    }

    if (schedulerSubstep == -1) {
        schedulerSubstep = 0;
        return;
    }

    bool keepRunning = true;

    while (keepRunning && physicsFrameActive) {
        if (stepMode == StepMode::Global) {
            // Run the entire awake world once per global substep.
            if (schedulerSubstep < currentSubstepAmount) {
                float h =
                    dt / static_cast<float>(currentSubstepAmount);

                stepScope(restScope, h);

                ++schedulerSubstep;

                if (engine.isPaused()) {
                    std::cout
                        << "Global substep "
                        << schedulerSubstep
                        << "/"
                        << currentSubstepAmount
                        << " completed.\n";
                }
            }

            if (schedulerSubstep >= currentSubstepAmount) {
                endPhysicsStep(dt);

                physicsFrameActive = false;
                schedulerSubstep = 0;
                highestSubstepIslandCount = 0;
            }
        }
        else {
            // Adaptive island substeps.
            if (schedulerSubstep < highestSubstepIslandCount) {
                for (PhysicsScope& scope : islandScopes) {
                    if (schedulerSubstep >= scope.substeps) {
                        continue;
                    }

                    float h =
                        dt / static_cast<float>(scope.substeps);

                    stepScope(scope, h);
                }

                ++schedulerSubstep;

                if (engine.isPaused()) {
                    std::cout
                        << "Scheduler substep "
                        << schedulerSubstep
                        << "/"
                        << highestSubstepIslandCount
                        << " completed.\n";
                }
            }

            // Run non-island bodies once with the outer timestep.
            if (schedulerSubstep >= highestSubstepIslandCount) {
                stepScope(restScope, dt);
                endPhysicsStep(dt);

                physicsFrameActive = false;
                schedulerSubstep = 0;
                highestSubstepIslandCount = 0;
            }
        }

        if (engine.isPaused()) {
            keepRunning = false;
        }
    }

    frameTimers->submit(
        "Physics",
        frameTimers->get("Physics") +
        glfwGetTime() * 1000.0 -
        physicsStart
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

    debugSweeps.clear();
    islandScopes.clear();

    contactsGeneratedThisFrame = 0;
}

//===================================
//    Step scope
//===================================
void PhysicsEngine::stepScope(
    PhysicsScope& scope,
    float h)
{
    this->dt = h;

    auto updateTimer = [&](const std::string& name, double start) {
        frameTimers->submit(
            name,
            frameTimers->get(name) +
            glfwGetTime() * 1000.0 -
            start);
        };

    double start = glfwGetTime() * 1000.0;

    // Sync bodies and colliders
    if (scope.type == PhysicsScopeType::Rest) {
        updateBodiesAndColliders(broadphaseManager.getAwakeList(), h);
    } else {
        updateBodiesAndColliders(scope.bodies, h);
    }
    updateTimer("Sync", start);

    start = glfwGetTime() * 1000.0;
    broadphaseManager.updateBVHs();
    frameTimers->submit(
        "BVH update",
        frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
    );

    // Broadphase
    start = glfwGetTime() * 1000.0;

    if (!scope.internalSapBuilt && stepMode != StepMode::Global) {
        // Build the internal SAP for this scope if it hasn't been built yet.
        if (scope.bodies.size() > 16) {
            broadphaseManager.buildSAP(
                scope.internalSap,
                scope.bodies
            );
        }
        scope.internalSapBuilt = true;
    }

    if (scope.type == PhysicsScopeType::Rest && !scope.vsAsleepSapBuilt && stepMode != StepMode::Global) {
        // Build the SAP for the rest scope against the sleeping bodies if it hasn't been built yet.
        broadphaseManager.buildSAPTwoSets(
            scope.vsAsleepSap,
            scope.bodies,
            broadphaseManager.getAsleepList()
        );
        scope.vsAsleepSapBuilt = true;
    }
    PairBatch pairs;
    pairs.dynamicPairs.reserve(scope.bodies.size() * 2);
    buildPairsForScope(scope, pairs);
    updateTimer("Broadphase", start);

    // Narrowphase
    start = glfwGetTime() * 1000.0;
    ContactBatch contacts;
    narrowphaseManager.narrowPhase(pairs, contacts);
    updateTimer("Narrowphase", start);

    // Contact collection
    start = glfwGetTime() * 1000.0;
    contacts.sortByMinY();
    updateTimer("Contact collection", start);

    // Solver
    start = glfwGetTime() * 1000.0;
    pgsSolver.solve(
        contacts,
        caches,
        pgsIterations,
        h
    );
    updateTimer("Collision resolution", start);

    processWakeList();

    contactsGeneratedThisFrame += contacts.size();
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