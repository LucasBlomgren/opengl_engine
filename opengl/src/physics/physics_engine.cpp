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

    // #BUG: Fixa NAN-bugg med sphere vs sphere för island substepping
    bool globalStep = false;

    beginPhysicsStep(dt);

    if (globalStep) {
        int substeps = 1;
        if (maxSubsteps > 1) {
            substeps = computeGlobalSubsteps(dt);
            substeps = std::max(substeps, 1);
        }
        float h = dt / static_cast<float>(substeps);
        currentSubstepAmount = substeps;

        StepScope globalScope;
        globalScope.type = StepScopeType::Global;
        globalScope.bodies = &broadphaseManager.getAwakeList();

        for (int i = 0; i < substeps; ++i) {
            stepScope(globalScope, h);
        }
    }
    else {
        createPredictedIslandsMVP(dt);

        for (const Island& island : predictedIslands) {
            StepScope islandScope;
            islandScope.type = StepScopeType::Island;
            islandScope.bodies = &island.bodies;
            islandScope.islandId = island.id;
            islandScope.islandBroadphaseMode = IslandBroadphaseMode::SAP;
            float islandH = dt / static_cast<float>(island.substeps);

            for (int i = 0; i < island.substeps; ++i) {
                stepScope(islandScope, islandH);
            }
        }

        StepScope restScope;
        restScope.type = StepScopeType::Rest;
        restScope.bodies = &restBodies;
        stepScope(restScope, dt);
    }

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

    broadphaseManager.updateBVHs();
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

    // pre-step: update bodies and colliders
    start = glfwGetTime() * 1000.0;
    updateBodiesAndColliders(*scope.bodies, h);
    updateTimer("Pre step", start);

    start = glfwGetTime() * 1000.0;
    broadphaseManager.updateBVHs();
    updateTimer("BVH update", start);

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