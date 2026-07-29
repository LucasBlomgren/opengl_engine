#include "pch.h"

#include <algorithm>

#include "physics/engine/physics_engine_impl.h"

#include "engine/engine_state.h"

namespace physics::internal {

//==============================
// Initialization
//==============================
void EngineImpl::init(FrameTimers* frameTimers) {
    this->frameTimers = frameTimers;

    collisionManifold = std::make_unique<CollisionManifold>();
    collisionManifold->init(&caches);

    commandBuffer.reserve(1024, 4096, 4096);
}

//==============================
// Step-loop preparation
//==============================
void EngineImpl::prepareStepLoop() {
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

//==============================
// Solver configuration
//==============================
int EngineImpl::getPgsIterations() const {
    return pgsIterations;
}

void EngineImpl::setPgsIterations(
    int iterations) {
    pgsIterations = (std::max)(1, iterations);
}

//==============================
// Simulation
//==============================
void EngineImpl::step(float dt, EngineState& engine) {
    if (engine.isPaused() && !engine.getAdvanceStep()) {
        return;
    }

    double physicsStart = glfwGetTime() * 1000.0;
    const std::vector<BodyHandle>& awake = broadphaseManager.getAwakeList();

    // Resume a step paused after collision resolution
    if (debugPhase == physics::debug::StepPhase::PausedBeforePositionIntegration) {
        integratePositionsAndColliders(awake, dt);
        endPhysicsStep(dt);
        debugPhase = physics::debug::StepPhase::Ready;
        frameTimers->submit(
            "Physics", 
            frameTimers->get("Physics") + glfwGetTime() * 1000.0 - physicsStart + savedPhysicsSurpassedTime
        );
    }

    // Step initialization
    beginPhysicsStep(dt);

    // Force and velocity integration
    double start = glfwGetTime() * 1000.0;
    integrateForcesAndVelocities(awake, dt);
    frameTimers->submit(
        "Sync", 
        frameTimers->get("Sync") + glfwGetTime() * 1000.0 - start
    );

    //// BVH update
    //start = glfwGetTime() * 1000.0;
    //broadphaseManager.updateBVHs();
    //frameTimers->submit(
    //    "BVH update", 
    //    frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
    //);

    // Broadphase
    start = glfwGetTime() * 1000.0;
    pairBatch.clear();
    broadphaseManager.buildPairs(pairBatch);
    broadphaseManager.buildSpeculativePairs(dt, pairBatch, debugSweeps);
    frameTimers->submit(
        "Broadphase", 
        frameTimers->get("Broadphase") + glfwGetTime() * 1000.0 - start
    );

    // Narrowphase
    start = glfwGetTime() * 1000.0;
    contactBatch.clear();

    contactBatch.contacts.reserve(
        pairBatch.dynamicPairs.size() + 
        pairBatch.terrainPairs.size()
    );
    contactBatch.speculativeContacts.reserve(
        pairBatch.dynamicPairs.size() + 
        pairBatch.terrainPairs.size()
    );
    narrowphaseManager.narrowPhase(pairBatch, contactBatch, dt);

    contactsGeneratedThisFrame +=
        static_cast<uint32_t>(
            contactBatch.contacts.size() + 
            contactBatch.speculativeContacts.size()
            );

    frameTimers->submit(
        "Narrowphase", 
        frameTimers->get("Narrowphase") + glfwGetTime() * 1000.0 - start
    );

    // Contact collection
    start = glfwGetTime() * 1000.0;
    contactBatch.sortByMinY();
    frameTimers->submit(
        "Contact collection", 
        frameTimers->get("Contact collection") + glfwGetTime() * 1000.0 - start
    );

    // Collision resolution
    start = glfwGetTime() * 1000.0;
    pgsSolver.solve(contactBatch, caches, pgsIterations, dt);
    frameTimers->submit(
        "Collision resolution", 
        frameTimers->get("Collision resolution") + glfwGetTime() * 1000.0 - start
    );

    // Debug pause before position integration
    if (engine.isPaused()) {
        broadphaseManager.updateBVHs();

        debugPhase = physics::debug::StepPhase::PausedBeforePositionIntegration;
        savedPhysicsSurpassedTime = glfwGetTime() * 1000.0 - physicsStart;

        return;
    }

    // Position integration
    integratePositionsAndColliders(awake, dt);

    // BVH update
    start = glfwGetTime() * 1000.0;
    broadphaseManager.updateBVHs();
    frameTimers->submit(
        "BVH update",
        frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
    );

    // Step finalization
    endPhysicsStep(dt);

    frameTimers->submit(
        "Physics", 
        frameTimers->get("Physics") + glfwGetTime() * 1000.0 - physicsStart
    );
}

//==============================
// Step initialization
//==============================
void EngineImpl::beginPhysicsStep(float outerDt) {
    double start = glfwGetTime() * 1000.0;

    processPendingCommands();

    uint32_t bodySlotCapacity =
        physicsWorld.getRigidBodiesMap().slot_capacity();

    toWake.reserve(bodySlotCapacity);
    toSleep.reserve(bodySlotCapacity);

    toWake.clear();
    toSleep.clear();

    debugSweeps.clear();
    debugSpeculativeContacts.clear();

    contactsGeneratedThisFrame = 0;

    for (auto& [key, contact] : contactCache) {
        contact.wasUsedThisFrame = false;

        for (size_t i = 0; i < contact.numPoints; ++i) {
            ContactPoint& contactPoint = contact.points[i];
            contactPoint.wasUsedThisFrame = false;
            contactPoint.wasWarmStarted = false;
        }
    }

    frameTimers->submit(
        "Pre step",
        frameTimers->get("Pre step") + glfwGetTime() * 1000.0 - start
    );
}

//==============================
// Step finalization
//==============================
void EngineImpl::endPhysicsStep(float outerDt) {
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

}
