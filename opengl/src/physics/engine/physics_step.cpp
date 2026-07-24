#include "pch.h"

#include "physics/engine/physics_engine_impl.h"

#include "engine/engine_state.h"
#include "game/world.h"

void PhysicsEngine::Impl::init(World* world, FrameTimers* frameTimers) {
    this->world = world;
    this->frameTimers = frameTimers;

    collisionManifold = std::make_unique<CollisionManifold>();
    collisionManifold->init(&caches);

    pending.reserve(50000);
}

void PhysicsEngine::Impl::prepareStepLoop() {
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

void PhysicsEngine::Impl::step(float dt, EngineState& engine) {
    if (engine.isPaused() && !engine.getAdvanceStep()) {
        return;
    }

    double physicsStart = glfwGetTime() * 1000.0;

    const std::vector<RigidBodyHandle>& awake = broadphaseManager.getAwakeList();

    if (debugPhase == PhysicsStepDebugPhase::PausedBeforePositionIntegration) {
        integratePositionsAndColliders(awake, dt);
        endPhysicsStep(dt);

        debugPhase = PhysicsStepDebugPhase::Ready;

        frameTimers->submit("Physics", frameTimers->get("Physics") + glfwGetTime() * 1000.0 - physicsStart + savedPhysicsSurpassedTime);
    }

    beginPhysicsStep(dt);

    double start = glfwGetTime() * 1000.0;

    integrateForcesAndVelocities(awake, dt);

    frameTimers->submit("Sync", frameTimers->get("Sync") + glfwGetTime() * 1000.0 - start);

    start = glfwGetTime() * 1000.0;

    broadphaseManager.updateBVHs();

    frameTimers->submit("BVH update", frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start);

    start = glfwGetTime() * 1000.0;

    PairBatch pairs;

    broadphaseManager.buildPairs(pairs);
    broadphaseManager.buildSpeculativePairs(dt, pairs, debugSweeps);

    frameTimers->submit("Broadphase", frameTimers->get("Broadphase") + glfwGetTime() * 1000.0 - start);

    start = glfwGetTime() * 1000.0;

    ContactBatch contacts;

    narrowphaseManager.narrowPhase(pairs, contacts, dt);

    contactsGeneratedThisFrame += static_cast<uint32_t>(contacts.contacts.size() + contacts.speculativeContacts.size());

    frameTimers->submit("Narrowphase", frameTimers->get("Narrowphase") + glfwGetTime() * 1000.0 - start);

    start = glfwGetTime() * 1000.0;

    contacts.sortByMinY();

    frameTimers->submit("Contact collection", frameTimers->get("Contact collection") + glfwGetTime() * 1000.0 - start);

    start = glfwGetTime() * 1000.0;

    pgsSolver.solve(contacts, caches, pgsIterations, dt);

    frameTimers->submit("Collision resolution", frameTimers->get("Collision resolution") + glfwGetTime() * 1000.0 - start);

    if (engine.isPaused()) {
        broadphaseManager.updateBVHs();

        debugPhase = PhysicsStepDebugPhase::PausedBeforePositionIntegration;
        savedPhysicsSurpassedTime = glfwGetTime() * 1000.0 - physicsStart;

        return;
    }

    integratePositionsAndColliders(awake, dt);
    endPhysicsStep(dt);

    frameTimers->submit("Physics", frameTimers->get("Physics") + glfwGetTime() * 1000.0 - physicsStart);
}

void PhysicsEngine::Impl::beginPhysicsStep(float outerDt) {
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

    frameTimers->submit("Pre step", frameTimers->get("Pre step") + glfwGetTime() * 1000.0 - start);
}

void PhysicsEngine::Impl::endPhysicsStep(float outerDt) {
    double start = glfwGetTime() * 1000.0;

    processWakeList();
    updateSleepThresholds();
    processSleepList(outerDt);
    addSleepDamping();
    updateContactCache();

    frameTimers->submit("Post step", frameTimers->get("Post step") + glfwGetTime() * 1000.0 - start);
}