#include "pch.h"
#include "physics_engine.h"

#include "engine/engine_state.h"
#include "world.h"
#include "aabb.h"
#include "wake_sleep_utils.h"
#include "rigidbody.h"
#include "broadphase/broadphase_types.h"

//===================================
//    Initialization 
//===================================
void PhysicsEngine::init(World* world, FrameTimers* ft) {
    this->world = world;
    this->frameTimers = ft;
    collisionManifold = new CollisionManifold();
    collisionManifold->init(&caches);
}

//===================================
// Prepare for stepping loop 
//===================================
void PhysicsEngine::prepareStepLoop() {
    // only clear pointer caches before accumulator loop, since objects are not supposed to be added/removed during accumulator step loop,
    // and if cache is cleared multiple times during the loop, it will cause redundant lookups in the slot maps and thus worse performance.
    caches.clear();
}

//====================================
//         Frame step
//====================================
void PhysicsEngine::step(float dt, EngineState& engine) {
    double physicsStart = glfwGetTime() * 1000.0;

    beginPhysicsStep(dt);

    int substeps = computeAdaptiveSubsteps(dt);
    substeps = std::max(substeps, 1);

    float h = dt / static_cast<float>(substeps);
    currentSubstepAmount = substeps;

    StepScope globalScope;
    globalScope.type = StepScopeType::Global;
    globalScope.bodies = &broadphaseManager.getAwakeList();

    int executedSubsteps = 0;

    for (int i = 0; i < substeps; ++i) {
        currentStepId++;

        stepScope(globalScope, h);

        executedSubsteps++;

        if (engine.getAdvanceStep()) {
            break;
        }
    }

    float simulatedDt = h * static_cast<float>(executedSubsteps);

    endPhysicsStep(simulatedDt);

    frameTimers->submit(
        "Physics",
        frameTimers->get("Physics") + glfwGetTime() * 1000.0 - physicsStart
    );
}

//=====================================================================
//   Begin physics step: 
//   reset timers, prepare lists and contact cache for the new frame
//=====================================================================
void PhysicsEngine::beginPhysicsStep(float outerDt) {
    // Timers
    frameTimers->reset("Physics");
    frameTimers->reset("Pre step");
    frameTimers->reset("BVH update");
    frameTimers->reset("Broadphase");
    frameTimers->reset("Narrowphase");
    frameTimers->reset("Contact collection");
    frameTimers->reset("Collision resolution");
    frameTimers->reset("Post step");

    currentFrame++;
    currentStepId = 0;

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
}

//===================================
//    Compute adaptive substeps
//===================================
int PhysicsEngine::computeAdaptiveSubsteps(float dt) {
    //ScopedTimer t(*frameTimers, "Adaptive substep computation");

    constexpr float safeFraction = 0.50f;
    constexpr float minSafeDistance = 0.02f;

    int halfMaxSubsteps = maxSubsteps / 2;
    int globalSubsteps = 1;

    const std::vector<RigidBodyHandle>& awakeHandles = broadphaseManager.getAwakeList();

#if DEBUG_PRINT_WANTED_SUBSTEP_AMOUNT
    std::vector<int> substepsForBodies;
    std::unordered_map<RigidBodyHandle, int> bodyToSubsteps;
#endif

    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches.bodies.get(handle, FUNC_NAME);
        Collider* mainCollider = caches.colliders.get(body->colliderHandles[0], FUNC_NAME);
        if (!body) continue;
        if (body->type != BodyType::Dynamic) continue;
        if (body->asleep) continue;

        Transform* rootTransform = caches.transforms.get(body->rootTransformHandle, FUNC_NAME);

        if (!rootTransform) continue;

        // -----------------------------
        // 1. Estimate object size
        // -----------------------------
        glm::vec3& scale = rootTransform->scale;
        float minExtent = std::min(scale.x, std::min(scale.y, scale.z));
        float boundingRadius = 0.5f * glm::length(scale);
        float safeDistance = std::max(minExtent * safeFraction, minSafeDistance);

        // -----------------------------
        // 2. Estimate motion this frame
        // -----------------------------
        float linearMotion = glm::length(body->linearVelocity) * dt;
        float angularMotion = glm::length(body->angularVelocity) * boundingRadius * dt;
        float totalMotion = linearMotion + angularMotion;

        // Not fast enough to matter.
        if (totalMotion <= safeDistance) {

#if DEBUG_PRINT_WANTED_SUBSTEP_AMOUNT
            bodyToSubsteps[handle] = 1;
#endif

            continue;
        }

        int wantedSubsteps = static_cast<int>(std::ceil(totalMotion / safeDistance));
        wantedSubsteps = std::clamp(wantedSubsteps, 1, maxSubsteps);

        // If this body cannot increase the current global value, skip expensive query.
        if (wantedSubsteps <= globalSubsteps) {
            continue;
        }

        // -----------------------------
        // 3. Build swept AABB
        // -----------------------------
        AABB& currentAABB = body->aabb;

        glm::vec3 delta = body->linearVelocity * dt;

        AABB endAABB = currentAABB;
        endAABB.worldMin += delta;
        endAABB.worldMax += delta;

        AABB sweptAABB;
        sweptAABB.worldMin = glm::min(currentAABB.worldMin, endAABB.worldMin);
        sweptAABB.worldMax = glm::max(currentAABB.worldMax, endAABB.worldMax);

        // Optional expansion for rotation.
        if (mainCollider->type != ColliderType::SPHERE || body->isCompound()) {
            float angularExpansion = glm::length(body->angularVelocity) * boundingRadius * dt;
            sweptAABB.worldMin -= glm::vec3(angularExpansion);
            sweptAABB.worldMax += glm::vec3(angularExpansion);
        }

        // Optional small skin.
        constexpr float sweptSkin = 0.01f;
        sweptAABB.worldMin -= glm::vec3(sweptSkin);
        sweptAABB.worldMax += glm::vec3(sweptSkin);

        // -----------------------------
        // 4. Check if swept AABB actually hits anything
        // -----------------------------

#if DEBUG_PRINT_WANTED_SUBSTEP_AMOUNT
        std::vector<RigidBodyHandle> awakeCandidates;
        std::vector<RigidBodyHandle> asleepCandidates;
        broadphaseManager.getAwakeBVH().singleQuery(sweptAABB, awakeCandidates);
        broadphaseManager.getAsleepBVH().singleQuery(sweptAABB, asleepCandidates);
#endif

        bool hitAwake =
            broadphaseManager.getAwakeBVH().queryAny(sweptAABB, handle);

        bool hitAsleep = false;
        if (!hitAwake) {
            hitAsleep = broadphaseManager.getAsleepBVH().queryAny(sweptAABB, handle);
        }

        bool hitTerrain = false;
        if (!hitAwake && !hitAsleep) {
            hitTerrain = broadphaseManager.getTerrainBVH().queryAny(sweptAABB);
        }

        bool hitStatic = false;
        if (!hitAwake && !hitAsleep && !hitTerrain) {
            hitStatic = broadphaseManager.getStaticBVH().queryAny(sweptAABB, handle);
        }

        if (!hitAwake && !hitAsleep && !hitTerrain && !hitStatic) {
            continue;
        }

        // Static-only hits are cheaper: no dynamic target needs to wake up or propagate impulses.
        if (hitStatic) {
            wantedSubsteps = std::min(wantedSubsteps, halfMaxSubsteps);
        }

        // This fast object actually risks collision this frame.
        globalSubsteps = std::max(globalSubsteps, wantedSubsteps);

#if !DEBUG_PRINT_WANTED_SUBSTEP_AMOUNT
        if (globalSubsteps == maxSubsteps)
            break;
#endif

#if DEBUG_PRINT_WANTED_SUBSTEP_AMOUNT
        bodyToSubsteps[handle] = wantedSubsteps;
        for (RigidBodyHandle candidateHandle : awakeCandidates)
            bodyToSubsteps[candidateHandle] = std::max(bodyToSubsteps[candidateHandle], wantedSubsteps);
        for (RigidBodyHandle candidateHandle : asleepCandidates)
            bodyToSubsteps[candidateHandle] = std::max(bodyToSubsteps[candidateHandle], wantedSubsteps);
#endif
    }


#if DEBUG_PRINT_WANTED_SUBSTEP_AMOUNT
    // DEBUG: print substep distribution
    std::array substepCounts = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    for (auto it = bodyToSubsteps.begin(); it != bodyToSubsteps.end(); it++) {
        substepCounts[it->second - 1]++;
    }
    std::cout << "Adaptive substeps: " << globalSubsteps << "\n";
    for (int i = 0; i < maxSubsteps; ++i) {
        std::cout << i + 1 << " steps: " << substepCounts[i] << " \n";
    }
    std::cout << "==============================\n";
#endif

    return globalSubsteps;
}


//===================================
//    Step scope
//===================================
void PhysicsEngine::stepScope(const StepScope& scope, float h) {
    this->dt = h;

    double start = glfwGetTime() * 1000.0;
    updateBodiesAndColliders(*scope.bodies, h);

    frameTimers->submit(
        "Pre step",
        frameTimers->get("Pre step") + glfwGetTime() * 1000.0 - start
    );


    start = glfwGetTime() * 1000.0;
    broadphaseManager.updateBVHs();

    frameTimers->submit(
        "BVH update",
        frameTimers->get("BVH update") + glfwGetTime() * 1000.0 - start
    );


    start = glfwGetTime() * 1000.0;
    PairBatch pairs;
    broadphaseManager.buildPairs(scope, pairs);

    frameTimers->submit(
        "Broadphase",
        frameTimers->get("Broadphase") + glfwGetTime() * 1000.0 - start
    );

    ContactBatch contacts;
    start = glfwGetTime() * 1000.0;
    narrowphaseManager.narrowPhase(pairs, contacts);

    frameTimers->submit(
        "Narrowphase",
        frameTimers->get("Narrowphase") + glfwGetTime() * 1000.0 - start
    );

    start = glfwGetTime() * 1000.0;
    contacts.sortByMinY();

    frameTimers->submit(
        "Contact collection",
        frameTimers->get("Contact collection") + glfwGetTime() * 1000.0 - start
    );


    start = glfwGetTime() * 1000.0;
    pgsSolver.resolveContacts(scope, contacts, pgsIterations, h);
    pgsSolver.postSolve(scope, contacts, caches, currentFrame, h);

    frameTimers->submit(
        "Collision resolution",
        frameTimers->get("Collision resolution") + glfwGetTime() * 1000.0 - start
    );


    processWakeList();
}

//================================================================
//    End physics step: update sleep states, contact cache etc.
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

//====================================
//   Process wake list
//====================================
void PhysicsEngine::processWakeList() {
    for (RigidBodyHandle rb : toWake) {
        broadphaseManager.moveToAwake(rb);

        RigidBody* body = caches.bodies.get(rb, FUNC_NAME);
        if (body) {
            body->inSleepTransition = false;
        }
    }

    toWake.clear();
}

//====================================
//    Process sleep list
//====================================
void PhysicsEngine::processSleepList(float outerDt) {
    toSleep.clear();

    const std::vector<RigidBodyHandle>& awakeHandles = broadphaseManager.getAwakeList();

    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches.bodies.get(handle, FUNC_NAME);
        if (!body) continue;

        if (body->type != BodyType::Dynamic) continue;
        if (body->motionControl == MotionControl::External) continue;
        if (body->asleep) continue;
        if (!body->allowSleep) continue;
        if (body->inSleepTransition) continue;

        Transform* transform = caches.transforms.get(body->rootTransformHandle, FUNC_NAME);
        if (!transform) continue;

        bool goingToSleep =
            WakeSleep::updateSleepStateAndCheckIfShouldSleep(*body, *transform, outerDt);

        if (goingToSleep) {
            toSleep.push_back(handle);
        }
    }

    for (RigidBodyHandle rb : toSleep) {
        broadphaseManager.moveToAsleep(rb);
    }

    toSleep.clear();
}

//====================================
//       Update methods
//====================================
void PhysicsEngine::updateBodiesAndColliders(const std::vector<RigidBodyHandle>& bodies, float dt) {
    for (const RigidBodyHandle& bodyH : bodies) {
        RigidBody* body = caches.bodies.get(bodyH, FUNC_NAME);
        Transform* rootTransform = caches.transforms.get(body->rootTransformHandle, FUNC_NAME);
        Collider* mainCollider = caches.colliders.get(body->colliderHandles[0], FUNC_NAME);

        // once per body
        if (!body->colliderHandles.empty()) {
            // for solo spheres
            if (body->colliderHandles.size() == 1) {
                body->applyRollingFriction(mainCollider->type, dt);
            }

            body->applyVelocityDamping(dt);
            body->applyGravity(dt);
            //body->applyAntistuckFriction(dt);
            body->integrateVelocities(*rootTransform, dt);
            rootTransform->updateCache();
            body->updateInertiaWorld(*rootTransform);
        }

        // per collider
        for (const ColliderHandle& colH : body->colliderHandles) {
            Collider* collider = caches.colliders.get(colH, FUNC_NAME);
            Transform* localTransform = caches.transforms.get(collider->localTransformHandle, FUNC_NAME);

            collider->pose.combineIntoColliderPose(*rootTransform, *localTransform);
            collider->pose.ensureModelMatrix();
            collider->updateAABB(collider->pose);
            collider->updateCollider(collider->pose);
        }

        body->aabb = mainCollider->getAABB();

        // update compound body AABB
        if (body->isCompound()) {
            for (size_t i = 1; i < body->colliderHandles.size(); ++i) {
                Collider* c = caches.colliders.get(body->colliderHandles[i], FUNC_NAME);
                body->aabb.growToInclude(c->getAABB().worldMin);
                body->aabb.growToInclude(c->getAABB().worldMax);
            }

            body->aabb.worldCenter = (body->aabb.worldMin + body->aabb.worldMax) * 0.5f;
            body->aabb.worldHalfExtents = (body->aabb.worldMax - body->aabb.worldMin) * 0.5f;
            body->aabb.setSurfaceArea();
        }
    }
}

//====================================
//       Sleep Thresholds
//====================================
void PhysicsEngine::updateSleepThresholds() {
    const std::vector<RigidBodyHandle>& awakeHandles = broadphaseManager.getAwakeList();
    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches.bodies.get(handle, FUNC_NAME);

        if (body->asleep || !body->allowSleep ||
            body->motionControl == MotionControl::External || body->type != BodyType::Dynamic)
            continue;

        body->collisionHistory.push(body->totalCollisionCount);
        body->totalCollisionCount = 0;
        float avg = body->collisionHistory.average();

        if (avg <= 0.0f) {
            if (std::abs(avg - body->lastAvg) >= 1) {
                body->sleepCounter = 0.0f;
            }
            body->lastAvg = avg;

            continue;
        }

        avg = std::max(avg, 1.0f);
        body->lastAvg = avg;

        constexpr float linearFactor = 0.2f;
        constexpr float angularFactor = 0.1f;

        // set thresholds
        body->velocityThreshold = avg * linearFactor;
        body->angularVelocityThreshold = avg * angularFactor * body->invRadius;
    }
}


//====================================
//      Sleep damping
//====================================
void PhysicsEngine::addSleepDamping() {
    const std::vector<RigidBodyHandle>& awakeHandles = broadphaseManager.getAwakeList();
    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches.bodies.get(handle, FUNC_NAME);
        if (body->type != BodyType::Dynamic) continue;
        if (body->motionControl == MotionControl::External) continue;
        if (!body->allowSleep) continue;
        if (body->totalCollisionCount == 0) continue;
        if (body->sleepCounter <= 0.0f) continue;

        float sleepT = glm::clamp(body->sleepCounter / body->sleepCounterThreshold, 0.0f, 1.0f);

        // Smoothstep
        sleepT = sleepT * sleepT * (3.0f - 2.0f * sleepT);

        constexpr float linearDampingStrength = 5.0f;
        constexpr float angularDampingStrength = 4.5f;

        float linearFactor = std::exp(-linearDampingStrength * sleepT * dt);
        float angularFactor = std::exp(-angularDampingStrength * sleepT * dt);

        body->linearVelocity *= linearFactor;
        body->angularVelocity *= angularFactor;
    }
}

//====================================
//       Contact Cache
//====================================
void PhysicsEngine::updateContactCache() {
    constexpr int maxFramesWithoutCollision = 5;
    for (auto it = contactCache.begin(); it != contactCache.end(); ) {
        if (!it->second.wasUsedThisFrame) {
            it->second.framesSinceUsed++;

            // Ta bort manifold efter X antal frames utan kollisionsmatch
            if (it->second.framesSinceUsed > maxFramesWithoutCollision) {
                it = contactCache.erase(it);
                continue;
            }
        }
        else {
            it->second.framesSinceUsed = 0; 
        }
        ++it;
    }
}