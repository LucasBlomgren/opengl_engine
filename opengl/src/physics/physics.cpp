#include "pch.h"
#include "physics.h"
#include "aabb.h"
#include "wake_sleep_utils.h" 
#include <unordered_set>

void PhysicsEngine::init(World* world, FrameTimers* ft) {
    this->world = world;
    this->frameTimers = ft;
    collisionManifold = new CollisionManifold();
    collisionManifold->init(&caches);
}

//====================================
//         Setup scene
//====================================
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

//====================================
//         Clear scene
//====================================
void PhysicsEngine::clear() {
    toWake.clear();
    contactCache.clear();

    physicsWorld.clear();
    broadphaseManager.clear();
    narrowphaseManager.clear();
    contactsToSolve.clear();
    pending.clear();
    pending.reserve(10000);
}

//====================================
//          Getters
//====================================
PhysicsWorld* PhysicsEngine::getPhysicsWorld() {
    return &physicsWorld;
}
const BVHTree& PhysicsEngine::getDynamicAwakeBvh() const {
    return broadphaseManager.getAwakeBVH();
}
const BVHTree& PhysicsEngine::getDynamicAsleepBvh() const {
    return broadphaseManager.getAsleepBVH();
}
const BVHTree& PhysicsEngine::getStaticBvh() const {
    return broadphaseManager.getStaticBVH();
}
const TerrainBVH& PhysicsEngine::getTerrainBvh() const {
    return broadphaseManager.getTerrainBVH();
}
const std::unordered_map<size_t, Contact>& PhysicsEngine::GetContactCache() const {
    return contactCache;
}
std::vector<ExternalMotionContact>& PhysicsEngine::getExternalMotionContacts() {
    return narrowphaseManager.getExternalContacts();
}
const DebugData PhysicsEngine::getDebugData() {
    debugData.awake = broadphaseManager.getAwakeList().size();
    debugData.asleep = broadphaseManager.getAsleepList().size();
    debugData.Static = broadphaseManager.getStaticList().size();
    debugData.colliders = physicsWorld.getCollidersMap().dense().size();
    debugData.terrainTris = terrainTriangles->size();
    debugData.contacts = contactCache.size();
    return debugData;
}

void PhysicsEngine::setBVHDirty(RigidBodyHandle& handle) {
    broadphaseManager.setBVHDirty(handle);
}

//====================================
//           Raycast
//====================================
RaycastHit PhysicsEngine::performRaycast(Ray& r) {
    SlotMap<RigidBody, RigidBodyHandle>* bodyMap = &physicsWorld.getRigidBodiesMap();
    SlotMap<Collider, ColliderHandle>* colMap = &physicsWorld.getCollidersMap();
    SlotMap<GameObject, GameObjectHandle>* goMap = &world->getGameObjectsMap();
    RaycastHit a = raycast(r, broadphaseManager.getAwakeBVH(), bodyMap, colMap, goMap);
    RaycastHit b = raycast(r, broadphaseManager.getAsleepBVH(), bodyMap, colMap, goMap);
    RaycastHit c = raycast(r, broadphaseManager.getStaticBVH(), bodyMap, colMap, goMap);

    RaycastHit bestHit = a;
    if (b.hit) {
        if (bestHit.hit == false || b.t < bestHit.t) {
            bestHit = b;
        }
    }
    if (c.hit) {
        if (bestHit.hit == false || c.t < bestHit.t) {
            bestHit = c;
        }
    }
    return bestHit;
}

//====================================
//         Sleep All
//====================================
void PhysicsEngine::sleepAllObjects() {
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < (uint32_t)dense.size(); ++i) {
        RigidBody& body = dense[i];

        if (body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        RigidBodyHandle handle = bodyMap.handle_from_dense_index(i);

        // #rigidbody vector: loop over all the colliders
        broadphaseManager.moveToAsleep(handle);
    }
}

//====================================
//         Wake All
//====================================
void PhysicsEngine::awakenAllObjects() {
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < (uint32_t)dense.size(); ++i) {
        RigidBody& body = dense[i];

        if (!body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        RigidBodyHandle handle = bodyMap.handle_from_dense_index(i);

        // #rigidbody vector: loop over all the colliders
        broadphaseManager.moveToAwake(handle);
    }
}

//====================================
//     Add/Remove commands
//====================================
void PhysicsEngine::queueAdd(RigidBodyHandle& handle, BroadphaseBucket& target) {
    pending.push_back({ PhysCmd::Type::Add, handle, target });
}
void PhysicsEngine::queueRemove(RigidBodyHandle& handle) {
    std::cout << "[Physics QueueRemove]: Queue remove object with handle: slot " << handle.slot << ", gen " << handle.gen << "\n";
    pending.push_back({ PhysCmd::Type::Remove, handle, BroadphaseBucket::None });
}
void PhysicsEngine::queueMove(RigidBodyHandle& handle, BroadphaseBucket& target) {
    pending.push_back({ PhysCmd::Type::Move, handle, target });
}

//====================================--
//     Flush pending commands
//====================================--
void PhysicsEngine::flushBroadphaseCommands() {
    for (auto& cmd : pending) {
        switch (cmd.type) {
        case PhysCmd::Type::Add:
            broadphaseManager.add(cmd.handle, cmd.dst);
            break;

        case PhysCmd::Type::Remove:
            broadphaseManager.remove(cmd.handle);
            break;

        case PhysCmd::Type::Move:
            switch (cmd.dst) {
            case BroadphaseBucket::Awake:  broadphaseManager.moveToAwake(cmd.handle);  break;
            case BroadphaseBucket::Asleep: broadphaseManager.moveToAsleep(cmd.handle); break;
            case BroadphaseBucket::Static: broadphaseManager.moveToStatic(cmd.handle); break;
            default: break;
            }
            break;
        }
    }
    pending.clear();
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
//         Time step
//====================================
void PhysicsEngine::step(float deltaTime, std::mt19937& rng) {
    ScopedTimer t(*frameTimers, "Physics");

    currentFrame++;

    // Pre-step preparations
    {
        ScopedTimer t(*frameTimers, "Pre step");
        this->dt = deltaTime;

        // prepare for this step: clear caches, reserve memory for toWake/toSleep lists, etc.
        uint32_t bodiesSlotCap = physicsWorld.getRigidBodiesMap().slot_capacity();
        uint32_t collidersSlotCap = physicsWorld.getCollidersMap().slot_capacity();
        toWake.reserve(bodiesSlotCap);
        toSleep.reserve(bodiesSlotCap);
        toWake.clear();
        toSleep.clear();

        //externalContacts.clear();

        // add/remove objects to the BVH trees
        flushBroadphaseCommands();

        // Reset contact points for the current step
        for (auto& [key, contact] : contactCache) {
            contact.wasUsedThisFrame = false;

            for (ContactPoint& cp : contact.points) {
                cp.wasUsedThisFrame = false;
                cp.wasWarmStarted = false;
            }
        }

        // Update (position, orientation, AABB, collider, inertia, etc.)
        updateBodiesAndColliders();
    }

    // Update BVH trees
    {
        ScopedTimer t(*frameTimers, "BVH update");
        broadphaseManager.updateBVHs();
    }

    // Collision detection and resolution
    detectAndSolveCollisions();

    // Post-step updates
    {
        ScopedTimer t(*frameTimers, "Post step");
        // Update sleep thresholds based on collision history
        updateSleepThresholds();
        // Decide which objects to put to sleep or wake up
        decideSleep();
        // Clear contact points that were not used this frame
        updateContactCache();
    }
}

//====================================
//       Update methods
//====================================
void PhysicsEngine::updateBodiesAndColliders() {
    for (const RigidBodyHandle& bodyH : broadphaseManager.getAwakeList()) {
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

//==============================================
//      Collision detection & resolution
//==============================================
void PhysicsEngine::detectAndSolveCollisions()
{
    {
        ScopedTimer t(*frameTimers, "Broadphase");
        broadphaseManager.computePairs();
    }

    const auto& terrainPairs = broadphaseManager.getTerrainPairs();
    const auto& dynamicPairs = broadphaseManager.getDynamicPairs();

    {
        ScopedTimer t(*frameTimers, "Narrowphase");
        narrowphaseManager.narrowPhase(terrainPairs, dynamicPairs);
    }

    collectActiveContacts(); // collect contacts to solve
    resolveCollisions(); // PGS + Baumgarte stabilization
}

//==============================================
//         Collect active contacts
//==============================================
void PhysicsEngine::collectActiveContacts() {
    ScopedTimer t(*frameTimers, "Contact collection");

    contactsToSolve.clear();
    contactsToSolve.reserve(contactCache.size());

    for (auto& [key, c] : contactCache) {
        if (!c.wasUsedThisFrame) continue;

        const bool aActive = !c.noSolverResponseA;
        const bool bActive = (c.partnerTypeB == ContactPartnerType::RigidBody && !c.noSolverResponseB);
        if (aActive || bActive) {
            contactsToSolve.push_back(&c);
        }
    }

    std::sort(contactsToSolve.begin(), contactsToSolve.end(),
        [](const Contact* a, const Contact* b) {
            if (a->minY < b->minY) return true;
            if (b->minY < a->minY) return false;
            return a->hashKey < b->hashKey;
        });

    debugData.contacts = contactsToSolve.size();
}

//===============================================
//            Resolve collisions
//===============================================
void PhysicsEngine::resolveCollisions() {
    ScopedTimer t(*frameTimers, "Collision resolution");

    // #TODO: solver islands
    // för att undvika att lösa stora staplar av kontakter som inte påverkar varandra, vilket kan hända i t.ex. en pyramid av boxar där varje box har kontakt med flera
    // det påverkar också determinism 

    constexpr int iterations = 8;
    constexpr float velocityImpulseEps = 1e-2f;
    constexpr float biasImpulseEps = 5e-2f;

    constexpr float normalImpulseEps = 1e-6f;

    constexpr float staticFriction = 0.6f;
    constexpr float dynamicFriction = 0.4f;
    constexpr float twistFriction = 0.1f;

    constexpr float defaultSlop = 0.0005f;
    constexpr float noResponseSlop = 0.0005f;

    constexpr float defaultBaumgarte = 0.3f;
    constexpr float noResponseBaumgarte = 0.3f;

    constexpr float persistentSlop = 0.005f;
    constexpr float angularBiasScale = 0.2f;

    // optional, but use high value first
    constexpr float maxBiasVelocity = 2.0f;

    for (Contact* contact : contactsToSolve) {
        ContactRuntime& rt = contact->runtimeData;
        RigidBody* bodyA = rt.bodyA;
        RigidBody* bodyB = rt.bodyB;

        float contactSlop = defaultSlop;
        float contactBaumgarte = defaultBaumgarte;

        if (contact->noSolverResponseA || contact->noSolverResponseB) {
            contactSlop = noResponseSlop;
            contactBaumgarte = noResponseBaumgarte;
        }

        for (ContactPoint& cp : contact->points) {
            cp.accumulatedBiasImpulse = 0.0f;

            bool active = cp.depth > -persistentSlop;

            if (!active) {
                cp.accumulatedNormalImpulse = 0.0f;
                cp.accumulatedFrictionImpulse1 = 0.0f;
                cp.accumulatedFrictionImpulse2 = 0.0f;
                cp.accumulatedBiasImpulse = 0.0f;
                cp.biasVelocity = 0.0f;
                cp.active = false;
                continue;
            }

            cp.active = true;

            float allowed = cp.depth - contactSlop;

            if (allowed > 0.0f) {
                float correctionSpeed = (contactBaumgarte * allowed) / dt;
                correctionSpeed = glm::min(correctionSpeed, maxBiasVelocity);

                cp.biasVelocity = -correctionSpeed;
            }
            else {
                cp.biasVelocity = 0.0f;
            }

            float maxFriction = staticFriction * cp.accumulatedNormalImpulse;

            float f1 = cp.accumulatedFrictionImpulse1;
            float f2 = cp.accumulatedFrictionImpulse2;
            float len2 = f1 * f1 + f2 * f2;
            float max2 = maxFriction * maxFriction;

            if (len2 > max2) {
                float len = std::sqrt(len2);
                if (len > 1e-6f) {
                    float s = maxFriction / len;
                    cp.accumulatedFrictionImpulse1 *= s;
                    cp.accumulatedFrictionImpulse2 *= s;
                }
            }

            glm::vec3 Pn = cp.accumulatedNormalImpulse * contact->normal;
            glm::vec3 Pt =
                cp.accumulatedFrictionImpulse1 * contact->t1 +
                cp.accumulatedFrictionImpulse2 * contact->t2;

            glm::vec3 J = Pn + Pt;

            if (contact->partnerTypeA == ContactPartnerType::RigidBody &&
                !contact->noSolverResponseA) {
                bodyA->applyImpulseLinear(-J);
                bodyA->applyImpulseAngular(-glm::cross(cp.rA, J));
            }

            if (contact->partnerTypeB == ContactPartnerType::RigidBody &&
                !contact->noSolverResponseB) {
                bodyB->applyImpulseLinear(J);
                bodyB->applyImpulseAngular(glm::cross(cp.rB, J));
            }
        }
    }

    // ------ PGS solver ------
    int iterationsUsed = 0;
    for (int i = 0; i < iterations; i++) {
        float maxVelocityDelta = 0.0f;
        float maxBiasDelta = 0.0f;

        iterationsUsed++;

        // reverse order every other iteration to reduce directional bias in the solver
        int contactCount = static_cast<int>(contactsToSolve.size());

        for (int cc = 0; cc < contactCount; cc++) {
            int ci = (i % 2 == 0)
                ? cc
                : contactCount - 1 - cc;

            Contact* contact = contactsToSolve[ci];

            ContactRuntime& rt = contact->runtimeData;
            RigidBody* bodyA = rt.bodyA;
            RigidBody* bodyB = rt.bodyB;

            int count = static_cast<int>(contact->points.size());

            // reverse order every other iteration to reduce directional bias in the solver
            for (int jj = 0; jj < count; jj++) {
                int j = (i % 2 == 0) ? jj : (count - 1 - jj);

                ContactPoint& cp = contact->points[j];

                if (!cp.active) continue;

                // --- Relative velocity before normal solve ---
                glm::vec3 relVelA{ 0.0f };
                glm::vec3 relVelB{ 0.0f };
                glm::vec3 angVelA{ 0.0f };
                glm::vec3 angVelB{ 0.0f };

                if (contact->contributesMotionA) {
                    relVelA = bodyA->linearVelocity;
                    angVelA = bodyA->angularVelocity;
                }
                if (contact->contributesMotionB) {
                    relVelB = bodyB->linearVelocity;
                    angVelB = bodyB->angularVelocity;
                }

                glm::vec3 relativeVelocity =
                    (relVelB + glm::cross(angVelB, cp.rB)) -
                    (relVelA + glm::cross(angVelA, cp.rA));

                float normalVelocity = glm::dot(relativeVelocity, contact->normal);

                // ----- Normal impulse -----
                float v_target = cp.targetBounceVelocity;

                float J = -(normalVelocity - v_target) * cp.m_eff;

                float oldNormalImpulse = cp.accumulatedNormalImpulse;
                cp.accumulatedNormalImpulse = glm::max(oldNormalImpulse + J, 0.0f);

                float deltaImpulse = cp.accumulatedNormalImpulse - oldNormalImpulse;
                glm::vec3 deltaNormalImpulse = deltaImpulse * contact->normal;

                if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                    bodyA->applyImpulseLinear(-deltaNormalImpulse);
                    bodyA->applyImpulseAngular(-glm::cross(cp.rA, deltaNormalImpulse));
                }
                if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                    bodyB->applyImpulseLinear(deltaNormalImpulse);
                    bodyB->applyImpulseAngular(glm::cross(cp.rB, deltaNormalImpulse));
                }

                maxVelocityDelta = std::max(maxVelocityDelta, std::abs(deltaImpulse));

                // ----- Bias impulse (Baumgarte) -----
                if (cp.biasVelocity != 0.0f || cp.accumulatedBiasImpulse > 0.0f) {
                    glm::vec3 relVelA_bias{ 0.0f };
                    glm::vec3 relVelB_bias{ 0.0f };
                    glm::vec3 angVelA_bias{ 0.0f };
                    glm::vec3 angVelB_bias{ 0.0f };

                    if (contact->contributesMotionA) {
                        relVelA_bias = bodyA->biasLinearVelocity;
                        angVelA_bias = bodyA->biasAngularVelocity;
                    }
                    if (contact->contributesMotionB) {
                        relVelB_bias = bodyB->biasLinearVelocity;
                        angVelB_bias = bodyB->biasAngularVelocity;
                    }

                    glm::vec3 relativeBiasVelocity =
                        (relVelB_bias + glm::cross(angVelB_bias, cp.rB)) -
                        (relVelA_bias + glm::cross(angVelA_bias, cp.rA));

                    float normalBiasVelocity = glm::dot(relativeBiasVelocity, contact->normal);

                    float Jb = -(normalBiasVelocity + cp.biasVelocity) * cp.m_eff;

                    float oldB = cp.accumulatedBiasImpulse;
                    cp.accumulatedBiasImpulse = glm::max(oldB + Jb, 0.0f);

                    float deltaB = cp.accumulatedBiasImpulse - oldB;
                    glm::vec3 impulseB = deltaB * contact->normal;

                    if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                        bodyA->pushBiasImpulseLinear(-impulseB);
                        glm::vec3 angularBiasA = -glm::cross(cp.rA, impulseB);
                        bodyA->pushBiasImpulseAngular(angularBiasScale * angularBiasA);
                    }
                    if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                        bodyB->pushBiasImpulseLinear(impulseB);
                        glm::vec3 angularBiasB = glm::cross(cp.rB, impulseB);
                        bodyB->pushBiasImpulseAngular(angularBiasScale * angularBiasB);
                    }

                    maxBiasDelta = std::max(maxBiasDelta, std::abs(deltaB));
                }

                // ----- Friction -----
                if (cp.accumulatedNormalImpulse > normalImpulseEps) {
                    constexpr float recomputeThreshold = 1e-4f;

                    float v_t1, v_t2;

                    if (std::abs(deltaImpulse) > recomputeThreshold) {
                        // Recompute relative velocity after normal impulse
                        glm::vec3 relVelA2{ 0.0f };
                        glm::vec3 relVelB2{ 0.0f };
                        glm::vec3 angVelA2{ 0.0f };
                        glm::vec3 angVelB2{ 0.0f };

                        if (contact->contributesMotionA) {
                            relVelA2 = bodyA->linearVelocity;
                            angVelA2 = bodyA->angularVelocity;
                        }
                        if (contact->contributesMotionB) {
                            relVelB2 = bodyB->linearVelocity;
                            angVelB2 = bodyB->angularVelocity;
                        }

                        glm::vec3 relativeVelocity2 =
                            (relVelB2 + glm::cross(angVelB2, cp.rB)) -
                            (relVelA2 + glm::cross(angVelA2, cp.rA));

                        v_t1 = glm::dot(relativeVelocity2, contact->t1);
                        v_t2 = glm::dot(relativeVelocity2, contact->t2);
                    }
                    else {
                        // Reuse tangential velocity from old relative velocity
                        v_t1 = glm::dot(relativeVelocity, contact->t1);
                        v_t2 = glm::dot(relativeVelocity, contact->t2);
                    }

                    // Desired friction delta
                    float dF1 = -v_t1 * cp.invMassT1;
                    float dF2 = -v_t2 * cp.invMassT2;

                    // Candidate accumulated friction impulse
                    float newF1 = cp.accumulatedFrictionImpulse1 + dF1;
                    float newF2 = cp.accumulatedFrictionImpulse2 + dF2;

                    float Jn = std::abs(cp.accumulatedNormalImpulse);
                    float maxStatic = staticFriction * Jn;
                    float maxStatic2 = maxStatic * maxStatic;

                    float newLen2 = newF1 * newF1 + newF2 * newF2;
                    float dT = 0.0f;

                    if (newLen2 <= maxStatic2) {
                        // Static friction
                        cp.accumulatedFrictionImpulse1 = newF1;
                        cp.accumulatedFrictionImpulse2 = newF2;

                        glm::vec3 dFt = dF1 * contact->t1 + dF2 * contact->t2;
                        dT = std::sqrt(dF1 * dF1 + dF2 * dF2);

                        if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                            bodyA->applyImpulseLinear(-dFt);
                            bodyA->applyImpulseAngular(-glm::cross(cp.rA, dFt));
                        }
                        if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                            bodyB->applyImpulseLinear(dFt);
                            bodyB->applyImpulseAngular(glm::cross(cp.rB, dFt));
                        }
                    }
                    else {
                        // Dynamic friction
                        float maxDyn = dynamicFriction * Jn;
                        float len = std::sqrt(newLen2);

                        if (len > 1e-6f) {
                            float s = maxDyn / len;
                            float clampedF1 = newF1 * s;
                            float clampedF2 = newF2 * s;

                            float d1 = clampedF1 - cp.accumulatedFrictionImpulse1;
                            float d2 = clampedF2 - cp.accumulatedFrictionImpulse2;

                            cp.accumulatedFrictionImpulse1 = clampedF1;
                            cp.accumulatedFrictionImpulse2 = clampedF2;

                            glm::vec3 dFt = d1 * contact->t1 + d2 * contact->t2;
                            dT = std::sqrt(d1 * d1 + d2 * d2);

                            if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                                bodyA->applyImpulseLinear(-dFt);
                                bodyA->applyImpulseAngular(-glm::cross(cp.rA, dFt));
                            }
                            if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                                bodyB->applyImpulseLinear(dFt);
                                bodyB->applyImpulseAngular(glm::cross(cp.rB, dFt));
                            }
                        }
                    }

                    maxVelocityDelta = std::max(maxVelocityDelta, std::abs(dT));
                }
            }

            // ---------- Twist friction (per manifold) ----------
            // 1) Relativ rotationshastighet kring normalen
            glm::vec3 angVelA{ 0.0f };
            glm::vec3 angVelB{ 0.0f };
            if (contact->contributesMotionA) angVelA = bodyA->angularVelocity;
            if (contact->contributesMotionB) angVelB = bodyB->angularVelocity;

            float v_twist = glm::dot((angVelB - angVelA), contact->normal);

            // 3) Friktionsbudget baserad på TOTAL normalimpuls i manifoldet
            float Jn_total = 0.0f;
            for (const ContactPoint& cp : contact->points) {
                Jn_total += std::abs(cp.accumulatedNormalImpulse);
            }

            float maxTwistImpulse = twistFriction * Jn_total;

            // 4) PGS-uppdatering (delta) för en enda twist-λ per manifold
            float oldTwist = contact->accumulatedTwistImpulse;
            float dLambda = -v_twist * contact->invMassTwist;
            float newTwist = glm::clamp(oldTwist + dLambda, -maxTwistImpulse, maxTwistImpulse);
            float delta = newTwist - oldTwist;
            contact->accumulatedTwistImpulse = newTwist;

            // 5) Applicera moment kring n
            glm::vec3 tau = delta * contact->normal;
            if (glm::dot(tau, tau) > 1e-6f) {
                if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                    bodyA->applyImpulseAngular(-tau);
                }
                if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                    bodyB->applyImpulseAngular(tau);
                }
            }
        }

        if (maxVelocityDelta < velocityImpulseEps && 
            maxBiasDelta < biasImpulseEps) 
        {
            break;
        }
    }

    // commit bias impulses so they affect velocity in the next frame's collision detection and solving, which improves stability especially for stacked objects
    for (Contact* contact : contactsToSolve) {
        RigidBody* bodies[2] = {
            contact->runtimeData.bodyA,
            contact->runtimeData.bodyB
        };

        for (RigidBody* body : bodies) {
            if (!body) continue;
            if (body->type == BodyType::Static) continue;
            if (body->lastBiasCommitFrame == currentFrame) continue;

            body->lastBiasCommitFrame = currentFrame;

            Transform& t = *caches.transforms.get(body->rootTransformHandle, FUNC_NAME);
            body->commitBiasImpulses(t, dt);
            t.updateCache();
            body->updateInertiaWorld(t);
        }
    }

    std::cout << "PGS iterations used: " << iterationsUsed << " out of " << iterations << "\n";
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

        constexpr float linearFactor = 0.17f;
        constexpr float angularFactor = 0.10f;

        // set thresholds
        body->velocityThreshold = avg * linearFactor;
        body->angularVelocityThreshold = avg * angularFactor * body->invRadius;
    }
}

//======================================
//      Decide sleep/awake
//======================================
void PhysicsEngine::decideSleep() {
    const std::vector<RigidBodyHandle>& awakeHandles = broadphaseManager.getAwakeList();

    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches.bodies.get(handle, FUNC_NAME);

        if (body->type != BodyType::Dynamic) continue;
        if (body->motionControl == MotionControl::External) continue;
        if (body->asleep) continue;
        if (!body->allowSleep) continue;
        if (body->inSleepTransition) continue;

        Transform* transform = caches.transforms.get(body->rootTransformHandle, FUNC_NAME);

        bool goingToSleep = WakeSleep::updateSleepStateAndCheckIfShouldSleep(
            *body,
            *transform,
            dt
        );

        if (goingToSleep) {
            toSleep.push_back(handle);
        }
    }

    for (RigidBodyHandle rb : toSleep) {
        broadphaseManager.moveToAsleep(rb);
    }

    for (RigidBodyHandle rb : toWake) {
        broadphaseManager.moveToAwake(rb);

        RigidBody* body = caches.bodies.get(rb, FUNC_NAME);
        body->inSleepTransition = false;
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