#include "pch.h"
#include "physics.h"
#include "aabb.h"
#include "wake_sleep_utils.h" 

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
void PhysicsEngine::clearPhysicsData() {
    physicsWorld.getCollidersMap().clear();
    physicsWorld.getRigidBodiesMap().clear();

    contactCache.clear();
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
        for (auto it = contactCache.begin(); it != contactCache.end(); ++it) {
            for (ContactPoint& cp : it->second.points) {
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

            //body->applyVelocityDamping();
            body->applyGravity(dt);
            body->applyAntistuckFriction(dt);
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

    collectActiveContacts();                    // collect contacts to solve
    resolveCollisions();                        // PGS + Baumgarte stabilization
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

    // Sort contacts by minimum Y coordinate of contact points to improve solving stability
    auto minY = [](const Contact* c) {
        const auto& pts = c->points;
        if (pts.empty()) return std::numeric_limits<float>::infinity();
        const auto it = std::min_element(pts.begin(), pts.end(),
            [](const ContactPoint& p1, const ContactPoint& p2) {
                return p1.worldPos.y < p2.worldPos.y;
            });
        return it->worldPos.y;
        };

    std::sort(contactsToSolve.begin(), contactsToSolve.end(),
        [&](const Contact* a, const Contact* b) {
            float ay = minY(a), by = minY(b);
            if (ay < by) return true;
            if (by < ay) return false;
            // tie-breaker for determinism:
            return a->hashKey < b->hashKey;
        });

    debugData.contacts = contactsToSolve.size();
}

//===============================================
//            Resolve collisions
//===============================================
void PhysicsEngine::resolveCollisions() {
    ScopedTimer t(*frameTimers, "Collision resolution");

    // Justera beroende på material
    constexpr float staticFriction = 0.6f;
    constexpr float dynamicFriction = 0.4f;
    constexpr float twistFriction = 0.1f;

    float lastResidualSq = 0.0f;

    for (Contact* contact : contactsToSolve) {
        ContactRuntime& rt = contact->runtimeData;
        RigidBody* bodyA = rt.bodyA;
        RigidBody* bodyB = rt.bodyB;

        // ----- Baumgarte stabilization -----
        float slop = 0.0005f;
        float baumgarteFactor = 0.15f;
        if (contact->noSolverResponseA or contact->noSolverResponseB) {
            baumgarteFactor = 0.30f;
            slop = 0.00005f;
        }

        for (int j = 0; j < contact->points.size(); j++) {
            ContactPoint& cp = contact->points[j];

            float penetration = cp.depth;
            float allowed = penetration - slop;

            if (allowed > 0.0f) {
                cp.biasVelocity = -((baumgarteFactor * allowed) / dt);
            }
            else {
                cp.biasVelocity = 0.0f;
            }
        }

        // ----- Warm start -----
        constexpr float warmStartFactor = 0.9f;
        for (ContactPoint& cp : contact->points) {
            cp.accumulatedImpulse *= warmStartFactor;
            cp.accumulatedFrictionImpulse1 *= warmStartFactor;
            cp.accumulatedFrictionImpulse2 *= warmStartFactor;

            // total impuls från föregående steg 
            glm::vec3 Pn = cp.accumulatedImpulse * contact->normal;
            glm::vec3 Pt = cp.accumulatedFrictionImpulse1 * contact->t1 + cp.accumulatedFrictionImpulse2 * contact->t2;

            // linjär + tangentiell
            glm::vec3 J = Pn + Pt;

            if (contact->partnerTypeA == ContactPartnerType::RigidBody and !contact->noSolverResponseA) {
                bodyA->applyImpulseLinear(-J * bodyA->invMass);
                bodyA->applyImpulseAngular(-bodyA->invInertiaWorld * glm::cross(cp.rA, J));
            }
            if (contact->partnerTypeB == ContactPartnerType::RigidBody and !contact->noSolverResponseB) {
                bodyB->applyImpulseLinear(J * bodyB->invMass);
                bodyB->applyImpulseAngular(bodyB->invInertiaWorld * glm::cross(cp.rB, J));
            }
        }

        if (contact->accumulatedTwistImpulse != 0.0f) {
            glm::vec3 tauWS = (contact->accumulatedTwistImpulse * warmStartFactor) * contact->normal;
            if (!contact->noSolverResponseA) bodyA->applyImpulseAngular(-bodyA->invInertiaWorld * tauWS);
            if (!contact->noSolverResponseB) bodyB->applyImpulseAngular(bodyB->invInertiaWorld * tauWS);
        }
    }

    // ------ PGS solver ------
    int maxIterations = 4;
    int iterCount = 0;
    for (int i = 0; i < maxIterations; i++) {
        float maxDelta = 0.0f;
        float residualSq = 0.0f;

        for (Contact* contact : contactsToSolve) {
            ContactRuntime& rt = contact->runtimeData;
            RigidBody* bodyA = rt.bodyA;
            RigidBody* bodyB = rt.bodyB;

            for (int j = 0; j < contact->points.size(); j++) {
                ContactPoint& cp = contact->points[j];

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
                float v_bias = cp.biasVelocity;

                float J = -(normalVelocity - v_target + v_bias) * cp.m_eff;

                float oldNormalImpulse = cp.accumulatedImpulse;
                cp.accumulatedImpulse = glm::max(oldNormalImpulse + J, 0.0f);
                float deltaImpulse = cp.accumulatedImpulse - oldNormalImpulse;

                //residualSq += deltaImpulse * deltaImpulse;

                glm::vec3 deltaNormalImpulse = deltaImpulse * contact->normal;

                if (glm::dot(deltaNormalImpulse, deltaNormalImpulse) > 1e-6f) {
                    if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                        bodyA->applyImpulseLinear(-deltaNormalImpulse * bodyA->invMass);
                        bodyA->applyImpulseAngular(-bodyA->invInertiaWorld * glm::cross(cp.rA, deltaNormalImpulse));
                    }
                    if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                        bodyB->applyImpulseLinear(deltaNormalImpulse * bodyB->invMass);
                        bodyB->applyImpulseAngular(bodyB->invInertiaWorld * glm::cross(cp.rB, deltaNormalImpulse));
                    }
                }

                maxDelta = std::max(maxDelta, std::abs(deltaImpulse));

                // ----- Friction -----
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

                float Jn = std::abs(cp.accumulatedImpulse);
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

                    //residualSq += dT * dT;

                    if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                        bodyA->applyImpulseLinear(-dFt * bodyA->invMass);
                        bodyA->applyImpulseAngular(-bodyA->invInertiaWorld * glm::cross(cp.rA, dFt));
                    }
                    if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                        bodyB->applyImpulseLinear(dFt * bodyB->invMass);
                        bodyB->applyImpulseAngular(bodyB->invInertiaWorld * glm::cross(cp.rB, dFt));
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

                        //residualSq += dT * dT;

                        if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                            bodyA->applyImpulseLinear(-dFt * bodyA->invMass);
                            bodyA->applyImpulseAngular(-bodyA->invInertiaWorld * glm::cross(cp.rA, dFt));
                        }
                        if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                            bodyB->applyImpulseLinear(dFt * bodyB->invMass);
                            bodyB->applyImpulseAngular(bodyB->invInertiaWorld * glm::cross(cp.rB, dFt));
                        }
                    }
                }

                maxDelta = std::max(maxDelta, std::abs(dT));
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
                Jn_total += std::abs(cp.accumulatedImpulse);
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

                //residualSq += delta * delta;

                if (contact->partnerTypeA == ContactPartnerType::RigidBody && !contact->noSolverResponseA) {
                    bodyA->applyImpulseAngular(-bodyA->invInertiaWorld * tau);
                }
                if (contact->partnerTypeB == ContactPartnerType::RigidBody && !contact->noSolverResponseB) {
                    bodyB->applyImpulseAngular(bodyB->invInertiaWorld * tau);
                }
            }
        }
        iterCount++;

        lastResidualSq = residualSq;

        if (maxDelta < 1e-3f) {
            break;
        }
    }

    //std::cout << "PGS iterations: " << iterCount << "\n";
    //float residual = std::sqrt(lastResidualSq);
    //std::cout << "Residual impulse: " << residual << "\n";
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
            // Nollställ för nästa frame
            it->second.wasUsedThisFrame = false;
            it->second.framesSinceUsed = 0;  // Nollställ räknaren vid träff
        }
        ++it;
    }
}