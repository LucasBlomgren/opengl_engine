#include "pch.h"
#include "physics.h"
#include "aabb.h"
#include "sat.h"

#define FUNC_NAME __FUNCTION__

PhysicsWorld* PhysicsEngine::getPhysicsWorld() {
    return &physicsWorld;
}

void PhysicsEngine::init(World* world, FrameTimers* ft) {
    this->world = world;
    this->frameTimers = ft;
    collisionManifold = new CollisionManifold();
}

//-----------------------------
//         Setup scene
//-----------------------------
void PhysicsEngine::setupScene(std::vector<Tri>* terrainTris) {
    this->terrainTriangles = terrainTris;

    broadphaseManager.init(&physicsWorld.getColliders(), &physicsWorld.getRigidBodies(), terrainTris);
    
    gameObjectPtrCache.init(world->getGameObjects(), "GameObject");
    colliderPtrCache.init(physicsWorld.getColliders(), "Collider");
    bodyPtrCache.init(physicsWorld.getRigidBodies(), "RigidBody");

    flushBroadphaseCommands();

    uint32_t slotCap = world->getGameObjects().slot_capacity();
    toWake.reserve(slotCap);
    toSleep.reserve(slotCap);
}

const DebugData PhysicsEngine::getDebugData() {
    debugData.awake = broadphaseManager.getAwakeList().size();
    debugData.asleep = broadphaseManager.getAsleepList().size();
    debugData.Static = broadphaseManager.getStaticList().size();
    debugData.terrainTris = terrainTriangles->size();
    debugData.collisions = contactCache.size();
    return debugData;
}

//-----------------------------
//         Clear scene
//-----------------------------
void PhysicsEngine::clearPhysicsData() {
    contactCache.clear();
    contactsToSolve.clear();
    pending.clear();
    pending.reserve(10000);
}

//-----------------------------
//          Getters
//-----------------------------
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

//-----------------------------
//           Raycast
//-----------------------------
RaycastHit PhysicsEngine::performRaycast(Ray& r) {
    SlotMap<GameObject, GameObjectHandle>* slotmap = &world->getGameObjects();
    RaycastHit a = raycast(r, broadphaseManager.getAwakeBVH(), slotmap);
    RaycastHit b = raycast(r, broadphaseManager.getAsleepBVH(), slotmap);
    RaycastHit c = raycast(r, broadphaseManager.getStaticBVH(), slotmap);

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

//-----------------------------
//         Sleep All
//-----------------------------
void PhysicsEngine::sleepAllObjects() {
    auto& bodyMap = physicsWorld.getRigidBodies();
    auto& colliderMap = physicsWorld.getColliders();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < (uint32_t)dense.size(); ++i) {
        RigidBody& body = dense[i];

        if (body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.player) continue;
        //if (body.broadphaseHandle.bucket == BroadphaseBucket::None) continue;

        ColliderHandle h = colliderMap.handle_from_dense_index(i);
        broadphaseManager.moveToAsleep(h);
    }
}

//-----------------------------
//         Wake All
//-----------------------------
void PhysicsEngine::awakenAllObjects() {
    auto& bodyMap = physicsWorld.getRigidBodies();
    auto& colliderMap = physicsWorld.getColliders();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < (uint32_t)dense.size(); ++i) {
        RigidBody& body = dense[i];

        if (!body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        //if (body.broadphaseHandle.bucket == BroadphaseBucket::None) continue;

        ColliderHandle h = colliderMap.handle_from_dense_index(i);
        broadphaseManager.moveToAwake(h);
    }
}

//-----------------------------
//     Add/Remove commands
//-----------------------------
void PhysicsEngine::queueAdd(ColliderHandle& handle, BroadphaseBucket& target) {
    pending.push_back({ PhysCmd::Type::Add, handle, target });
}
void PhysicsEngine::queueRemove(ColliderHandle& handle) {

    std::cout << "Physics QueueRemove: Queue remove object with handle: slot " << handle.slot << ", gen " << handle.gen << "\n";
    pending.push_back({ PhysCmd::Type::Remove, handle, BroadphaseBucket::None });
}
void PhysicsEngine::queueMove(ColliderHandle& handle, BroadphaseBucket& target) {
    pending.push_back({ PhysCmd::Type::Move, handle, target });
}

//-------------------------------
//     Flush pending commands
//-------------------------------
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

//-----------------------------
//         Time step
//-----------------------------
void PhysicsEngine::step(float deltaTime, std::mt19937 rng) {
    ScopedTimer t(*frameTimers, "Physics");

    colliderPtrCache.clear();
    bodyPtrCache.clear();

    // Pre-step preparations
    {
        ScopedTimer t(*frameTimers, "Pre step");
        this->dt = deltaTime;

        // prepare for this frame
        uint32_t slotCap = physicsWorld.getRigidBodies().slot_capacity();
        toWake.reserve(slotCap);
        toSleep.reserve(slotCap);
        toWake.clear();
        toSleep.clear();

        // add/remove objects to the BVH trees
        flushBroadphaseCommands();

        // Reset contact points for the current frame
        for (auto it = contactCache.begin(); it != contactCache.end(); ++it) {
            for (ContactPoint& cp : it->second.points) {
                cp.wasUsedThisFrame = false;
                cp.wasWarmStarted = false;
            }
        }

        // Update object states (position, orientation, AABB, collider)
        updateStates();
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

//-----------------------------
//       Update methods
//-----------------------------
void PhysicsEngine::updateStates() {
    for (const ColliderHandle& handle : broadphaseManager.getAwakeList()) {
        Collider* collider = colliderPtrCache.get(handle, __FUNCTION__);
        RigidBody* body = bodyPtrCache.get(collider->rigidBodyHandle, __FUNCTION__);
        GameObject* obj = gameObjectPtrCache.get(body->gameObjectHandle, __FUNCTION__);

        obj->transform.updateCache();
        body->update(this->dt);
        collider->updateAABB(obj->transform);
        collider->updateCollider(obj->transform);
    }

    for (const ColliderHandle& handle : broadphaseManager.getStaticList()) {
        Collider* collider = colliderPtrCache.get(handle, __FUNCTION__);
        RigidBody* body = bodyPtrCache.get(collider->rigidBodyHandle, __FUNCTION__);
        GameObject* obj = gameObjectPtrCache.get(body->gameObjectHandle, __FUNCTION__);

        obj->transform.updateCache();
        body->update(this->dt);
        collider->updateAABB(obj->transform);
        collider->updateCollider(obj->transform);
    }
}

//---------------------------------------------
//      Collision detection & resolution
//---------------------------------------------
void PhysicsEngine::detectAndSolveCollisions() 
{
    {
        ScopedTimer t(*frameTimers, "Broadphase");
        broadphaseManager.computePairs();
    }
    const auto& terrainPairs = broadphaseManager.getTerrainPairs();
    const auto& dynamicPairs = broadphaseManager.getDynamicPairs();

    narrowPhase(terrainPairs, dynamicPairs);    // SAT + collisionManifold
    collectActiveContacts();                    // collect contacts to solve
    resolveCollisions();                        // PGS + Baumgarte stabilization
}

//---------------------------------------------
//               Narrow phase
//---------------------------------------------
void PhysicsEngine::narrowPhase(const std::vector<TerrainPair>& terrainHits, const std::vector<DynamicPair>& dynamicHits) {
    ScopedTimer t(*frameTimers, "Narrowphase");

    // ----- Terrain vs dynamic ----- 
    for (const TerrainPair& th : terrainHits) {
        Collider* collider = colliderPtrCache.get(th.colliderHandle, __FUNCTION__);
        RigidBody* body = bodyPtrCache.get(collider->rigidBodyHandle, __FUNCTION__);
        if (body->asleep) {
            continue;
        }
        GameObject* obj = gameObjectPtrCache.get(body->gameObjectHandle, __FUNCTION__);

        static std::vector<SAT::Result> allResults;
        int reserveSize = th.tris.size() * 2; // worst case
        allResults.clear(); 
        allResults.reserve(reserveSize); // reserve space for results

        // CUBE vs tris
        if (collider->type == ColliderType::CUBOID) {
            for (Tri* tri : th.tris) {
                SAT::Result satResult;
                if (!SAT::boxTri(*collider, *tri, satResult)) {
                    continue;
                }

                glm::vec3& centerBox = std::get<OOBB>(collider->shape).wCenter;
                SAT::reverseNormal(centerBox, satResult.tri_ptr->centroid, satResult.normal);
                allResults.push_back(satResult);
            }

            if (allResults.size() == 0) {
                continue;
            }

            //body->setHelperMatrices();  // update helper matrixes and aabb faces
            body->updateInertiaWorld();

            Contact contact(colliderHandle, nullptr);

            glm::vec3 avgNormal{ 0.0f }; 
            for (const SAT::Result& res : allResults) { 
                avgNormal += res.normal; 
            }
            avgNormal = glm::normalize(avgNormal); // average normal  
            contact.normal = avgNormal;

            if (body->player) {
                pushAwayPlayer(*obj, false, avgNormal, allResults[0].depth);
                continue;
            }

            SAT::findBestTriangles(allResults);
            collisionManifold->boxMesh(contact, contactCache, allResults);
        }
        // SPHERE vs tris
        else if (collider->type == ColliderType::SPHERE) {
            for (Tri* tri : th.tris) {
                SAT::Result satResult;
                if (!SAT::sphereTri(*collider, *tri, satResult)) {
                    continue;
                }
                SAT::reverseNormal(obj->transform.position, tri->centroid, satResult.normal); // reverse normal to point outwards
                allResults.push_back(satResult);
            }

            if (allResults.size() == 0) {
                continue;
            }

            //obj->setHelperMatrices();
            body->updateInertiaWorld(); 

            glm::vec3 avgNormal{ 0.0f };
            for (const SAT::Result& res : allResults) {
                avgNormal += res.normal;
            }
            avgNormal = glm::normalize(avgNormal); // average normal 

            Contact contact(colliderHandle, nullptr);
            contact.normal = avgNormal; 

            SAT::findBestTriangles(allResults);

            collisionManifold->sphereMesh(contact, contactCache, allResults); 
        }
    }

    // ----- Dynamic vs dynamic -----
    for (const DynamicPair& dh : dynamicHits) {
        Collider* colliderA = colliderPtrCache.get(dh.A, __FUNCTION__);
        Collider* colliderB = colliderPtrCache.get(dh.B, __FUNCTION__);

        RigidBody* bodyA = bodyPtrCache.get(colliderA->rigidBodyHandle, __FUNCTION__);
        RigidBody* bodyB = bodyPtrCache.get(colliderB->rigidBodyHandle, __FUNCTION__);

        if (bodyA->type == BodyType::Static and 
            bodyB->type == BodyType::Static) {
            continue;
        }

        GameObject* objA = gameObjectPtrCache.get(bodyA->gameObjectHandle, __FUNCTION__);
        GameObject* objB = gameObjectPtrCache.get(bodyB->gameObjectHandle, __FUNCTION__);

        // CUBE vs CUBE
        if (colliderA->type == ColliderType::CUBOID and colliderB->type == ColliderType::CUBOID) {
            SAT::Result satResult;
            if (!SAT::boxBox(*colliderA, *colliderB, satResult)) {
                continue;
            }
            bodyA->totalCollisionCount++;
            bodyB->totalCollisionCount++;

            glm::vec3& centerA = std::get<OOBB>(colliderA->shape).wCenter;
            glm::vec3& centerB = std::get<OOBB>(colliderB->shape).wCenter;
            SAT::reverseNormal(centerA, centerB, satResult.normal);

            if (objA->player) {
                pushAwayPlayer(*objA, false, satResult.normal, satResult.depth);
                continue;
            }
            if (objB->player) {
                pushAwayPlayer(*objB, true, satResult.normal, satResult.depth);
                continue;
            }

            // used only for worldInertia (so far)
            //objA->setHelperMatrices();
            //objB->setHelperMatrices();
            bodyA->updateInertiaWorld();
            bodyB->updateInertiaWorld();

            Contact contact(objA, objB);

            PhysicsEngine::WakeUpInfo wakeInfo = wakeUpCheck(*bodyA, *bodyB);

            // freeze if kinematic and colliding with dynamic or kinematic, or if sleeping and not woken up by this collision
            bool frozenA =
                bodyA->type == BodyType::Static ||
                bodyA->type == BodyType::Kinematic ||
                (bodyA->type == BodyType::Dynamic && bodyA->asleep && !wakeInfo.A);

            bool frozenB =
                bodyB->type == BodyType::Static ||
                bodyB->type == BodyType::Kinematic ||
                (bodyB->type == BodyType::Dynamic && bodyB->asleep && !wakeInfo.B);

            contact.freezeA = frozenA;
            contact.freezeB = frozenB;

            collisionManifold->boxBox(contact, contactCache, satResult);
        }
        // CUBE vs SPHERE
        else if (
            (colliderA->type == ColliderType::CUBOID and colliderB->type == ColliderType::SPHERE) or
            (colliderA->type == ColliderType::SPHERE and colliderB->type == ColliderType::CUBOID)) 
        {
            // swap if A is not cuboid
            if (colliderA->type != ColliderType::CUBOID) {
                std::swap(objA, objB);
                std::swap(bodyA, bodyB);
            }

            // used in SAT
            //objA->setHelperMatrices();
            //objB->setHelperMatrices();
            bodyA->updateInertiaWorld();
            bodyB->updateInertiaWorld();

            SAT::Result satResult;
            if (!SAT::boxSphere(*colliderA, *colliderB, objA->transform, satResult)) {
                continue;
            }

            bodyA->totalCollisionCount++;
            bodyB->totalCollisionCount++;

            if (objA->player) {
                pushAwayPlayer(*objA, false, satResult.normal, satResult.depth);
                continue;
            }
            if (objB->player) {
                pushAwayPlayer(*objB, true, satResult.normal, satResult.depth);
                continue;
            }

            Contact contact(objA, objB);

            PhysicsEngine::WakeUpInfo wakeInfo = wakeUpCheck(*bodyA, *bodyB);
            bool frozenA =
                bodyA->type == BodyType::Static ||
                bodyA->type == BodyType::Kinematic ||
                (bodyA->type == BodyType::Dynamic && bodyA->asleep && !wakeInfo.A);

            bool frozenB =
                bodyB->type == BodyType::Static ||
                bodyB->type == BodyType::Kinematic ||
                (bodyB->type == BodyType::Dynamic && bodyB->asleep && !wakeInfo.B);

            contact.freezeA = frozenA;
            contact.freezeB = frozenB;

            collisionManifold->boxSphere(contact, contactCache, satResult);
        }
        // SPHERE vs SPHERE
        else if (
            colliderA->type == ColliderType::SPHERE and 
            colliderB->type == ColliderType::SPHERE) 
        {
            SAT::Result satResult;
            if (!SAT::sphereSphere(*colliderA, *colliderB, satResult)) {
                continue;
            }

            // used only for worldInertia
            //objA->setHelperMatrices();
            //objB->setHelperMatrices();

            bodyA->totalCollisionCount++;
            bodyB->totalCollisionCount++;

            Contact contact(objA, objB);

            PhysicsEngine::WakeUpInfo wakeInfo = wakeUpCheck(*bodyA, *bodyB);
            bool frozenA =
                bodyA->type == BodyType::Static ||
                bodyA->type == BodyType::Kinematic ||
                (bodyA->type == BodyType::Dynamic && bodyA->asleep && !wakeInfo.A);

            bool frozenB =
                bodyB->type == BodyType::Static ||
                bodyB->type == BodyType::Kinematic ||
                (bodyB->type == BodyType::Dynamic && bodyB->asleep && !wakeInfo.B);

            contact.freezeA = frozenA;
            contact.freezeB = frozenB;

            collisionManifold->sphereSphere(contact, contactCache, satResult);  
        }
    }
}

//---------------------------------------------
//         Collect active contacts
//---------------------------------------------
void PhysicsEngine::collectActiveContacts() {
    ScopedTimer t(*frameTimers, "Contact collection");

    contactsToSolve.clear();
    contactsToSolve.reserve(contactCache.size());

    for (auto& [key, c] : contactCache) {
        if (!c.wasUsedThisFrame) continue;

        const bool aActive = !c.freezeA;
        const bool bActive = (c.objB_ptr && !c.freezeB);
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
                return p1.globalCoord.y < p2.globalCoord.y;
            });
        return it->globalCoord.y;
        };

    std::sort(contactsToSolve.begin(), contactsToSolve.end(),
        [&](const Contact* a, const Contact* b) {
            float ay = minY(a), by = minY(b);
            if (ay < by) return true;
            if (by < ay) return false;
            // tie-breaker for determinism:
            return a->hashKey < b->hashKey;
        });

    debugData.collisions = contactsToSolve.size();
}

//---------------------------------------------
//            Resolve collisions
//---------------------------------------------
void PhysicsEngine::resolveCollisions() {
    ScopedTimer t(*frameTimers, "Collision resolution");

    // Justera beroende på material
    constexpr float staticFriction  = 0.6f;
    constexpr float dynamicFriction = 0.4f;
    constexpr float twistFriction   = 0.1f;

    float lastResidualSq = 0.0f;

    for (Contact* contact : contactsToSolve) { 
        GameObject& objA = *contact->objA_ptr; 
        GameObject& objB = *contact->objB_ptr; 

        // ----- Baumgarte stabilization -----
        float slop = 0.0005f; 
        float baumgarteFactor = 0.15f; 
        if (contact->freezeA or contact->freezeB) {
            baumgarteFactor = 0.30f;
            slop = 0.00005f;
        }

        for (int j = 0; j < contact->points.size(); j++) { 
            ContactPoint& cp = contact->points[j]; 

            float penetration = cp.depth; 
            float allowed = penetration - slop; 

            if (allowed > 0.0f) {
                cp.biasVelocity = -((baumgarteFactor * allowed) / dt);
            } else {
                cp.biasVelocity = 0.0f;
            }
        }

        // ----- Warm start -----
        constexpr float warmStartFactor = 0.9f;

        // skip warm starting if frozen
        if (contact->freezeA or contact->freezeB) {
            for (ContactPoint& cp : contact->points) {
                cp.accumulatedImpulse = 0.f;
                cp.accumulatedFrictionImpulse1 = 0.f;
                cp.accumulatedFrictionImpulse2 = 0.f;
            }
            contact->accumulatedTwistImpulse = 0.f;
        } 
        // normal warm starting
        else {
            for (ContactPoint& cp : contact->points) {
                cp.accumulatedImpulse *= warmStartFactor;
                cp.accumulatedFrictionImpulse1 *= warmStartFactor;
                cp.accumulatedFrictionImpulse2 *= warmStartFactor;

                // total impuls från föregående steg 
                glm::vec3 Pn = cp.accumulatedImpulse * contact->normal;
                glm::vec3 Pt = cp.accumulatedFrictionImpulse1 * contact->t1 + cp.accumulatedFrictionImpulse2 * contact->t2;

                // linjär + tangentiell
                glm::vec3 J = Pn + Pt;

                if (contact->objA_ptr and !contact->freezeA) {
                    objA.applyImpulseLinear(-J * objA.invMass);
                    objA.applyImpulseAngular(-objA.inverseInertiaWorld * glm::cross(cp.rA, J));
                }
                if (contact->objB_ptr and !contact->freezeB) {
                    objB.applyImpulseLinear(J * objB.invMass);
                    objB.applyImpulseAngular(objB.inverseInertiaWorld * glm::cross(cp.rB, J));
                }
            }

            if (contact->accumulatedTwistImpulse != 0.0f) {
                glm::vec3 tauWS = (contact->accumulatedTwistImpulse * warmStartFactor) * contact->normal;
                if (!contact->freezeA) objA.applyImpulseAngular(-objA.inverseInertiaWorld * tauWS);
                if (!contact->freezeB) objB.applyImpulseAngular(objB.inverseInertiaWorld * tauWS);
            }
        }
    } 

    // ------ PGS solver ------
    int maxIterations = 8; 
    int iterCount = 0;
    for (int i = 0; i < maxIterations; i++) {
        float maxDelta = 0.0f;
        float residualSq = 0.0f;

        for (Contact* contact : contactsToSolve) {

            GameObject& objA = *contact->objA_ptr;
            GameObject& objB = *contact->objB_ptr;

            for (int j = 0; j < contact->points.size(); j++) {
                ContactPoint& cp = contact->points[j];

                glm::vec3 relVelA{ 0.0f };
                glm::vec3 relVelB{ 0.0f };
                glm::vec3 angVelA{ 0.0f };
                glm::vec3 angVelB{ 0.0f };
                if (contact->objA_ptr and !contact->freezeA) {
                    relVelA = objA.linearVelocity;
                    angVelA = objA.angularVelocity;
                }
                if (contact->objB_ptr and !contact->freezeB) {
                    relVelB = objB.linearVelocity;
                    angVelB = objB.angularVelocity;
                }

                glm::vec3 relativeVelocity =
                    (relVelB + glm::cross(angVelB, cp.rB)) -
                    (relVelA + glm::cross(angVelA, cp.rA));

                float normalVelocity = glm::dot(relativeVelocity, contact->normal);

                // Baumgarte-bias built into target velocity
                float v_target = cp.targetBounceVelocity;
                float v_bias = cp.biasVelocity;

                // Condition: normalVelocity + v_bias = desired velocity at the contact
                float J = -(normalVelocity - v_target + v_bias) * cp.m_eff;

                // Clamp
                float temp = cp.accumulatedImpulse;
                cp.accumulatedImpulse = glm::max(temp + J, 0.0f);
                float deltaImpulse = cp.accumulatedImpulse - temp;

                residualSq += deltaImpulse * deltaImpulse;

                // Add normal to impulse
                glm::vec3 deltaNormalImpulse = deltaImpulse * contact->normal;

                // Apply impulses
                float deltaNormalImpulseLen = glm::dot(deltaNormalImpulse, deltaNormalImpulse);
                if (deltaNormalImpulseLen > 1e-6f) {
                    if (contact->objA_ptr and !contact->freezeA) {
                        objA.applyImpulseLinear(-deltaNormalImpulse * objA.invMass);
                        objA.applyImpulseAngular(-objA.inverseInertiaWorld * glm::cross(cp.rA, deltaNormalImpulse));
                    }
                    if (contact->objB_ptr and !contact->freezeB) {
                        objB.applyImpulseLinear(deltaNormalImpulse * objB.invMass);
                        objB.applyImpulseAngular(objB.inverseInertiaWorld * glm::cross(cp.rB, deltaNormalImpulse));
                    }
                }
                maxDelta = std::max(maxDelta, std::abs(deltaImpulse)); 

                //--------------- Pre-calculate for friction ----------------
                relVelA = glm::vec3{ 0.0f };
                relVelB = glm::vec3{ 0.0f };
                angVelA = glm::vec3{ 0.0f };
                angVelB = glm::vec3{ 0.0f };
                if (contact->objA_ptr and !contact->freezeA) {
                    relVelA = objA.linearVelocity;
                    angVelA = objA.angularVelocity;
                }
                if (contact->objB_ptr and !contact->freezeB) {
                    relVelB = objB.linearVelocity;
                    angVelB = objB.angularVelocity;
                }

                relativeVelocity =
                    (relVelB + glm::cross(angVelB, cp.rB)) -
                    (relVelA + glm::cross(angVelA, cp.rA));

                // Project tangential velocity to the tangent plane
                float v_t1 = glm::dot(relativeVelocity, contact->t1);
                float v_t2 = glm::dot(relativeVelocity, contact->t2);

                // Beräkna delta i tangentplanet
                float dF1 = -v_t1 * cp.invMassT1;
                float dF2 = -v_t2 * cp.invMassT2;

                // Kandidat: ny TOTAL friktionsimpuls (ackumulerad + delta)
                float newF1 = cp.accumulatedFrictionImpulse1 + dF1;
                float newF2 = cp.accumulatedFrictionImpulse2 + dF2;

                float Jn = std::abs(cp.accumulatedImpulse);
                float maxStatic = staticFriction * Jn;
                float maxStatic2 = maxStatic * maxStatic;

                float newLen2 = newF1 * newF1 + newF2 * newF2;
                float dT = 0.0f;

                if (newLen2 <= maxStatic2) {
                    // Statisk friktion: acceptera hela delta
                    cp.accumulatedFrictionImpulse1 = newF1;
                    cp.accumulatedFrictionImpulse2 = newF2;
                    glm::vec3 dFt = dF1 * contact->t1 + dF2 * contact->t2;
                    dT = std::sqrt(dF1 * dF1 + dF2 * dF2);

                    residualSq += dT * dT;

                    // Applicera den statiska friktionsimpulsen:
                    if (contact->objA_ptr and !contact->freezeA) {
                        objA.applyImpulseLinear(-dFt * objA.invMass);
                        objA.applyImpulseAngular(-objA.inverseInertiaWorld * glm::cross(cp.rA, dFt));
                    }
                    if (contact->objB_ptr and !contact->freezeB) {
                        objB.applyImpulseLinear(dFt * objB.invMass);
                        objB.applyImpulseAngular(objB.inverseInertiaWorld * glm::cross(cp.rB, dFt));
                    }
                } 
                else {
                // ---------- Dynamic friction ----------
                    // Dynamisk friktion: projektera TOTAL impuls till cirkeln μ_d * Jn
                    float maxDyn = dynamicFriction * Jn;

                    float len = std::sqrt(newLen2);
                    if (len > 1e-6f) {
                        float s = maxDyn / len;
                        float clampedF1 = newF1 * s;
                        float clampedF2 = newF2 * s;

                        float d1 = clampedF1 - cp.accumulatedFrictionImpulse1;
                        float d2 = clampedF2 - cp.accumulatedFrictionImpulse2;

                        dT = std::sqrt(d1 * d1 + d2 * d2);

                        cp.accumulatedFrictionImpulse1 = clampedF1;
                        cp.accumulatedFrictionImpulse2 = clampedF2;

                        glm::vec3 dFt = d1 * contact->t1 + d2 * contact->t2;
                        if (contact->objA_ptr and !contact->freezeA) {

                            residualSq += dT * dT;

                            objA.applyImpulseLinear(-dFt * objA.invMass);
                            objA.applyImpulseAngular(-objA.inverseInertiaWorld * glm::cross(cp.rA, dFt));
                        }
                        if (contact->objB_ptr and !contact->freezeB) {
                            objB.applyImpulseLinear(dFt * objB.invMass);
                            objB.applyImpulseAngular(objB.inverseInertiaWorld * glm::cross(cp.rB, dFt));
                        }
                    }
                }
                maxDelta = std::max(maxDelta, std::abs(dT));
            }

            // ---------- Twist friction (per manifold) ----------
            // 1) Relativ rotationshastighet kring normalen
            glm::vec3 angVelA{ 0.0f };
            glm::vec3 angVelB{ 0.0f };
            if (contact->objA_ptr && !contact->freezeA) angVelA = objA.angularVelocity;
            if (contact->objB_ptr && !contact->freezeB) angVelB = objB.angularVelocity;

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

                residualSq += delta * delta;

                if (contact->objA_ptr && !contact->freezeA) {
                    objA.applyImpulseAngular(-objA.inverseInertiaWorld * tau);
                }
                if (contact->objB_ptr && !contact->freezeB) {
                    objB.applyImpulseAngular(objB.inverseInertiaWorld * tau);
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

//-----------------------------
//       Wake up check
//-----------------------------
PhysicsEngine::WakeUpInfo PhysicsEngine::wakeUpCheck(RigidBody& A, RigidBody& B) {
    constexpr float velocityThreshold = 1.2f; // hälften för 120hz
    constexpr float angularThreshold = 0.8f;
    constexpr float v2 = velocityThreshold * velocityThreshold;
    constexpr float w2 = angularThreshold * angularThreshold;

    // Beräkna lokalt (aktuella värden just nu i narrow)
    const float Av2 = glm::dot(A.linearVelocity, A.linearVelocity);
    const float Aw2 = glm::dot(A.angularVelocity, A.angularVelocity);
    const float Bv2 = glm::dot(B.linearVelocity, B.linearVelocity);
    const float Bw2 = glm::dot(B.angularVelocity, B.angularVelocity);

    WakeUpInfo info{ false, false };

    if (A.asleep and !A.isStatic and A.allowSleep) {
        if (Bv2 > v2 || Bw2 > w2) {
            info.A = true;
            if (!A.inSleepTransition) {
                toWake.push_back(&A);
                A.inSleepTransition = true;
            }
        }
    }
    if (B.asleep and !B.isStatic and B.allowSleep) {
        if (Av2 > v2 || Aw2 > w2) {
            info.B = true;
            if (!B.inSleepTransition) {
                toWake.push_back(&B);
                B.inSleepTransition = true;
            }
        }
    }

    return info;
}

//-----------------------------
//       Sleep Thresholds
//-----------------------------
void PhysicsEngine::updateSleepThresholds() {
    const std::vector<GameObjectHandle>& awakeHandles = broadphaseManager.getAwakeList();
    for (const GameObjectHandle& handle : awakeHandles) {
        GameObject* obj = stepCache.get(handle);

        if (!obj) {
            std::cout << "Error: Object handle in awake list is invalid. Function: updateSleepThresholds\n";
            continue;
        }

        if (obj->isStatic or obj->asleep or !obj->allowSleep)
            continue;

        obj->collisionHistory.push(obj->totalCollisionCount);
        obj->totalCollisionCount = 0;
        float avg = obj->collisionHistory.average();

        if (avg <= 0.0f) {
            if (std::abs(avg - obj->lastAvg) >= 1) {
                obj->sleepCounter = 0.0f;
            }
            obj->lastAvg = avg;

            continue;
        }

        avg = std::max(avg, 1.0f);
        obj->lastAvg = avg;

        constexpr float linearFactor = 0.17f;
        constexpr float angularFactor = 0.10f;

        // set thresholds
        obj->velocityThreshold = avg * linearFactor;
        obj->angularVelocityThreshold = avg * angularFactor * obj->invRadius;
    }
}

//-------------------------------
//      Decide sleep/awake
//-------------------------------
void PhysicsEngine::decideSleep() 
{
    constexpr float jitterThreshold = 1.0f; // threshold for considering an object to be jittering
    constexpr float anchorTimerThreshold = 5.0f; // time an object must be within the jitter threshold to fall asleep 

    const std::vector<GameObjectHandle>& awakeHandles = broadphaseManager.getAwakeList();
    for (GameObjectHandle handle : awakeHandles) {
        GameObject* obj = stepCache.get(handle);

        if (!obj) {
            std::cout << "Error: Object handle in awake list is invalid. Function: decideSleep\n";
            continue;
        }

        if (!obj->allowSleep) continue;
        if (obj->inSleepTransition) continue;

        bool goingToSleep = false;

        // anchor point logic
        if (glm::abs(obj->anchorPoint.x - obj->position.x) < jitterThreshold &&
            glm::abs(obj->anchorPoint.y - obj->position.y) < jitterThreshold &&
            glm::abs(obj->anchorPoint.z - obj->position.z) < jitterThreshold) 
        {
            // object is within jitter threshold of anchor point, increase timer
            obj->anchorTimer += dt;
        }
        else {
            // decrease timer if object moves away from anchor point (but never below 0)
            obj->anchorTimer = glm::max(0.0f, obj->anchorTimer - dt);
        }

        // set anchor point if timer is 0
        if (obj->anchorTimer == 0.0f) {
            obj->anchorPoint = obj->position;
        }

        // check if anchor timer has exceeded threshold
        if (obj->anchorTimer >= anchorTimerThreshold) {
            goingToSleep = true;
        }

        // sleep counter
        if (glm::length(obj->linearVelocity) < obj->velocityThreshold and
            glm::length(obj->angularVelocity) < obj->angularVelocityThreshold)
        {
            obj->sleepCounter += dt;
        }
        else {
            obj->sleepCounter = 0.0f;
        }

        if (obj->sleepCounter >= obj->sleepCounterThreshold) {
            goingToSleep = true;
        }

        if (goingToSleep) {
            toSleep.push_back(handle);
        }
    }

    for (GameObjectHandle& handle : toSleep) {
        broadphaseManager.moveToAsleep(handle);
    }

    for (GameObjectHandle& handle : toWake) {
        GameObject* obj = stepCache.get(handle);

        if (!obj) {
            std::cout << "Error: Object handle in wake list is invalid. Function: decideSleep\n";
            continue;
        }

        broadphaseManager.moveToAwake(handle);
        obj->inSleepTransition = false;
    }
}

//-----------------------------
//       Contact Cache
//-----------------------------
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

void PhysicsEngine::pushAwayPlayer(GameObject& player, bool playerIsA, glm::vec3& n, float d) {
    glm::vec3 up;

    if (playerIsA) {
        player.position += n * (d + 0.001f);
        up = glm::vec3(0.0f, 1.0f, 0.0f);
    } else {
        player.position -= n * (d - 0.001f);
        up = glm::vec3(0.0f, -1.0f, 0.0f);
    }

    player.onGround = false;

    // Ground check
    float t = glm::dot(up, n);
    if (t > 0.75f) {
        player.onGround = true;
    }
}