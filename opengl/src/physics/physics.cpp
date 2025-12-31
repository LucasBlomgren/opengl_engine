#include "pch.h"
#include "physics.h"
#include "aabb.h"
#include "sat.h"

template class BVHTree<GameObject>;

//-----------------------------
//         Setup scene
//-----------------------------
void PhysicsEngine::setupScene(std::vector<GameObject>* gameObjects, std::vector<Tri>* terrainTris) {
    this->dynamicObjects = gameObjects;

    awake_DynamicObjects.clear();
    awake_DynamicObjects.reserve(5000000);
    asleep_DynamicObjects.clear();
    asleep_DynamicObjects.reserve(5000000);
    static_Objects.clear();
    static_Objects.reserve(5000000);

    dynamicAwakeBvh.nodes.reserve(5000000);
    dynamicAwakeBvh.rootIdx = -1;
    dynamicAwakeBvh.nodes.clear();

    dynamicAsleepBvh.nodes.reserve(5000000);
    dynamicAsleepBvh.rootIdx = -1;
    dynamicAsleepBvh.nodes.clear();

    staticBvh.nodes.reserve(5000000);
    staticBvh.rootIdx = -1;
    staticBvh.nodes.clear();

    insertPendingObjects();
    dynamicAwakeBvh.build(*gameObjects, awake_DynamicObjects, false);
    dynamicAsleepBvh.build(*gameObjects, asleep_DynamicObjects, false);
    staticBvh.build(*gameObjects, static_Objects, false);

    this->terrainTriangles = terrainTris;
    std::vector<int> placeholder;
    terrainBvh.build(*terrainTris, placeholder, true);

    toWake.reserve(dynamicObjects->size());
    toSleep.reserve(dynamicObjects->size());
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
//     Add/Remove commands
//-----------------------------
void PhysicsEngine::queueAdd(GameObject* o) { 
    pending.push_back({ PhysCmd::Add, o }); 
}
void PhysicsEngine::queueRemove(GameObject* o) { 
    pending.push_back({ PhysCmd::Remove, o }); 
}

//-----------------------------
//          Getters
//-----------------------------
BVHTree<GameObject>& PhysicsEngine::getDynamicAwakeBvh() {
    return dynamicAwakeBvh;
}
BVHTree<GameObject>& PhysicsEngine::getDynamicAsleepBvh() {
    return dynamicAsleepBvh;
}
BVHTree<GameObject>& PhysicsEngine::getStaticBvh() {
    return staticBvh;
}
BVHTree<Tri>& PhysicsEngine::getTerrainBvh() {
    return terrainBvh;
}
const std::unordered_map<size_t, Contact>& PhysicsEngine::GetContactCache() const {
    return contactCache;
}

//-----------------------------
//           Raycast
//-----------------------------
RaycastHit PhysicsEngine::performRaycast(Ray& r) {
    RaycastHit a = raycast(r, this->dynamicAwakeBvh);
    RaycastHit b = raycast(r, this->dynamicAsleepBvh);
    RaycastHit c = raycast(r, this->staticBvh);

    RaycastHit bestHit = a;
    if (b.object != nullptr) {
        if (bestHit.object == nullptr || b.t < bestHit.t) {
            bestHit = b;
        }
    }
    if (c.object != nullptr) {
        if (bestHit.object == nullptr || c.t < bestHit.t) {
            bestHit = c;
        }
    }
    return bestHit;
}

//-----------------------------
//         Sleep All
//-----------------------------
void PhysicsEngine::sleepAllObjects() {
    for (GameObject& obj : *dynamicObjects) {
        moveToAsleep(obj);
    }
}

//-----------------------------
//         Wake All
//-----------------------------
void PhysicsEngine::awakenAllObjects() {
    for (GameObject& obj : *dynamicObjects) {
        moveToAwake(obj);
    }
}

//-----------------------------
//         Time step
//-----------------------------
void PhysicsEngine::step(float deltaTime, std::mt19937 rng) {
    ScopedTimer t(*frameTimers, "Physics");

    // Pre-step preparations
    {
        ScopedTimer t(*frameTimers, "Pre step");
        this->dt = deltaTime;

        // prepare for this frame
        toWake.reserve(dynamicObjects->size());
        toWake.clear();
        toSleep.reserve(dynamicObjects->size());
        toSleep.clear();

        // add/remove objects to the BVH trees
        insertPendingObjects();

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
        /*if (dynamicAwakeBvh.dirty)*/ dynamicAwakeBvh.update(*dynamicObjects, awake_DynamicObjects, false);
        if (dynamicAsleepBvh.dirty) dynamicAsleepBvh.update(*dynamicObjects, asleep_DynamicObjects, false);
        /*if (staticBvh.dirty)*/ staticBvh.update(*dynamicObjects, static_Objects, false);

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
    for (int idx : awake_DynamicObjects) {
        GameObject& obj = (*dynamicObjects)[idx];
        obj.resetDirtyFlags();
        obj.updatePos(this->dt);
        obj.updateAABB();
        obj.updateCollider();
    }
}

//-------------------------------
//     Insert pending objects
//-------------------------------
void PhysicsEngine::insertPendingObjects() {
    // add/remove objects to the BVH trees
    for (auto& c : pending) {
        if (c.obj->isStatic) {
            moveToStatic(*c.obj);
        } else if (c.obj->asleep) {
            moveToAsleep(*c.obj);
        } else {
            moveToAwake(*c.obj);
        }
    }
    pending.clear();
}

//-----------------------------
//       Move to awake
//-----------------------------
inline void PhysicsEngine::moveToAwake(GameObject& obj) {
    if (obj.isStatic) return;
    if (obj.awakeListIdx != -1) { obj.setAwake(); return; } // redan där

    if (obj.asleepListIdx != -1) { // ta ur asleep
        int last = (int)asleep_DynamicObjects.size() - 1;
        if (obj.asleepListIdx != last) {
            GameObject& lastObj = (*dynamicObjects)[asleep_DynamicObjects[last]];
            lastObj.asleepListIdx = obj.asleepListIdx;
            asleep_DynamicObjects[obj.asleepListIdx] = lastObj.dynamicObjectIdx;
        }
        asleep_DynamicObjects.pop_back();
        obj.asleepListIdx = -1;
        dynamicAsleepBvh.removeLeaf(obj.bvhLeafIdx);
        dynamicAsleepBvh.dirty = true;
    }
    obj.setAwake(); // <- synka state
    awake_DynamicObjects.push_back(obj.dynamicObjectIdx);
    obj.awakeListIdx = (int)awake_DynamicObjects.size() - 1;
    dynamicAwakeBvh.insertLeaf(&obj);
}

//-----------------------------
//       Move to asleep
//-----------------------------
inline void PhysicsEngine::moveToAsleep(GameObject& obj) {
    if (obj.isStatic) return;
    if (obj.asleepListIdx != -1) { obj.setAsleep(); return; } // redan där

    if (obj.awakeListIdx != -1) { // ta ur awake
        int last = (int)awake_DynamicObjects.size() - 1;
        if (obj.awakeListIdx != last) {
            GameObject& lastObj = (*dynamicObjects)[awake_DynamicObjects[last]];
            lastObj.awakeListIdx = obj.awakeListIdx;
            awake_DynamicObjects[obj.awakeListIdx] = lastObj.dynamicObjectIdx;
        }
        awake_DynamicObjects.pop_back();
        obj.awakeListIdx = -1;
        dynamicAwakeBvh.removeLeaf(obj.bvhLeafIdx);
    }
    obj.setAsleep(); // <- synka state
    asleep_DynamicObjects.push_back(obj.dynamicObjectIdx);
    obj.asleepListIdx = (int)asleep_DynamicObjects.size() - 1;
    dynamicAsleepBvh.insertLeaf(&obj);
    dynamicAsleepBvh.dirty = true;
}

//-----------------------------
//       Move to static
//-----------------------------
inline void PhysicsEngine::moveToStatic(GameObject& obj) {
    if (!obj.isStatic) return;
    if (obj.staticListIdx != -1) { return; } // redan där

    obj.awakeListIdx = -1;
    obj.asleepListIdx = -1;

    static_Objects.push_back(obj.dynamicObjectIdx);
    obj.staticListIdx = (int)static_Objects.size() - 1;
    staticBvh.insertLeaf(&obj);

    staticBvh.dirty = true;
}

//---------------------------------------------
//      Collision detection & resolution
//---------------------------------------------
void PhysicsEngine::detectAndSolveCollisions() {
    static std::vector<TerrainHit> tHits; 
    static std::vector<DynamicHit> dHits; 

    tHits.reserve(BVHTree<Tri>::MaxCollisionBuf); 
    dHits.reserve(BVHTree<GameObject>::MaxCollisionBuf); 

    broadPhase(tHits, dHits);       // hashmap kanske långsam? 
    narrowPhase(tHits, dHits);      // SAT + collisionManifold
    collectActiveContacts();        // collect contacts to solve
    resolveCollisions();            // PGS + Baumgarte stabilization
}

//---------------------------------------------
//                Broadphase
//---------------------------------------------
void PhysicsEngine::broadPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits) {
    ScopedTimer t(*frameTimers, "Broadphase");

    // ----- dynamic vs terrain -----
    if (terrainBvh.rootIdx != -1) {
        static std::vector<std::pair<GameObject*, Tri*>> hitsBufTerrain;         // hits buffer
        hitsBufTerrain.reserve(BVHTree<Tri>::MaxCollisionBuf);
        hitsBufTerrain.clear();
        treeVsTreeQuery(dynamicAwakeBvh, terrainBvh, hitsBufTerrain);          // query terrain vs dynamic objects

        // Sort terrain hits by GameObject to avoid duplicates 
        std::unordered_map<GameObject*, std::vector<Tri*>> temp;
        temp.reserve(hitsBufTerrain.size());
        if (temp.bucket_count() < hitsBufTerrain.size())
            temp.reserve(hitsBufTerrain.size());

        for (int i = 0; i < hitsBufTerrain.size(); i++) {
            auto [obj, tri] = hitsBufTerrain[i];
            temp[obj].push_back(tri);
        }

        // Finalize terrain hits
        tHits.clear();
        int cap = static_cast<int>(temp.size());
        tHits.resize(cap);
        int sp = 0;

        for (auto& [obj, trisVec] : temp) {
            tHits[sp++] = TerrainHit{ obj, std::move(trisVec) };
        }
        tHits.resize(sp);
    }

    static std::vector<std::pair<GameObject*, GameObject*>> hitsBufDynamic; // hits buffer
    dHits.clear();

    // ----- dynamic vs dynamic -----
    if (dynamicAwakeBvh.rootIdx != -1) {
        hitsBufDynamic.clear();
        treeVsTreeQuery(dynamicAwakeBvh, dynamicAwakeBvh, hitsBufDynamic);

        int cap = static_cast<int>(hitsBufDynamic.size());
        dHits.resize(cap);
        int sp = 0;

        for (auto& hp : hitsBufDynamic) {
            GameObject* A = hp.first, * B = hp.second;

            if (A == B) continue;
            dHits[sp++] = DynamicHit{ A,B };
        }
        dHits.resize(sp);
    }

    // ----- dynamic vs asleep -----
    if (dynamicAsleepBvh.rootIdx != -1) {
        hitsBufDynamic.clear();
        treeVsTreeQuery(dynamicAwakeBvh, dynamicAsleepBvh, hitsBufDynamic);

        int cap = static_cast<int>(hitsBufDynamic.size());
        dHits.reserve(dHits.size() + cap);
        int sp = 0;

        for (auto& hp : hitsBufDynamic) {
            GameObject* A = hp.first, * B = hp.second;

            if (A == B) continue;
            dHits.emplace_back(DynamicHit{ A,B });
        }
    }

    // ----- dynamic vs static -----
    if (staticBvh.rootIdx != -1) {
        hitsBufDynamic.clear();
        treeVsTreeQuery(dynamicAwakeBvh, staticBvh, hitsBufDynamic);

        int cap = static_cast<int>(hitsBufDynamic.size());
        dHits.reserve(dHits.size() + cap);
        int sp = 0;

        for (auto& hp : hitsBufDynamic) {
            GameObject* A = hp.first, * B = hp.second;

            if (A == B) continue;
            dHits.emplace_back(DynamicHit{ A,B });
        }
    }
}

//---------------------------------------------
//               Narrow phase
//---------------------------------------------
void PhysicsEngine::narrowPhase(std::vector<TerrainHit>& terrainHits, std::vector<DynamicHit>& dynamicHits) {
    ScopedTimer t(*frameTimers, "Narrowphase");

    // ----- Terrain vs dynamic ----- 
    for (TerrainHit& th : terrainHits) {
        if (th.obj->asleep) {
            continue;
        }

        static std::vector<SAT::Result> allResults;
        int reserveSize = th.tris.size() * 2; // worst case
        allResults.clear(); 
        allResults.reserve(reserveSize); // reserve space for results

        // CUBE vs tris
        if (th.obj->colliderType == ColliderType::CUBOID) {
            for (Tri* tri : th.tris) {
                SAT::Result satResult;
                if (!SAT::boxTri(th.obj->collider, *tri, satResult)) {
                    continue;
                }

                glm::vec3& centerBox = std::get<OOBB>(th.obj->collider.shape).wCenter;
                SAT::reverseNormal(centerBox, satResult.tri_ptr->centroid, satResult.normal);
                allResults.push_back(satResult);
            }

            if (allResults.size() == 0) {
                continue;
            }

            th.obj->setHelperMatrices();  // update helper matrixes and aabb faces

            Contact contact(th.obj, nullptr);

            glm::vec3 avgNormal{ 0.0f }; 
            for (const SAT::Result& res : allResults) { 
                avgNormal += res.normal; 
            }
            avgNormal = glm::normalize(avgNormal); // average normal  
            contact.normal = avgNormal;

            if (th.obj->player) {
                pushAwayPlayer(*th.obj, false, avgNormal, allResults[0].depth);
                continue;
            }

            SAT::findBestTriangles(allResults);

            collisionManifold->boxMesh(contact, contactCache, allResults);
        }
        // SPHERE vs tris
        else if (th.obj->colliderType == ColliderType::SPHERE) {
            for (Tri* tri : th.tris) { 
                SAT::Result satResult; 
                if (!SAT::sphereTri(th.obj->collider, *tri, satResult)) { 
                    continue; 
                }
                SAT::reverseNormal(th.obj->position, tri->centroid, satResult.normal); // reverse normal to point outwards
                allResults.push_back(satResult); 
            }

            if (allResults.size() == 0) { 
                continue; 
            }

            th.obj->setHelperMatrices();

            glm::vec3 avgNormal{ 0.0f };
            for (const SAT::Result& res : allResults) { 
                avgNormal += res.normal; 
            }
            avgNormal = glm::normalize(avgNormal); // average normal 

            Contact contact(th.obj, nullptr); 
            contact.normal = avgNormal; 

            SAT::findBestTriangles(allResults);

            collisionManifold->sphereMesh(contact, contactCache, allResults); 
        }
    }

    // ----- Dynamic vs dynamic -----
    for (DynamicHit& dh : dynamicHits) {
        if (dh.A->isStatic and dh.B->isStatic) {
            continue;
        }

        GameObject* objA = dh.A; 
        GameObject* objB = dh.B; 

        // CUBE vs CUBE
        if (objA->colliderType == ColliderType::CUBOID and objB->colliderType == ColliderType::CUBOID) {
            SAT::Result satResult;
            if (!SAT::boxBox(objA->collider, objB->collider, satResult)) {
                continue;
            }
            objA->totalCollisionCount++;
            objB->totalCollisionCount++;

            glm::vec3 centerA = std::get<OOBB>(objA->collider.shape).wCenter;
            glm::vec3 centerB = std::get<OOBB>(objB->collider.shape).wCenter;
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
            objA->setHelperMatrices();
            objB->setHelperMatrices();

            Contact contact(objA, objB);

            PhysicsEngine::WakeUpInfo wakeInfo = wakeUpCheck(*objA, *objB);
            // editor-freeze bara om motparten INTE är statisk
            bool editorFreezeA = (objA->selectedByEditor or objA->selectedByPlayer) and !objB->isStatic;
            bool editorFreezeB = (objB->selectedByEditor or objB->selectedByPlayer) and !objA->isStatic;
            contact.freezeA = editorFreezeA || (objA->asleep and !wakeInfo.A);
            contact.freezeB = editorFreezeB || (objB->asleep and !wakeInfo.B);

            collisionManifold->boxBox(contact, contactCache, satResult);
        }
        // CUBE vs SPHERE
        else if ((objA->colliderType == ColliderType::CUBOID and objB->colliderType == ColliderType::SPHERE) or 
                (objA->colliderType == ColliderType::SPHERE and objB->colliderType == ColliderType::CUBOID)) {
            // swap if A is not cuboid
            if (objA->colliderType != ColliderType::CUBOID) {
                std::swap(objA, objB);
            }

            // used in SAT
            objA->setHelperMatrices();
            objB->setHelperMatrices();

            SAT::Result satResult;

            if (!SAT::boxSphere(objA->collider, objB->collider, satResult)) {
                continue;
            }

            objA->totalCollisionCount++; 
            objB->totalCollisionCount++; 

            if (objA->player) {
                pushAwayPlayer(*objA, false, satResult.normal, satResult.depth);
                continue;
            }
            if (objB->player) {
                pushAwayPlayer(*objB, true, satResult.normal, satResult.depth);
                continue;
            }

            Contact contact(objA, objB); 

            PhysicsEngine::WakeUpInfo wakeInfo = wakeUpCheck(*objA, *objB);
            contact.freezeA = (objA->isStatic) || (objA->asleep && !wakeInfo.A);
            contact.freezeB = (objB->isStatic) || (objB->asleep && !wakeInfo.B);

            collisionManifold->boxSphere(contact, contactCache, satResult); 
        }
        // SPHERE vs SPHERE
        else if (objA->colliderType == ColliderType::SPHERE and objB->colliderType == ColliderType::SPHERE) {
            SAT::Result satResult;
            if (!SAT::sphereSphere(objA->collider, objB->collider, satResult)) { 
                continue;
            }

            // used only for worldInertia
            objA->setHelperMatrices();
            objB->setHelperMatrices();

            objA->totalCollisionCount++; 
            objB->totalCollisionCount++; 

            Contact contact(objA, objB); 

            PhysicsEngine::WakeUpInfo wakeInfo = wakeUpCheck(*objA, *objB);
            contact.freezeA = (objA->isStatic) || (objA->asleep && !wakeInfo.A);
            contact.freezeB = (objB->isStatic) || (objB->asleep && !wakeInfo.B);

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


    //loat residual = std::sqrt(lastResidualSq);
    //std::cout << "Residual impulse: " << residual << "\n";
}

//-----------------------------
//       Wake up check
//-----------------------------
PhysicsEngine::WakeUpInfo PhysicsEngine::wakeUpCheck(GameObject& A, GameObject& B) {
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
                toWake.push_back(A.dynamicObjectIdx);
                A.inSleepTransition = true;
            }
        }
    }
    if (B.asleep and !B.isStatic and B.allowSleep) {
        if (Av2 > v2 || Aw2 > w2) {
            info.B = true;
            if (!B.inSleepTransition) {
                toWake.push_back(B.dynamicObjectIdx);
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
    for (int idx : awake_DynamicObjects) {
        GameObject& obj = (*dynamicObjects)[idx];

        if (obj.isStatic or obj.asleep or !obj.allowSleep)
            continue;

        obj.collisionHistory.push(obj.totalCollisionCount);
        obj.totalCollisionCount = 0;
        float avg = obj.collisionHistory.average();

        if (avg <= 0.0f) {
            if (std::abs(avg - obj.lastAvg) >= 1) {
                obj.sleepCounter = 0.0f;
            }
            obj.lastAvg = avg;

            continue;
        }

        avg = std::max(avg, 1.0f);
        obj.lastAvg = avg;

        constexpr float linearFactor  = 0.17f;
        constexpr float angularFactor = 0.10f;

        // set thresholds
        obj.velocityThreshold = avg * linearFactor;
        obj.angularVelocityThreshold = avg * angularFactor * obj.invRadius;
    }
}

//-------------------------------
//      Decide sleep/awake
//-------------------------------
void PhysicsEngine::decideSleep() 
{
    for (int idx : awake_DynamicObjects) {
        GameObject& obj = (*dynamicObjects)[idx];
        if (obj.isStatic) continue;
        if (!obj.allowSleep) continue;
        //if (obj.inSleepTransition) continue;

        // sleep counter
        if (glm::length(obj.linearVelocity) < obj.velocityThreshold and 
            glm::length(obj.angularVelocity) < obj.angularVelocityThreshold)
        {
            obj.sleepCounter += dt;
        } else {
            obj.sleepCounter = 0.0f;
        }

        if (obj.sleepCounter >= obj.sleepCounterThreshold) {
            toSleep.push_back(idx);
        }
    }

    for (int idx : toSleep) {
        moveToAsleep((*dynamicObjects)[idx]);
    }

    for (int idx : toWake) {
        GameObject& obj = (*dynamicObjects)[idx];
        moveToAwake(obj);
        obj.inSleepTransition = false;
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