#include "pch.h"
#include "physics.h"
#include "aabb.h"
#include "sat.h"

//-----------------------------
//            Init
//-----------------------------
void PhysicsEngine::init(EngineState* engineState) {
    this->engineState = engineState;
    this->collisionManifold = new CollisionManifold();
}

//-----------------------------
//         Setup scene
//-----------------------------
void PhysicsEngine::setupScene(std::vector<GameObject>* gameObjects, std::vector<Tri>* terrainTris) {
    this->dynamicObjects = gameObjects;

    awake_DynamicObjects.clear();
    awake_DynamicObjects.reserve(25000);
    asleep_DynamicObjects.clear();
    asleep_DynamicObjects.reserve(25000);
    static_Objects.clear();
    static_Objects.reserve(20000);

    dynamicAwakeBvh.nodes.reserve(25000);
    dynamicAwakeBvh.rootIdx = -1;
    dynamicAwakeBvh.nodes.clear();

    dynamicAsleepBvh.nodes.reserve(25000);
    dynamicAsleepBvh.rootIdx = -1;
    dynamicAsleepBvh.nodes.clear();

    staticBvh.nodes.reserve(20000);
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

    if (a.object == nullptr && b.object != nullptr) return b;
    if (b.object == nullptr && a.object != nullptr) return a;

    return (a.t < b.t ? a : b);
}

//-----------------------------
//         Sleep All
//-----------------------------
void PhysicsEngine::sleepAllObjects() {
    for (GameObject& obj : *dynamicObjects) {
        if (!obj.isStatic) {
            obj.setAsleep();
        }
    }
}

//-----------------------------
//         Wake All
//-----------------------------
void PhysicsEngine::awakenAllObjects() {
    for (GameObject& obj : *dynamicObjects) {
        if (!obj.isStatic) {
            obj.setAwake();
        }
    }
}

//-----------------------------
//         Time step
//-----------------------------
void PhysicsEngine::step(float deltaTime, std::mt19937 rng) {
    this->dt = deltaTime;

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

    updateStates();

    dynamicAwakeBvh.update(*dynamicObjects, awake_DynamicObjects, false);

    if (asleepBvhDirty) {
        dynamicAsleepBvh.update(*dynamicObjects, asleep_DynamicObjects, false);
        asleepBvhDirty = false;
    }

    if (staticBvhDirty) {
        staticBvh.update(*dynamicObjects, static_Objects, false);
        staticBvhDirty = false;
    }

    detectAndSolveCollisions();

    for (GameObject& obj : *dynamicObjects) {
        updateSleepThresholds(obj);
    }

    decideSleep();

    updateContactCache();

    //std::cout << "Physics: " << dynamicAwakeBvh.nodes.size() + dynamicAsleepBvh.nodes.size() << " BVH nodes, " 
    //          << contactCache.size() << " contacts, "
    //          << awake_DynamicObjects.size() << " awake objects, "
    //    << asleep_DynamicObjects.size() << " asleep objects.\n";
}

//-----------------------------
//       Update methods
//-----------------------------
void PhysicsEngine::updateStates() {
    for (GameObject& obj : *dynamicObjects) {
        if (!obj.isStatic and !obj.asleep) {
            obj.resetDirtyFlags();

            obj.updatePos(this->dt);

            obj.updateAABB();
            obj.updateCollider();
        }
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

void PhysicsEngine::decideSleep() {
    for (int idx : toWake) {
        moveToAwake((*dynamicObjects)[idx]);
    }

    for (int idx : awake_DynamicObjects) {
        GameObject& obj = (*dynamicObjects)[idx];
        if (obj.isStatic) continue;

        // sleep counter
        if (std::abs(glm::length(obj.linearVelocity)) < obj.velocityThreshold and std::abs(glm::length(obj.angularVelocity)) < obj.angularVelocityThreshold) {
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
}

PhysicsEngine::WakeUpInfo PhysicsEngine::wakeUpCheck(GameObject& A, GameObject& B) {
    constexpr float velocityThreshold = 1.0f;
    constexpr float angularThreshold = 0.4f;
    constexpr float v2 = velocityThreshold * velocityThreshold;
    constexpr float w2 = angularThreshold * angularThreshold;

    // Beräkna lokalt (aktuella värden just nu i narrow)
    const float Av2 = glm::dot(A.linearVelocity, A.linearVelocity);
    const float Aw2 = glm::dot(A.angularVelocity, A.angularVelocity);
    const float Bv2 = glm::dot(B.linearVelocity, B.linearVelocity);
    const float Bw2 = glm::dot(B.angularVelocity, B.angularVelocity);

    WakeUpInfo info{ false, false };

    if (A.asleep && !A.isStatic) {
        if (Bv2 > v2 || Bw2 > w2) {
            toWake.push_back(A.dynamicObjectIdx);
            info.A = true;
        }
    }
    if (B.asleep && !B.isStatic) {
        if (Av2 > v2 || Aw2 > w2) {
            toWake.push_back(B.dynamicObjectIdx);
            info.B = true;
        }
    }

    return info;
}

inline void PhysicsEngine::moveToAwake(GameObject& obj) {
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
    }
    obj.setAwake(); // <- synka state
    awake_DynamicObjects.push_back(obj.dynamicObjectIdx);
    obj.awakeListIdx = (int)awake_DynamicObjects.size() - 1;
    dynamicAwakeBvh.insertLeaf(&obj);
}

inline void PhysicsEngine::moveToAsleep(GameObject& obj) {
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

    asleepBvhDirty = true;
}

inline void PhysicsEngine::moveToStatic(GameObject& obj) {
    if (!obj.isStatic) return;
    if (obj.staticListIdx != -1) { return; } // redan där

    obj.awakeListIdx = -1;
    obj.asleepListIdx = -1;

    static_Objects.push_back(obj.dynamicObjectIdx);
    obj.staticListIdx = (int)static_Objects.size() - 1;
    staticBvh.insertLeaf(&obj);

    staticBvhDirty = true;
}

//-----------------------------
//       Contact Cache
//-----------------------------
void PhysicsEngine::updateContactCache() {
    constexpr int maxFramesWithoutCollision = 10;
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

//-----------------------------
//       Sleep Thresholds
//-----------------------------
void PhysicsEngine::updateSleepThresholds(GameObject& obj) {
    if (obj.isStatic or obj.asleep) 
        return;

    obj.collisionHistory.push(obj.totalCollisionCount);
    obj.totalCollisionCount = 0;
    float avg = obj.collisionHistory.average();

    if (avg <= 0.0f) {
        if (std::abs(avg - obj.lastAvg) >= 1)
            obj.sleepCounter = 0.0f;
        obj.lastAvg = avg;

        return;
    }

    avg = std::max(avg, 1.0f);

    // reset sleep counter if difference is big enough
    if (std::abs(avg - obj.lastAvg) >= 1)
        obj.sleepCounter = 0.0f;
    obj.lastAvg = avg;

    float linearFactor = 0.1f; 
    float angularFactor = 0.2f;
    // if only in contact with one object, use a smaller factor
    if (avg > 0.0f and avg <= 1.0f) { angularFactor *= 0.5f; };

    // set thresholds
    obj.velocityThreshold = avg * linearFactor;
    obj.angularVelocityThreshold = (avg * angularFactor) * 1.5f * obj.invRadius;
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
    midPhase(tHits, dHits);         // reserve behöver definieras (inte 16).
    narrowPhase(tHits, dHits);      // SAT + collisionManifold
    collectActiveContacts();        // collect contacts to solve
    resolveCollisions();            // PGS + Baumgarte stabilization
}

//---------------------------------------------
//                Broadphase
//---------------------------------------------
void PhysicsEngine::broadPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits) {
    // ----- dynamic vs terrain -----
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

    // ----- dynamic vs dynamic -----
    static std::vector<std::pair<GameObject*, GameObject*>> hitsBufDynamic; // hits buffer
    hitsBufDynamic.clear();
    treeVsTreeQuery(dynamicAwakeBvh, dynamicAwakeBvh, hitsBufDynamic);                // query dynamic vs dynamic objects

    cap = static_cast<int>(hitsBufDynamic.size()); 
    dHits.resize(cap);         
    sp = 0; 

    for (auto& hp : hitsBufDynamic) { 
        GameObject* A = hp.first, * B = hp.second; 

        if (A == B) continue; 
        dHits[sp++] = DynamicHit{ A,B };
    }
    dHits.resize(sp);

    // ----- dynamic vs asleep -----
    hitsBufDynamic.clear();
    treeVsTreeQuery(dynamicAwakeBvh, dynamicAsleepBvh, hitsBufDynamic);

    cap = static_cast<int>(hitsBufDynamic.size());
    dHits.reserve(dHits.size() + cap);
    sp = 0;

    for (auto& hp : hitsBufDynamic) {
        GameObject* A = hp.first, * B = hp.second;

        if (A == B) continue;
        dHits.emplace_back(DynamicHit{ A,B });
    }

    // ----- dynamic vs static -----
    hitsBufDynamic.clear();
    treeVsTreeQuery(dynamicAwakeBvh, staticBvh, hitsBufDynamic);

    cap = static_cast<int>(hitsBufDynamic.size());
    dHits.reserve(dHits.size() + cap);
    sp = 0;

    for (auto& hp : hitsBufDynamic) {
        GameObject* A = hp.first, * B = hp.second;

        if (A == B) continue;
        dHits.emplace_back(DynamicHit{ A,B });
    }
}

//---------------------------------------------
//                 Midphase
//---------------------------------------------
void PhysicsEngine::midPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits) {
    // gameobject.collider == TriMesh: run BVH queries
    for (TerrainHit& th : tHits) {
        if (th.obj->colliderType != ColliderType::MESH)
            continue;

        Mesh* triMeshPtr = std::get_if<Mesh>(&th.obj->collider.shape);
        int cap = static_cast<int>(th.coarse.size());
        th.refined.resize(cap);

        int sp = 0;
        BVHTree<Tri>& bvh = triMeshPtr->bvh;

        for (Tri* tri : th.coarse) {
            std::vector<Tri*> hitsBuf;
            hitsBuf.reserve(16);

            bvh.singleQuery(tri->getAABB(), hitsBuf);
            th.refined[sp++] = { tri, std::move(hitsBuf) };
        }

        th.refined.resize(sp);
    }

    // gameobject.collider == TriMesh: run BVH queries 
    for (DynamicHit& dh : dHits) {
        if ((dh.A->colliderType != ColliderType::MESH) and (dh.B->colliderType != ColliderType::MESH))
            continue;

        Mesh* meshA = std::get_if<Mesh>(&dh.A->collider.shape);
        Mesh* meshB = std::get_if<Mesh>(&dh.B->collider.shape);

        if (meshA and meshB) {
            dh.doubleMeshTris.reserve(16);
            treeVsTreeQuery(meshA->bvh, meshB->bvh, dh.doubleMeshTris);
        }

        else if (meshA) {
            AABB& boxB = dh.B->collider.getAABB();
            dh.singleMeshTris.reserve(16);
            meshA->bvh.singleQuery(boxB, dh.singleMeshTris);
        }

        else {
            AABB& boxA = dh.A->collider.getAABB();
            dh.singleMeshTris.reserve(16);
            meshB->bvh.singleQuery(boxA, dh.singleMeshTris);
        }
    }
}
//---------------------------------------------
//               Narrow phase
//---------------------------------------------
void PhysicsEngine::narrowPhase(std::vector<TerrainHit>& terrainHits, std::vector<DynamicHit>& dynamicHits) 
{
    // ----- Terrain vs collider ----- 
    for (TerrainHit& th : terrainHits) 
    {
        if (th.obj->asleep) {
            continue;
        }

        static std::vector<SAT::Result> allResults;
        allResults.clear(); 
        int reserveSize = 0;
        if (th.refined.size() == 0) {
            reserveSize = th.coarse.size() * 2; // worst case: all tris collide with collider 
        }
        else {
            reserveSize = th.refined.size() * 2; 
        }
        allResults.reserve(reserveSize); // reserve space for results

        // MESH vs tris
        if (th.obj->colliderType == ColliderType::MESH) {
            // gå igenom refined
        }
        // BOX vs tris
        else if (th.obj->colliderType == ColliderType::CUBOID) 
        {
            for (Tri* tri : th.coarse) {
                SAT::Result satResult;
                if (!SAT::boxTri(th.obj->collider, *tri, satResult)) {
                    continue;
                }

                SAT::reverseNormal(th.obj->position, satResult.tri_ptr->centroid, satResult.normal);
                allResults.push_back(satResult);
            }

            if (allResults.size() == 0) {
                continue;
            }

            if (th.obj->helperMatricesDirty) th.obj->setHelperMatrices();  // update helper matrixes and aabb faces

            Contact contact(th.obj, nullptr);

            glm::vec3 avgNormal{ 0.0f }; 
            for (const SAT::Result& res : allResults) { 
                avgNormal += res.normal; 
            }
            avgNormal = glm::normalize(avgNormal); // average normal  
            contact.normal = avgNormal;

            SAT::findBestTriangles(allResults);

            collisionManifold->boxMesh(contact, contactCache, allResults);
        }
        // SPHERE vs tris
        else if (th.obj->colliderType == ColliderType::SPHERE) 
        {
            for (Tri* tri : th.coarse) { 
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

            if (th.obj->helperMatricesDirty) th.obj->setHelperMatrices();

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

        if (dh.singleMeshTris.size() > 0) {
            // collider vs tris
        }

        else if (dh.doubleMeshTris.size() > 0) {
            // tris vs tris
        }

        // collider vs collider
        else {
            // CUBE vs CUBE
            if (objA->colliderType == ColliderType::CUBOID and objB->colliderType == ColliderType::CUBOID)
            {
                SAT::Result satResult;
                if (!SAT::boxBox(objA->collider, objB->collider, satResult)) {
                    continue;
                }
                objA->totalCollisionCount++;
                objB->totalCollisionCount++;

                // used only for worldInertia (so far)
                if (objA->helperMatricesDirty) objA->setHelperMatrices();
                if (objB->helperMatricesDirty) objB->setHelperMatrices();

                Contact contact(objA, objB);

                PhysicsEngine::WakeUpInfo wakeInfo = wakeUpCheck(*objA, *objB);
                contact.freezeA = (objA->isStatic) || (objA->asleep && !wakeInfo.A);
                contact.freezeB = (objB->isStatic) || (objB->asleep && !wakeInfo.B);

                collisionManifold->boxBox(contact, contactCache, satResult);
            }
            // CUBE vs SPHERE
            else if ((objA->colliderType == ColliderType::CUBOID and objB->colliderType == ColliderType::SPHERE) or (objA->colliderType == ColliderType::SPHERE and objB->colliderType == ColliderType::CUBOID)) 
            {
                // swap if A is not cuboid
                if (objA->colliderType != ColliderType::CUBOID) {
                    std::swap(objA, objB);
                }

                // used in SAT
                if (objA->helperMatricesDirty) objA->setHelperMatrices();
                if (objB->helperMatricesDirty) objB->setHelperMatrices();

                SAT::Result satResult;
                if (!SAT::boxSphere(objA->collider, objB->collider, satResult)) {
                    continue;
                }

                objA->totalCollisionCount++; 
                objB->totalCollisionCount++; 

                Contact contact(objA, objB); 

                PhysicsEngine::WakeUpInfo wakeInfo = wakeUpCheck(*objA, *objB);
                contact.freezeA = (objA->isStatic) || (objA->asleep && !wakeInfo.A);
                contact.freezeB = (objB->isStatic) || (objB->asleep && !wakeInfo.B);

                collisionManifold->boxSphere(contact, contactCache, satResult); 
            }
            // SPHERE vs SPHERE
            else if (objA->colliderType == ColliderType::SPHERE and objB->colliderType == ColliderType::SPHERE) 
            {
                SAT::Result satResult;
                if (!SAT::sphereSphere(objA->collider, objB->collider, satResult)) { 
                    continue;
                }

                // used only for worldInertia
                if (objA->helperMatricesDirty) objA->setHelperMatrices();
                if (objB->helperMatricesDirty) objB->setHelperMatrices();

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
}

void PhysicsEngine::collectActiveContacts() {
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
}

//---------------------------------------------
//            Resolve collisions
//---------------------------------------------
void PhysicsEngine::resolveCollisions() {
    // Justera beroende på material
    constexpr float staticFriction = 0.8f;
    constexpr float dynamicFriction = 0.6f;
    constexpr float twistFriction = 0.2f;

    for (Contact* contact : contactsToSolve) { 
        GameObject& objA = *contact->objA_ptr; 
        GameObject& objB = *contact->objB_ptr; 

        // ----- Baumgarte stabilization -----
        float slop = 0.0005f; 
        float baumgarteFactor = 0.2f; 
        for (int j = 0; j < contact->points.size(); j++) { 
            ContactPoint& cp = contact->points[j]; 

            float penetration = cp.depth; 
            float allowed = penetration - slop; 

            if (allowed > 0.0f) 
                cp.biasVelocity = glm::min(-(baumgarteFactor * allowed) / this->dt, 0.0f); 
            else
                cp.biasVelocity = 0.0f; 
        }

        // ----- Warm start -----
        if (contact->freezeA or contact->freezeB) {
            for (ContactPoint& cp : contact->points) {
                cp.accumulatedImpulse = 0.f;
                cp.accumulatedFrictionImpulse1 = 0.f;
                cp.accumulatedFrictionImpulse2 = 0.f;
                cp.accumulatedTwistImpulse = 0.f;
            }
        } else {
            for (ContactPoint& cp : contact->points) {
                // total impuls från föregående steg 
                glm::vec3 Pn = cp.accumulatedImpulse * contact->normal;
                glm::vec3 Pt = cp.accumulatedFrictionImpulse1 * contact->t1 + cp.accumulatedFrictionImpulse2 * contact->t2;

                // linjär + tangentiell
                glm::vec3 J = Pn + Pt;

                if (contact->objA_ptr and !contact->freezeA) {
                    objA.applyForceLinear(-J * objA.invMass);
                    objA.applyForceAngular(-objA.inverseInertiaWorld * glm::cross(cp.rA, J));
                    // twist
                    objA.applyForceAngular(-objA.inverseInertiaWorld * (cp.accumulatedTwistImpulse * contact->normal));
                }
                if (contact->objB_ptr and !contact->freezeB) {
                    objB.applyForceLinear(J * objB.invMass);
                    objB.applyForceAngular(objB.inverseInertiaWorld * glm::cross(cp.rB, J));
                    // twist
                    objB.applyForceAngular(objB.inverseInertiaWorld * (cp.accumulatedTwistImpulse * contact->normal));
                }
            }
        }
    } 

    // ------ PGS solver ------
    int maxIterations = 8; 

    for (int i = 0; i < maxIterations; i++) {
        float maxDelta = 0.0f;

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

                // Baumgarte-bias inbyggd i impuls­beräkningen:
                float v_target = cp.targetBounceVelocity;
                float v_bias = cp.biasVelocity;
                // villkor: normalVelocity + v_bias = önskad hastighet vid kontakten
                float J = -(normalVelocity - v_target + v_bias) * cp.m_eff;

                // Clamp
                float temp = cp.accumulatedImpulse;
                cp.accumulatedImpulse = glm::max(temp + J, 0.0f);
                float deltaImpulse = cp.accumulatedImpulse - temp;

                // Add normal to impulse
                glm::vec3 deltaNormalImpulse = deltaImpulse * contact->normal;

                // Apply impulses
                float deltaNormalImpulseLen = glm::dot(deltaNormalImpulse, deltaNormalImpulse);
                if (deltaNormalImpulseLen > 1e-6f) {

                    if (contact->objA_ptr and !contact->freezeA) {
                        objA.applyForceLinear(-deltaNormalImpulse * objA.invMass);
                        objA.applyForceAngular(-objA.inverseInertiaWorld * glm::cross(cp.rA, deltaNormalImpulse));
                    }
                    if (contact->objB_ptr and !contact->freezeB) {
                        objB.applyForceLinear(deltaNormalImpulse * objB.invMass);
                        objB.applyForceAngular(objB.inverseInertiaWorld * glm::cross(cp.rB, deltaNormalImpulse));
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

                    // Applicera den statiska friktionsimpulsen:
                    if (contact->objA_ptr and !contact->freezeA) {
                        objA.applyForceLinear(-dFt * objA.invMass);
                        objA.applyForceAngular(-objA.inverseInertiaWorld * glm::cross(cp.rA, dFt));
                    }
                    if (contact->objB_ptr and !contact->freezeB) {
                        objB.applyForceLinear(dFt * objB.invMass);
                        objB.applyForceAngular(objB.inverseInertiaWorld * glm::cross(cp.rB, dFt));
                    }
                }

                // ---------- Dynamic friction ----------
                else {
                    // Dynamisk friktion: projektera TOTAL impuls till cirkeln μ_d * Jn
                    float maxDyn = dynamicFriction * Jn;

                    float len = std::sqrt(newLen2);
                    if (len > 1e-8f) {
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
                            objA.applyForceLinear(-dFt * objA.invMass);
                            objA.applyForceAngular(-objA.inverseInertiaWorld * glm::cross(cp.rA, dFt));
                        }
                        if (contact->objB_ptr and !contact->freezeB) {
                            objB.applyForceLinear(dFt * objB.invMass);
                            objB.applyForceAngular(objB.inverseInertiaWorld * glm::cross(cp.rB, dFt));
                        }
                    }
                }
                maxDelta = std::max(maxDelta, std::abs(dT));

                // ---------- Twist friction ----------
                // Beräkna den relativa rotationshastigheten kring kontaktnormalen
                angVelA = glm::vec3{ 0.0f };
                angVelB = glm::vec3{ 0.0f };
                if (contact->objA_ptr and !contact->freezeA) {
                    angVelA = objA.angularVelocity;
                }
                if (contact->objB_ptr and !contact->freezeB) {
                    angVelB = objB.angularVelocity;
                }
                float relativeAngularSpeed = glm::dot((angVelB - angVelA), contact->normal);

                // Beräkna preliminär twist impulse (i rotationsdomänen)
                float twistImpulse = -relativeAngularSpeed * cp.invMassTwist;
                float maxTwistImpulse = twistFriction * std::abs(cp.accumulatedImpulse);

                // Ackumulera twistimpulsen
                float oldTwistImpulse = cp.accumulatedTwistImpulse;
                float newTwistImpulse = glm::clamp(oldTwistImpulse + twistImpulse, -maxTwistImpulse, maxTwistImpulse);
                float deltaTwistImpulse = newTwistImpulse - oldTwistImpulse;
                cp.accumulatedTwistImpulse = newTwistImpulse;

                // Den twistimpuls vi applicerar är ett moment (angular impulse) kring kontaktnormalen
                glm::vec3 twistImpulseVec = deltaTwistImpulse * contact->normal;

                // Applicera twistimpulsen på kropparnas angulära hastigheter
                float twistImpulseVecLen = glm::dot(twistImpulseVec, twistImpulseVec);
                if (twistImpulseVecLen > 1e-6f) {

                    if (contact->objA_ptr and !contact->freezeA) {
                        objA.applyForceAngular(-objA.inverseInertiaWorld * twistImpulseVec);
                    }
                    if (contact->objB_ptr and !contact->freezeB) {
                        objB.applyForceAngular(objB.inverseInertiaWorld * twistImpulseVec);
                    }
                }
                maxDelta = std::max(maxDelta, std::abs(deltaTwistImpulse));
            }
        }

        if (maxDelta < 1e-4f) {
            break;
        }
    }
}