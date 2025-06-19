#include "physics.h"

// ----- init -----
void PhysicsEngine::init(EngineState* engineState) {
    this->engineState = engineState;
    this->collisionManifold = new CollisionManifold();
}

// ----- setup scene -----
void PhysicsEngine::setupScene(std::vector<GameObject>* gameObjects, std::vector<Tri>* terrainTriangles) {
    this->dynamicObjects = gameObjects;
    dynamicBvh.build(*gameObjects); 

    this->terrainTriangles = terrainTriangles;
    terrainBvh.build(*terrainTriangles); 
}

// ----- clear -----
void PhysicsEngine::clearPhysicsData() {
    contactCache.clear();
}

// ----- Getters -----
BVHTree<GameObject>& PhysicsEngine::getDynamicBvh() {
    return dynamicBvh;
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

// ----- raycast -----
RaycastHit PhysicsEngine::performRaycast(Ray& r) {
    RaycastHit hitData = raycast(r, this->dynamicObjects, this->dynamicBvh);
    return hitData;
}

// ----- Update methods -----
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
    float angularFactor = 0.1f;
    // if only in contact with one object, use a smaller factor
    if (avg > 0.0f and avg <= 1.0f) { angularFactor = 0.01f; };

    // set thresholds
    obj.velocityThreshold = avg * linearFactor;
    obj.angularVelocityThreshold = (avg * angularFactor) * 1.5f * obj.invRadius;
}

void PhysicsEngine::updatePositions(float deltaTime) {
    for (GameObject& obj : *dynamicObjects) {
        if (!obj.isStatic and !obj.asleep) {
            obj.updatePos(deltaTime);
            obj.updateAABB();
            obj.updateCollider();

            obj.aabb.facesHasUpdated = false;
            obj.helperMatrixesHasUpdated = false;

            obj.linearVelocityLen = glm::dot(obj.linearVelocity, obj.linearVelocity);
            obj.angularVelocityLen = glm::dot(obj.angularVelocity, obj.angularVelocity);
        }
    }
}

void PhysicsEngine::detectCollisions(std::vector<DynamicHit>& dHits) {
    static std::vector<TerrainHit> tHits; 
    // static std::vector<DynamicHit>& dynamicHits;    

    tHits.reserve(BVHTree<Tri>::MaxCollisionBuf); 
    dHits.reserve(BVHTree<GameObject>::MaxCollisionBuf); 

    broadPhase(tHits, dHits);       // hashmap kanske långsam? 
    midPhase(tHits, dHits);         // reserve behöver definieras (inte 16).
    narrowPhase(tHits, dHits);      // SAT + collisionManifold
}

void PhysicsEngine::broadPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits) {
    // ----- dynamic vs terrain -----
    static std::vector<std::pair<GameObject*, Tri*>> hitsBufTerrain;  // hits buffer
    hitsBufTerrain.clear();
    treeVsTreeQuery(dynamicBvh, terrainBvh, hitsBufTerrain);          // query terrain vs dynamic objects
    
    // Sort terrain hits by GameObject to avoid duplicates 
    static std::unordered_map<GameObject*, std::vector<Tri*>> temp; 
    temp.clear();
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
    treeVsTreeQuery(dynamicBvh, dynamicBvh, hitsBufDynamic);                // query dynamic vs dynamic objects

    dHits.clear();
    cap = static_cast<int>(hitsBufDynamic.size()); 
    dHits.resize(cap);         
    sp = 0; 

    for (auto& hp : hitsBufDynamic) { 
        GameObject* A = hp.first, * B = hp.second; 

        if ((A->isStatic and B->isStatic) or A == B) 
            continue; 

        dHits[sp++] = DynamicHit{ A,B };
    }
    dHits.resize(sp);
}

void PhysicsEngine::midPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits) {
    // gameobject.collider == TriMesh: run BVH queries
    for (TerrainHit& th : tHits) {
        if (th.obj->colliderType != ColliderType::MESH)
            continue;

        TriMesh* triMeshPtr = std::get_if<TriMesh>(&th.obj->collider.shape);
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

        TriMesh* meshA = std::get_if<TriMesh>(&dh.A->collider.shape);
        TriMesh* meshB = std::get_if<TriMesh>(&dh.B->collider.shape);

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

void PhysicsEngine::narrowPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits) {
    for (TerrainHit& th : tHits) 
    {
        if (th.obj->colliderType == ColliderType::MESH) {
            // gå igenom refined
        }
        else {
            // gå igenom coarse
        }
            
    }

    for (DynamicHit& dh : dHits) 
    {
        if (dh.singleMeshTris.size() > 0) {
            // collider vs tris
        }

        else if (dh.doubleMeshTris.size() > 0) {
            // tris vs tris
        }

        else {
            // collider vs collider
        }

    }
}

void PhysicsEngine::resolveCollisions() {

}

// ----- Time step -----
void PhysicsEngine::step(float deltaTime, std::mt19937 rng) {
    updatePositions(deltaTime);

    constexpr float velocityThreshold = 1.0f;
    constexpr float angularVelocityThreshold = 1.4f;
    constexpr float velocityThreshold2 = velocityThreshold * velocityThreshold;
    constexpr float angularVelocityThreshold2 = angularVelocityThreshold * angularVelocityThreshold;

    // Justera beroende på material
    constexpr float staticFriction = 0.8f;
    constexpr float dynamicFriction = 0.5f;
    constexpr float twistFriction = 0.2f;

    static std::vector<DynamicHit> dynamicHits;
    dynamicHits.reserve(BVHTree<GameObject>::MaxCollisionBuf);
    dynamicHits.clear();

    detectCollisions(dynamicHits);

    for (int i = 0; i < dynamicHits.size(); i++) {
        GameObject& objA = *dynamicHits[i].A;
        GameObject& objB = *dynamicHits[i].B;

        //if (objB.id <= objA.id) continue;
        if (objA.isStatic and objB.isStatic) 
            continue;

        // ------ set awake ------
        if (objA.asleep) {
            if (objB.linearVelocityLen > velocityThreshold2 or objB.angularVelocityLen > angularVelocityThreshold2) {
                objA.setAwake();
            }
        }
        if (objB.asleep) {
            if (objA.linearVelocityLen > velocityThreshold2 or objA.angularVelocityLen > angularVelocityThreshold2) {
                objB.setAwake();
            }
        }
        // ------ sleep check ------
        if ((objA.asleep and (objB.asleep or objB.isStatic)) or (objB.asleep and (objA.asleep or objA.isStatic))) { 
            objA.setAsleep();
            objB.setAsleep();
            continue;
        }

        // ------ Narrow phase -------
        glm::vec3 cNormal;
        float depth = std::numeric_limits<float>::max();
        int cNormalOwner = 0;
        if (!SATQuery(objA, objB, cNormal, depth, cNormalOwner))
            continue;

        objA.totalCollisionCount++;
        objB.totalCollisionCount++;

        glm::vec3 direction = objB.position - objA.position;
        if (glm::dot(direction, cNormal) < 0)
            cNormal = -cNormal;

        // ------- contactPoints -------
        if (!objA.aabb.facesHasUpdated) objA.aabb.updateFaces(objA.modelMatrix);
        if (!objB.aabb.facesHasUpdated) objB.aabb.updateFaces(objB.modelMatrix);

        if (!objA.helperMatrixesHasUpdated) objA.setHelperMatrixes();
        if (!objB.helperMatrixesHasUpdated) objB.setHelperMatrixes();

        Contact contact;
        collisionManifold->createContact(contact, contactCache, objA, objB, cNormal, cNormalOwner);

        // ----- Baumgarte stabilization -----
        float slop = 0.001f;
        float baumgarteFactor = 0.2f;
        for (int j = 0; j < contact.counter; j++) {
            ContactPoint& cp = contact.points[j];

            float penetration = cp.depth;             
            float allowed = penetration - slop;

            if (allowed > 0.0f) 
                cp.biasVelocity = glm::min(-(baumgarteFactor * allowed) / deltaTime, 0.0f); 
            else 
                cp.biasVelocity = 0.0f;
        }

        // ------ PGS solver ------
        int maxIterations = 8;
        for (int i = 0; i < maxIterations; i++) {
            bool converged = true;

            for (int j = 0; j < contact.counter; j++) {
                ContactPoint& cp = contact.points[j];

                glm::vec3 relativeVelocity = 
                    (objB.linearVelocity + glm::cross(objB.angularVelocity, cp.rB)) -
                    (objA.linearVelocity + glm::cross(objA.angularVelocity, cp.rA));

                float normalVelocity = glm::dot(relativeVelocity, contact.normal);

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
                glm::vec3 deltaNormalImpulse = deltaImpulse * contact.normal;

                // Apply impulses
                float deltaNormalImpulseLen = glm::dot(deltaNormalImpulse, deltaNormalImpulse);
                if (deltaNormalImpulseLen > 1e-6f) {
                    objA.linearVelocity -= deltaNormalImpulse * objA.invMass;
                    objA.angularVelocity -= objA.inverseInertia * glm::cross(cp.rA, deltaNormalImpulse);

                    objB.linearVelocity += deltaNormalImpulse * objB.invMass;
                    objB.angularVelocity += objB.inverseInertia * glm::cross(cp.rB, deltaNormalImpulse);

                    // impulse was changed
                    converged = false;
                }

                //--------------- Pre-calculate for friction ----------------
                relativeVelocity = 
                    (objB.linearVelocity + cross(objB.angularVelocity, cp.rB)) - 
                    (objA.linearVelocity + cross(objA.angularVelocity, cp.rA));
                // Project tangential velocity to the tangent plane
                float v_t1 = glm::dot(relativeVelocity, cp.t1);
                float v_t2 = glm::dot(relativeVelocity, cp.t2);

                // Beräkna preliminära impulser i varje riktning:
                float J1 = -v_t1 * cp.invMassT1;
                float J2 = -v_t2 * cp.invMassT2;

                // ---------- Static friction ----------
                glm::vec3 desiredFrictionImpulse = (J1 * cp.t1) + (J2 * cp.t2);
                float JtLen2 = glm::dot(desiredFrictionImpulse, desiredFrictionImpulse);
                float Jn = std::abs(cp.accumulatedImpulse);
                // Beräkna maximal statisk friktionsimpuls (mu_static * |accumulatedImpulse|)
                float Jmax = staticFriction * Jn;
                float Jmax2 = Jmax * Jmax; // max2 = mu_static^2 * |accumulatedImpulse|^2

                if (JtLen2 <= Jmax2) {
                    cp.accumulatedFrictionImpulse1 += J1;
                    cp.accumulatedFrictionImpulse2 += J2;

                    // Applicera den statiska friktionsimpulsen:
                    objA.linearVelocity -= desiredFrictionImpulse * objA.invMass;
                    objA.angularVelocity -= objA.inverseInertia * glm::cross(cp.rA, desiredFrictionImpulse);

                    objB.linearVelocity += desiredFrictionImpulse * objB.invMass;
                    objB.angularVelocity += objB.inverseInertia * glm::cross(cp.rB, desiredFrictionImpulse);

                    if (JtLen2 > 1e-6f)
                        converged = false;
                }

                // ---------- Dynamic friction ----------
                else {
                    // Uppdatera de ackumulerade friktionsimpulserna (använd samma klampningsmetod som du gör idag)
                    float newFrictionImpulse1 = cp.accumulatedFrictionImpulse1 + J1;
                    float newFrictionImpulse2 = cp.accumulatedFrictionImpulse2 + J2;

                    // Klampar impulsen så att total friktion inte överskrider mu_d * |accumulatedImpulse|
                    float maxFriction = dynamicFriction * std::abs(cp.accumulatedImpulse);
                    float lengthSq = newFrictionImpulse1 * newFrictionImpulse1 + newFrictionImpulse2 * newFrictionImpulse2;
                    float maxFrictionSq = maxFriction * maxFriction;
                    if (lengthSq > maxFrictionSq) {
                        float scale = maxFriction / std::sqrt(lengthSq);
                        newFrictionImpulse1 *= scale;
                        newFrictionImpulse2 *= scale;
                    }

                    float deltaFrictionImpulse1 = newFrictionImpulse1 - cp.accumulatedFrictionImpulse1;
                    float deltaFrictionImpulse2 = newFrictionImpulse2 - cp.accumulatedFrictionImpulse2;
                    cp.accumulatedFrictionImpulse1 = newFrictionImpulse1;
                    cp.accumulatedFrictionImpulse2 = newFrictionImpulse2;

                    // Bygg friktionsimpulsen i tangentplanet:
                    glm::vec3 frictionImpulse = (deltaFrictionImpulse1 * cp.t1) + (deltaFrictionImpulse2 * cp.t2);

                    // Applicera impulsen:
                    float frictionImpulseLen = glm::dot(frictionImpulse, frictionImpulse);
                    if (frictionImpulseLen > 1e-6f) {
                        objA.linearVelocity -= frictionImpulse * objA.invMass;
                        objA.angularVelocity -= objA.inverseInertia * glm::cross(cp.rA, frictionImpulse);

                        objB.linearVelocity += frictionImpulse * objB.invMass;
                        objB.angularVelocity += objB.inverseInertia * glm::cross(cp.rB, frictionImpulse);

                        converged = false;
                    }
                }
                // ---------- Twist friction ----------
                // Beräkna den relativa rotationshastigheten kring kontaktnormalen
                float relativeAngularSpeed = glm::dot((objB.angularVelocity - objA.angularVelocity), contact.normal);

                // Beräkna preliminär twist impulse (i rotationsdomänen)
                float twistImpulse = -relativeAngularSpeed * cp.invMassTwist;
                float maxTwistImpulse = twistFriction * std::abs(cp.accumulatedImpulse);

                // Ackumulera twistimpulsen
                float oldTwistImpulse = cp.accumulatedTwistImpulse;
                float newTwistImpulse = glm::clamp(oldTwistImpulse + twistImpulse, -maxTwistImpulse, maxTwistImpulse);
                float deltaTwistImpulse = newTwistImpulse - oldTwistImpulse;
                cp.accumulatedTwistImpulse = newTwistImpulse;

                // Den twistimpuls vi applicerar är ett moment (angular impulse) kring kontaktnormalen
                glm::vec3 twistImpulseVec = deltaTwistImpulse * contact.normal;

                // Applicera twistimpulsen på kropparnas angulära hastigheter
                float twistImpulseVecLen = glm::dot(twistImpulseVec, twistImpulseVec);
                if (twistImpulseVecLen > 1e-6f) {
                    objA.angularVelocity -= objA.inverseInertia * twistImpulseVec;
                    objB.angularVelocity += objB.inverseInertia * twistImpulseVec;
                    converged = false;
                }
            }
            // Om vi inte har någon impuls att applicera, är vi klara
            if (converged)
                break;
        }
    }
    
    for (GameObject& obj : *dynamicObjects)
        updateSleepThresholds(obj);

    updateContactCache();
}