#include "physics.h"

// ----- init -----
void PhysicsEngine::init(EngineState* engineState) {
    this->engineState = engineState;
    this->collisionManifold = new CollisionManifold();
}

// ----- setup scene -----
void PhysicsEngine::setupScene(std::vector<GameObject>* gameObjects, std::vector<Tri>* terrainTris) {
    this->dynamicObjects = gameObjects;
    dynamicBvh.build(*gameObjects); 

    this->terrainTriangles = terrainTris;
    terrainBvh.build(*terrainTris); 
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

// ----- Time step -----
void PhysicsEngine::step(float deltaTime, std::mt19937 rng) {
    this->dt = deltaTime;

    updatePositions();

    detectAndSolveCollisions();

    for (GameObject& obj : *dynamicObjects)
        updateSleepThresholds(obj);

    updateContactCache();
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

void PhysicsEngine::updatePositions() {
    for (GameObject& obj : *dynamicObjects) {
        if (!obj.isStatic and !obj.asleep) {
            obj.updatePos(this->dt);
            obj.updateAABB();
            obj.updateCollider();

            obj.aabb.facesShouldUpdate = true;
            obj.helperMatrixesShouldUpdate = true;

            obj.linearVelocityLen = glm::dot(obj.linearVelocity, obj.linearVelocity);
            obj.angularVelocityLen = glm::dot(obj.angularVelocity, obj.angularVelocity);
        }
    }
}

void PhysicsEngine::updateHelpers(GameObject& obj) {
    if (obj.helperMatrixesShouldUpdate) obj.setHelperMatrixes();
    if (obj.aabb.facesShouldUpdate) obj.aabb.updateFaces(obj.modelMatrix);
}

bool PhysicsEngine::updateSleep(GameObject& A, GameObject& B) {
    constexpr float velocityThreshold = 1.0f;
    constexpr float angularVelocityThreshold = 1.4f;
    constexpr float velocityThreshold2 = velocityThreshold * velocityThreshold;
    constexpr float angularVelocityThreshold2 = angularVelocityThreshold * angularVelocityThreshold;

    // ------ set awake ------
    if (A.asleep) {
        if (B.linearVelocityLen > velocityThreshold2 or B.angularVelocityLen > angularVelocityThreshold2) {
            A.setAwake();
        }
    }
    if (B.asleep) {
        if (A.linearVelocityLen > velocityThreshold2 or A.angularVelocityLen > angularVelocityThreshold2) {
            B.setAwake();
        }
    }
    // ------ sleep check ------
    if ((A.asleep and (B.asleep or B.isStatic)) or (B.asleep and (A.asleep or A.isStatic))) {
        A.setAsleep();
        B.setAsleep();

        return true;
    }

    return false;
}

void PhysicsEngine::detectAndSolveCollisions() {
    static std::vector<TerrainHit> tHits; 
    static std::vector<DynamicHit> dHits; 

    tHits.reserve(BVHTree<Tri>::MaxCollisionBuf); 
    dHits.reserve(BVHTree<GameObject>::MaxCollisionBuf); 

    broadPhase(tHits, dHits);       // hashmap kanske långsam? 
    midPhase(tHits, dHits);         // reserve behöver definieras (inte 16).
    narrowPhase(tHits, dHits);      // SAT + collisionManifold
}

void PhysicsEngine::broadPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits) {
    // ----- dynamic vs terrain -----
    std::vector<std::pair<GameObject*, Tri*>> hitsBufTerrain;  // hits buffer
    hitsBufTerrain.reserve(BVHTree<Tri>::MaxCollisionBuf);
    treeVsTreeQuery(dynamicBvh, terrainBvh, hitsBufTerrain);          // query terrain vs dynamic objects

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

void PhysicsEngine::narrowPhase(std::vector<TerrainHit>& terrainHits, std::vector<DynamicHit>& dynamicHits) {
    for (TerrainHit& th : terrainHits) 
    {
        if (th.obj->colliderType == ColliderType::MESH) {
            // gå igenom refined
        }
        else if (th.obj->colliderType == ColliderType::CUBOID) {
            if (th.obj->asleep)
                continue;

            static std::vector<SAT::Result> allResults;
            allResults.clear();
            allResults.reserve(th.coarse.size() * 2); // worst case: all tris collide with collider

            for (Tri* tri : th.coarse) {
                SAT::Result satResult;
                if (!SAT::triVsCuboid(th.obj->collider, *tri, satResult))
                    continue;

                allResults.push_back(satResult);
            }

            if (allResults.size() == 0)
                continue; 

            SAT::findBestTriangles(allResults);

            updateHelpers(*th.obj); // update helper matrixes and aabb faces

            SAT::reverseNormal(th.obj->position, allResults[0].tri_ptr->centroid, allResults[0].normal);

            Contact contact(th.obj, nullptr);
            contact.normal = allResults[0].normal; // use deepest penetration (sorted list)

            collisionManifold->meshVsCuboid(contact, contactCache, allResults);

            resolveCollision(contact); // resolve collision
        }
    }

    for (DynamicHit& dh : dynamicHits) 
    {
        if (dh.singleMeshTris.size() > 0) {
            // collider vs tris
        }

        else if (dh.doubleMeshTris.size() > 0) {
            // tris vs tris
        }

        else {
            // collider vs collider
            GameObject& objA = *dh.A; 
            GameObject& objB = *dh.B;

            if (objA.isStatic and objB.isStatic)
                continue;

            if (updateSleep(objA, objB))
                continue;

            SAT::Result satResult; 
            if (!SAT::cuboidVsCuboid(objA.collider, objB.collider, satResult))
                continue;

            SAT::reverseNormal(objA.position, objB.position, satResult.normal); 

            objA.totalCollisionCount++; 
            objB.totalCollisionCount++; 

            updateHelpers(objA); // update helper matrixes and aabb faces 
            updateHelpers(objB);

            Contact contact(&objA, &objB); 
            collisionManifold->cuboidVsCuboid(contact, contactCache, satResult); 

            resolveCollision(contact); // resolve collision 
        }
    }
}

void PhysicsEngine::resolveCollision(Contact& contact) {
    // Justera beroende på material
    constexpr float staticFriction = 0.8f;
    constexpr float dynamicFriction = 0.5f;
    constexpr float twistFriction = 0.2f;

    GameObject& objA = *contact.objA_ptr;
    GameObject& objB = *contact.objB_ptr;

    // ----- Baumgarte stabilization -----
    float slop = 0.001f;
    float baumgarteFactor = 0.2f;
    for (int j = 0; j < contact.points.size(); j++) {
        ContactPoint& cp = contact.points[j];

        float penetration = cp.depth;
        float allowed = penetration - slop;

        if (allowed > 0.0f)
            cp.biasVelocity = glm::min(-(baumgarteFactor * allowed) / this->dt, 0.0f);
        else
            cp.biasVelocity = 0.0f;
    }

    // ------ PGS solver ------
    int maxIterations = 8;
    for (int i = 0; i < maxIterations; i++) {
        bool converged = true;

        for (int j = 0; j < contact.points.size(); j++) {
            ContactPoint& cp = contact.points[j];

            glm::vec3 relVelA{ 0.0f };
            glm::vec3 relVelB{ 0.0f };
            glm::vec3 angVelA{ 0.0f };
            glm::vec3 angVelB{ 0.0f };
            if (&objA != nullptr) {
                relVelA = objA.linearVelocity;
                angVelA = objA.angularVelocity;
            }
            if (&objB != nullptr) {
                relVelB = objB.linearVelocity;
                angVelB = objB.angularVelocity;
            }

            glm::vec3 relativeVelocity =
                (relVelB + glm::cross(angVelB, cp.rB)) -
                (relVelA + glm::cross(angVelA, cp.rA)); 

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

                if (&objA != nullptr) {
                    objA.applyForceLinear(-deltaNormalImpulse * objA.invMass);
                    objA.applyForceAngular(-objA.inverseInertia * glm::cross(cp.rA, deltaNormalImpulse));
                }
                if (&objB != nullptr) {
                    objB.applyForceLinear(deltaNormalImpulse * objB.invMass);
                    objB.applyForceAngular(objB.inverseInertia * glm::cross(cp.rB, deltaNormalImpulse));
                }
                // impulse was changed
                converged = false;
            }

            //--------------- Pre-calculate for friction ----------------
            relVelA = glm::vec3{ 0.0f };
            relVelB = glm::vec3{ 0.0f };
            angVelA = glm::vec3{ 0.0f };
            angVelB = glm::vec3{ 0.0f };
            if (&objA != nullptr) {
                relVelA = objA.linearVelocity;
                angVelA = objA.angularVelocity;
            }
            if (&objB != nullptr) {
                relVelB = objB.linearVelocity;
                angVelB = objB.angularVelocity;
            }

            relativeVelocity =
                (relVelB + glm::cross(angVelB, cp.rB)) -
                (relVelA + glm::cross(angVelA, cp.rA));

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
                if (&objA != nullptr) {
                    objA.applyForceLinear(-desiredFrictionImpulse * objA.invMass);
                    objA.applyForceAngular(-objA.inverseInertia * glm::cross(cp.rA, desiredFrictionImpulse));
                }
                if (&objB != nullptr) {
                    objB.applyForceLinear(desiredFrictionImpulse * objB.invMass); 
                    objB.applyForceAngular(objB.inverseInertia * glm::cross(cp.rB, desiredFrictionImpulse)); 
                }
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

                    if (&objA != nullptr) { 
                        objA.applyForceLinear(-frictionImpulse * objA.invMass); 
                        objA.applyForceAngular(-objA.inverseInertia * glm::cross(cp.rA, frictionImpulse)); 
                    }
                    if (&objB != nullptr) { 
                        objB.applyForceLinear(frictionImpulse * objB.invMass); 
                        objB.applyForceAngular(objB.inverseInertia * glm::cross(cp.rB, frictionImpulse)); 
                    } 
                    converged = false;
                }
            }
            // ---------- Twist friction ----------
            // Beräkna den relativa rotationshastigheten kring kontaktnormalen
            angVelA = glm::vec3{ 0.0f };
            angVelB = glm::vec3{ 0.0f };
            if (&objA != nullptr) {
                angVelA = objA.angularVelocity;
            }
            if (&objB != nullptr) {
                angVelB = objB.angularVelocity;
            }
            float relativeAngularSpeed = glm::dot((angVelB - angVelA), contact.normal);

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

                if (&objA != nullptr) {
                    objA.applyForceAngular(-objA.inverseInertia * twistImpulseVec);
                }
                if (&objB != nullptr) { 
                    objB.applyForceAngular(objB.inverseInertia* twistImpulseVec); 
                }

                converged = false;
            }
        }
        // Om vi inte har någon impuls att applicera, är vi klara
        if (converged)
            break;
    }
}