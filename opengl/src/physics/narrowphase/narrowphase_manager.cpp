#include "pch.h"
#include "narrowphase_manager.h"


//=======================================================
//              Initialization
//=======================================================
void NarrowphaseManager::init(
    CollisionManifold* collisionManifold,
    std::vector<DebugSpeculativeContact>* debugSpeculativeContacts,
    std::unordered_map<size_t, Contact>* contactCache,
    RuntimeCaches* caches,
    std::vector<RigidBodyHandle>* toWake)
{
    this->collisionManifold = collisionManifold;
    this->debugSpeculativeContacts = debugSpeculativeContacts;
    this->contactCache = contactCache;
    this->caches = caches;
    this->toWake = toWake;
}

void NarrowphaseManager::clear() {
    externalContacts.clear();
    terrainTriCandidates.clear();
    SAT_resultsList.clear();
}

//=======================================================
//               Narrow phase
//=======================================================
void NarrowphaseManager::narrowPhase(
    const PairBatch& pairs,
    ContactBatch& batch,
    float dt)
{
    externalContacts.clear();
    normalHitPairs.clear();
    pendingSpeculativeContacts.clear();

    // Normal terrain
    for (const TerrainPair& pair : pairs.terrainPairs) {
        processTerrainPairs(pair, batch, dt);
    }

    // Normal dynamic
    double start = glfwGetTime();
    for (const DynamicPair& pair : pairs.dynamicPairs) {
        processDynamicPairs(pair, batch, dt);
    }
    double end = glfwGetTime();
    double elapsed = end - start;
    /*std::cout << "NARROWPHASE DEBUG INFO" << std::endl;
    std::cout << pairs.dynamicPairs.size() << " dynamic pairs processed" << std::endl;
    std::cout << "processDynamicPairs took " << elapsed * 1000.0 << " ms" << std::endl;
    std::cout << "Per pair time: " << (elapsed * 1000.0) / pairs.dynamicPairs.size() << " ms" << std::endl;*/

    // Speculative terrain
    for (const SpeculativeTerrainPair& pair : pairs.speculativeTerrainPairs) {
        processSpeculativeTerrainPairs(pair, dt);
    }

    // Speculative dynamic
    start = glfwGetTime();
    for (const SpeculativeDynamicPair& pair : pairs.speculativeDynamicPairs) {
        processSpeculativeDynamicPairs(pair, dt);
    }

    flushPendingSpeculativeContacts(batch, dt);
    end = glfwGetTime();
    elapsed = end - start;
    /*std::cout << pairs.speculativeDynamicPairs.size() << " speculative dynamic pairs processed" << std::endl;
    std::cout << "processSpeculativeDynamicPairs took " << elapsed * 1000.0 << " ms" << std::endl;
    std::cout << "Per pair time: " << (elapsed * 1000.0) / pairs.speculativeDynamicPairs.size() << " ms" << std::endl;*/
}

//=======================================================
//     Normal Dynamic pair processing
//=======================================================
void NarrowphaseManager::processDynamicPairs(
    const DynamicPair& pair,
    ContactBatch& batch,
    float dt)
{
    RigidBody* bodyA = caches->bodies.get(pair.bodyA, FUNC_NAME);
    RigidBody* bodyB = caches->bodies.get(pair.bodyB, FUNC_NAME);

    if (!bodyA || !bodyB) return;

    // If both bodies are static or kinematic and not externally controlled, skip contact generation.
    if ((bodyA->motionControl != MotionControl::External && (bodyA->type == BodyType::Static || bodyA->type == BodyType::Kinematic)) && 
        (bodyB->motionControl != MotionControl::External && (bodyB->type == BodyType::Static || bodyB->type == BodyType::Kinematic))) {
        return;
    }

    for (const ColliderHandle& colAH : bodyA->colliderHandles) {
        for (const ColliderHandle& colBH : bodyB->colliderHandles) {
            Collider* colliderA = caches->colliders.get(colAH, FUNC_NAME);
            Collider* colliderB = caches->colliders.get(colBH, FUNC_NAME);

            if (!colliderA || !colliderB) continue;

            processColliderPairNormal(
                batch,
                ContactBuildInput{
                    pair.bodyA,
                    pair.bodyB,
                    colAH,
                    colBH,
                    bodyA,
                    bodyB,
                    colliderA,
                    colliderB
                }
            );
        }
    }
}

void NarrowphaseManager::processColliderPairNormal(
    ContactBatch& batch,
    ContactBuildInput in)
{
    DynamicContactCandidate candidate;
    bool hit = false;

    const bool boxBox =
        in.colliderA->type == ColliderType::CUBOID &&
        in.colliderB->type == ColliderType::CUBOID;

    const bool boxSphere =
        (in.colliderA->type == ColliderType::CUBOID && in.colliderB->type == ColliderType::SPHERE) ||
        (in.colliderA->type == ColliderType::SPHERE && in.colliderB->type == ColliderType::CUBOID);

    const bool sphereSphere =
        in.colliderA->type == ColliderType::SPHERE &&
        in.colliderB->type == ColliderType::SPHERE;

    if (boxBox) {
        hit = tryBoxBox(in, candidate);
    }
    else if (boxSphere) {
        hit = tryBoxSphere(in, candidate);
    }
    else if (sphereSphere) {
        hit = trySphereSphere(in, candidate);
    }

    if (!hit) {
        return;
    }

    PairKey key = makeColliderPairKey(
        in.colliderHandleA,
        in.colliderHandleB
    );

    normalHitPairs.insert(key);

    emitRigidContact(batch, in, candidate);
}

//=======================================================
//     Speculative Dynamic pair processing
//=======================================================
void NarrowphaseManager::processSpeculativeDynamicPairs(
    const SpeculativeDynamicPair& pair,
    float dt)
{
    RigidBody* bodyA = caches->bodies.get(pair.bodyA, FUNC_NAME);
    RigidBody* bodyB = caches->bodies.get(pair.bodyB, FUNC_NAME);

    if (!bodyA || !bodyB) return;

    for (const ColliderHandle& colAH : bodyA->colliderHandles) {
        for (const ColliderHandle& colBH : bodyB->colliderHandles) {
            Collider* colliderA = caches->colliders.get(colAH, FUNC_NAME);
            Collider* colliderB = caches->colliders.get(colBH, FUNC_NAME);

            if (!colliderA || !colliderB) continue;

            processColliderPairSpeculative(
                ContactBuildInput{
                    pair.bodyA,
                    pair.bodyB,
                    colAH,
                    colBH,
                    bodyA,
                    bodyB,
                    colliderA,
                    colliderB
                },
                dt,
                pair.sweepOwner
            );
        }
    }
}

void NarrowphaseManager::processColliderPairSpeculative(
    ContactBuildInput in,
    float dt,
    RigidBodyHandle sweepOwner)
{
    PairKey key = makeColliderPairKey(
        in.colliderHandleA,
        in.colliderHandleB
    );

    // If normal narrowphase already created a real contact,
    // do not also create speculative contact for same collider pair.
    if (normalHitPairs.find(key) != normalHitPairs.end()) {
        return;
    }

    DynamicContactCandidate candidate;
    bool hit = false;

    const bool boxBox =
        in.colliderA->type == ColliderType::CUBOID &&
        in.colliderB->type == ColliderType::CUBOID;

    const bool boxSphere =
        (in.colliderA->type == ColliderType::CUBOID && in.colliderB->type == ColliderType::SPHERE) ||
        (in.colliderA->type == ColliderType::SPHERE && in.colliderB->type == ColliderType::CUBOID);

    const bool sphereSphere =
        in.colliderA->type == ColliderType::SPHERE &&
        in.colliderB->type == ColliderType::SPHERE;

    if (boxBox) {
        hit = trySpeculativeBoxBox(in, candidate, dt);
    }
    else if (boxSphere) {
        hit = trySpeculativeBoxSphere(in, candidate, dt);
    }
    else if (sphereSphere) {
        hit = trySpeculativeSphereSphere(in, candidate, dt);
    }

    if (!hit) {
        return;
    }

    pendingSpeculativeContacts.push_back({
        in,
        candidate,
        sweepOwner
    });
}

//=======================================================
//     Normal Terrain pair processing
//=======================================================
void NarrowphaseManager::processTerrainPairs(const TerrainPair& terrainPair, ContactBatch& batch, float dt) {
    RigidBody* body = caches->bodies.get(terrainPair.body, FUNC_NAME);
    if (body->asleep || body->type == BodyType::Kinematic) return;

    // per collider in body, collect candidate tris and send to SAT + collision manifold generation
    for (const ColliderHandle& colH : body->colliderHandles) {
        Collider* collider = caches->colliders.get(colH, FUNC_NAME);

        const std::vector<Tri*>* candidates = &terrainPair.tris;

        // if compound, do mid phase AABB tests to filter terrain tris before SAT, otherwise just send all terrain tris to SAT
        if (body->isCompound()) {
            terrainTriCandidates.clear();
            terrainTriCandidates.reserve(terrainPair.tris.size());
            collectTerrainTriCandidates(collider, terrainPair.tris, terrainTriCandidates);

            if (terrainTriCandidates.empty()) {
                continue;
            }

            candidates = &terrainTriCandidates;
        }

        SAT_resultsList.clear();
        SAT_resultsList.reserve(candidates->size());

        switch (collider->type) {
        case ColliderType::CUBOID:
            processTerrainTriBox(batch, collider->rigidBodyHandle, collider, body, *candidates);
            break;

        case ColliderType::SPHERE:
            processTerrainTriSphere(batch, collider->rigidBodyHandle, collider, body, *candidates);
            break;
        }
    }
}

//=======================================================
//     Speculative Terrain pair processing
//=======================================================
void NarrowphaseManager::processSpeculativeTerrainPairs(
    const SpeculativeTerrainPair& pair,
    float dt)
{
    RigidBody* body = caches->bodies.get(pair.body, FUNC_NAME);

    if (!body || body->asleep || body->type == BodyType::Kinematic) {
        return;
    }

    for (const ColliderHandle& colH : body->colliderHandles) {
        Collider* collider = caches->colliders.get(colH, FUNC_NAME);

        const std::vector<Tri*>* candidates = &pair.tris;

        if (body->isCompound()) {
            terrainTriCandidates.clear();
            terrainTriCandidates.reserve(pair.tris.size());

            collectTerrainTriCandidates(collider, pair.tris, terrainTriCandidates);

            if (terrainTriCandidates.empty()) {
                continue;
            }

            candidates = &terrainTriCandidates;
        }

        if (collider->type != ColliderType::SPHERE) {
            continue; // börja bara med sphere-terrain speculative
        }

        for (Tri* tri : *candidates) {
            ContactBuildInput in{};
            in.bodyHandleA = pair.body;
            in.colliderHandleA = colH;
            in.bodyA = body;
            in.colliderA = collider;

            // B = terrain
            in.bodyB = nullptr;
            in.colliderB = nullptr;

            DynamicContactCandidate candidate{};
            candidate.partnerTypeA = ContactPartnerType::RigidBody;
            candidate.partnerTypeB = ContactPartnerType::Terrain;

            if (!trySpeculativeSphereTriangle(in, tri, candidate, dt)) {
                continue;
            }

            PendingSpeculativeContact pending{};
            pending.input = in;
            pending.candidate = candidate;
            pending.sweepOwner = pair.body;

            pendingSpeculativeContacts.push_back(pending);
        }
    }
}

//=======================================================
//     Collider and body handle packing for pair keys
//=======================================================
uint64_t NarrowphaseManager::packColliderHandle(ColliderHandle h) {
    return (uint64_t(h.slot) << 32) | uint64_t(h.gen);
}
uint64_t NarrowphaseManager::packBodyHandle(RigidBodyHandle h) {
    return (uint64_t(h.slot) << 32) | uint64_t(h.gen);
}
PairKey NarrowphaseManager::makeColliderPairKey(ColliderHandle a, ColliderHandle b) {
    uint64_t pa = packColliderHandle(a);
    uint64_t pb = packColliderHandle(b);

    if (pa > pb) {
        std::swap(pa, pb);
    }

    return { pa, pb };
}

//=======================================================
//     Runtime data generation
//=======================================================
ContactRuntime NarrowphaseManager::makeRuntimeData(
    RigidBody* bodyA, RigidBody* bodyB,
    Collider* colliderA, Collider* colliderB,
    Transform* bodyRootA, Transform* bodyRootB) const
{
    ContactRuntime rt;
    rt.bodyA = bodyA;
    rt.bodyB = bodyB;
    rt.colliderA = colliderA;
    rt.colliderB = colliderB;
    rt.bodyRootA = bodyRootA;
    rt.bodyRootB = bodyRootB;
    return rt;
}

ContactRuntime NarrowphaseManager::makeRuntimeData(
    RigidBody* bodyA,
    Collider* colliderA,
    Transform* bodyRootA) const
{
    ContactRuntime rt;
    rt.bodyA = bodyA;
    rt.colliderA = colliderA;
    rt.bodyRootA = bodyRootA;
    return rt;
}

//=======================================================
//     Terrain tri candidate collection
//=======================================================
void NarrowphaseManager::collectTerrainTriCandidates(
    Collider* collider,
    const std::vector<Tri*>& inputTris,
    std::vector<Tri*>& outCandidates)
{
    outCandidates.clear();

    const AABB& colAABB = collider->getAABB();
    for (Tri* tri : inputTris) {
        if (tri->getAABB().intersects(colAABB)) {
            outCandidates.push_back(tri);
        }
    }
}