#include "pch.h"
#include "narrowphase_manager.h"

#include "terrain_processing.h"

//=======================================================
//              Initialization
//=======================================================
void NarrowphaseManager::init(
    CollisionManifold* collisionManifold,
    std::unordered_map<size_t, Contact>* contactCache,
    RuntimeCaches* caches,
    std::vector<RigidBodyHandle>* toWake)
{
    this->collisionManifold = collisionManifold;
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

    for (const TerrainPair& pair : pairs.terrainPairs) {
        processTerrainPair(pair, batch, dt, NarrowphasePass::Normal);
    }

    for (const DynamicPair& pair : pairs.dynamicPairs) {
        processDynamicPair(pair, batch, dt, NarrowphasePass::Normal);
    }

    for (const TerrainPair& pair : pairs.speculativeTerrainPairs) {
        processTerrainPair(pair, batch, dt, NarrowphasePass::Speculative);
    }

    for (const DynamicPair& pair : pairs.speculativeDynamicPairs) {
        processDynamicPair(pair, batch, dt, NarrowphasePass::Speculative);
    }
}

//=======================================================
//     Dynamic pair processing
//=======================================================
void NarrowphaseManager::processDynamicPair(
    const DynamicPair& pair,
    ContactBatch& batch,
    float dt,
    NarrowphasePass pass)
{
    RigidBody* bodyA = caches->bodies.get(pair.bodyA, FUNC_NAME);
    RigidBody* bodyB = caches->bodies.get(pair.bodyB, FUNC_NAME);

    if (!bodyA || !bodyB) return;

    if ((bodyA->type == BodyType::Static || bodyA->type == BodyType::Kinematic) &&
        (bodyB->type == BodyType::Static || bodyB->type == BodyType::Kinematic)) {
        return;
    }

    for (const ColliderHandle& colAH : bodyA->colliderHandles) {
        for (const ColliderHandle& colBH : bodyB->colliderHandles) {
            Collider* colliderA = caches->colliders.get(colAH, FUNC_NAME);
            Collider* colliderB = caches->colliders.get(colBH, FUNC_NAME);

            if (!colliderA || !colliderB) continue;

            processColliderPair(
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
                },
                dt,
                pass
            );
        }
    }
}

void NarrowphaseManager::processColliderPair(
    ContactBatch& batch,
    ContactBuildInput in,
    float dt,
    NarrowphasePass pass)
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

    if (pass == NarrowphasePass::Normal) {
        if (boxBox) { hit = tryBoxBox(in, candidate); }
        else if (boxSphere) { hit = tryBoxSphere(in, candidate); }
        else if (sphereSphere) { hit = trySphereSphere(in, candidate); }
    }
    else {
        if (boxBox) { hit = trySpeculativeBoxBox(in, candidate, dt); }
        else if (boxSphere) { hit = trySpeculativeBoxSphere(in, candidate, dt); }
        else if (sphereSphere) { hit = trySpeculativeSphereSphere(in, candidate, dt); }
    }

    if (!hit) {
        return;
    }

    emitRigidContact(batch, in, candidate);
}

//=======================================================
//     Terrain pair processing
//=======================================================
void NarrowphaseManager::processTerrainPair(const TerrainPair& terrainPair, ContactBatch& batch, float dt, NarrowphasePass pass) {
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
//      Helper functions
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

// Collects terrain tris whose AABBs intersect with the collider's AABB as candidates for SAT testing
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