#include "pch.h"
#include "narrowphase_manager.h"

namespace physics::internal {

namespace {

std::optional<ShapePairKind> classifyShapePair(
    const Collider& colliderA,
    const Collider& colliderB)
{
    const bool boxBox =
        colliderA.type == ColliderType::CUBOID &&
        colliderB.type == ColliderType::CUBOID;

    if (boxBox) {
        return ShapePairKind::BoxBox;
    }

    const bool boxSphere =
        (colliderA.type == ColliderType::CUBOID &&
            colliderB.type == ColliderType::SPHERE) ||
        (colliderA.type == ColliderType::SPHERE &&
            colliderB.type == ColliderType::CUBOID);

    if (boxSphere) {
        return ShapePairKind::BoxSphere;
    }

    const bool sphereSphere =
        colliderA.type == ColliderType::SPHERE &&
        colliderB.type == ColliderType::SPHERE;

    if (sphereSphere) {
        return ShapePairKind::SphereSphere;
    }

    return std::nullopt;
}

}


//=======================================================
//              Initialization
//=======================================================
void NarrowphaseManager::init(
    PhysicsWorld* physicsWorld,
    CollisionManifold* collisionManifold,
    std::vector<DebugSpeculativeContact>* debugSpeculativeContacts,
    std::unordered_map<size_t, Contact>* contactCache,
    std::vector<BodyHandle>* toWake)
{
    this->physicsWorld = physicsWorld;
    this->collisionManifold = collisionManifold;
    this->debugSpeculativeContacts = debugSpeculativeContacts;
    this->contactCache = contactCache;
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
    pendingSweepHits.clear();

    // Normal terrain
    for (const TerrainPair& pair : pairs.terrainPairs) {
        processTerrainPairs(pair, batch, dt);
    }

    // Normal dynamic
    double start = glfwGetTime();
    for (const DynamicPair& pair : pairs.dynamicPairs) {
        processDynamicPairs(pair, batch, dt);
    }

    // Speculative terrain
    for (const SpeculativeTerrainPair& pair : pairs.speculativeTerrainPairs) {
        processSpeculativeTerrainPairs(pair, dt);
    }

    // Speculative dynamic
    start = glfwGetTime();
    for (const SpeculativeDynamicPair& pair : pairs.speculativeDynamicPairs) {
        processSpeculativeDynamicPairs(pair, dt);
    }

    flushPendingSweepHits(batch);
}

//=======================================================
//     Normal Dynamic pair processing
//=======================================================
void NarrowphaseManager::processDynamicPairs(
    const DynamicPair& pair,
    ContactBatch& batch,
    float dt)
{
    RigidBody& bodyA = physicsWorld->getBody(pair.bodyA);
    RigidBody& bodyB = physicsWorld->getBody(pair.bodyB);

    // If both bodies are static or kinematic and not externally controlled, skip contact generation
    if ((bodyA.motionControl != MotionControl::External && (bodyA.type == BodyType::Static || bodyA.type == BodyType::Kinematic)) && 
        (bodyB.motionControl != MotionControl::External && (bodyB.type == BodyType::Static || bodyB.type == BodyType::Kinematic))) {
        return;
    }

    for (const ColliderHandle& colAH : bodyA.colliderHandles) {
        for (const ColliderHandle& colBH : bodyB.colliderHandles) {
            Collider& colliderA = physicsWorld->getCollider(colAH);
            Collider& colliderB = physicsWorld->getCollider(colBH);

            // If either body is compound, check if the colliders' AABBs intersect
            if (bodyA.isCompound() || bodyB.isCompound()) {
                if (!colliderA.aabb.intersects(colliderB.aabb)) {
                    continue;
                }
            }

            const std::optional<ShapePairKind> shapePair =
                classifyShapePair(colliderA, colliderB);

            if (!shapePair) {
                continue;
            }

            processColliderPairNormal(
                batch,
                ResolvedColliderPair{
                    ColliderEndpointRef{
                        pair.bodyA,
                        colAH,
                        &bodyA,
                        &colliderA
                    },
                    ColliderEndpointRef{
                        pair.bodyB,
                        colBH,
                        &bodyB,
                        &colliderB
                    },
                    *shapePair
                }
            );
        }
    }
}

void NarrowphaseManager::processColliderPairNormal(
    ContactBatch& batch,
    ResolvedColliderPair pair)
{
    std::optional<OverlapHit> hit;

    switch (pair.shapePair) {
    case ShapePairKind::BoxBox:
        hit = tryBoxBox(std::move(pair));
        break;

    case ShapePairKind::BoxSphere:
        hit = tryBoxSphere(std::move(pair));
        break;

    case ShapePairKind::SphereSphere:
        hit = trySphereSphere(std::move(pair));
        break;
    }

    if (!hit) {
        return;
    }

    PairKey key = makeColliderPairKey(
        hit->pair.a.colliderHandle,
        hit->pair.b.colliderHandle
    );

    normalHitPairs.insert(key);

    emitRigidContact(batch, *hit);
}

//=======================================================
//     Speculative Dynamic pair processing
//=======================================================
void NarrowphaseManager::processSpeculativeDynamicPairs(
    const SpeculativeDynamicPair& pair,
    float dt)
{
    RigidBody& bodyA = physicsWorld->getBody(pair.bodyA);
    RigidBody& bodyB = physicsWorld->getBody(pair.bodyB);

    for (const ColliderHandle& colAH : bodyA.colliderHandles) {
        for (const ColliderHandle& colBH : bodyB.colliderHandles) {
            Collider& colliderA = physicsWorld->getCollider(colAH);
            Collider& colliderB = physicsWorld->getCollider(colBH);

            // #TODO: Need to use the swept AABBs for speculative collision detection, not the current AABBs.
            //// If either body is compound, check if the colliders' AABBs intersect
            //if (bodyA.isCompound() || bodyB.isCompound()) {
            //    if (!colliderA.aabb.intersects(colliderB.aabb)) {
            //        continue;
            //    }
            //}

            const std::optional<ShapePairKind> shapePair =
                classifyShapePair(colliderA, colliderB);

            if (!shapePair) {
                continue;
            }

            processColliderPairSpeculative(
                ResolvedColliderPair{
                    ColliderEndpointRef{
                        pair.bodyA,
                        colAH,
                        &bodyA,
                        &colliderA
                    },
                    ColliderEndpointRef{
                        pair.bodyB,
                        colBH,
                        &bodyB,
                        &colliderB
                    },
                    *shapePair
                },
                dt,
                pair.sweepOwner
            );
        }
    }
}

void NarrowphaseManager::processColliderPairSpeculative(
    ResolvedColliderPair pair,
    float dt,
    BodyHandle sweepOwner)
{
    PairKey key = makeColliderPairKey(
        pair.a.colliderHandle,
        pair.b.colliderHandle
    );

    // If normal narrowphase already created a real contact,
    // do not also create speculative contact for same collider pair.
    if (normalHitPairs.find(key) != normalHitPairs.end()) {
        return;
    }

    std::optional<SweepHit> hit;

    switch (pair.shapePair) {
    case ShapePairKind::BoxBox:
        hit = trySpeculativeBoxBox(
            std::move(pair),
            dt,
            sweepOwner
        );
        break;

    case ShapePairKind::BoxSphere:
        hit = trySpeculativeBoxSphere(
            std::move(pair),
            dt,
            sweepOwner
        );
        break;

    case ShapePairKind::SphereSphere:
        hit = trySpeculativeSphereSphere(
            std::move(pair),
            dt,
            sweepOwner
        );
        break;
    }

    if (!hit) {
        return;
    }

    pendingSweepHits.emplace_back(std::move(*hit));
}

//=======================================================
//     Normal Terrain pair processing
//=======================================================
void NarrowphaseManager::processTerrainPairs(const TerrainPair& terrainPair, ContactBatch& batch, float dt) {
    RigidBody& body = physicsWorld->getBody(terrainPair.body);
    if (body.asleep || body.type == BodyType::Kinematic) return;

    // per collider in body, collect candidate tris and send to SAT + collision manifold generation
    for (const ColliderHandle& colH : body.colliderHandles) {
        Collider& collider = physicsWorld->getCollider(colH);

        const std::vector<Tri*>* candidates = &terrainPair.tris;

        // if compound, do mid phase AABB tests to filter terrain tris before SAT, otherwise just send all terrain tris to SAT
        if (body.isCompound()) {
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

        switch (collider.type) {
        case ColliderType::CUBOID:
            processTerrainTriBox(batch, collider.rigidBodyHandle, &collider, &body, *candidates);
            break;

        case ColliderType::SPHERE:
            processTerrainTriSphere(batch, collider.rigidBodyHandle, &collider, &body, *candidates);
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
    RigidBody& body = physicsWorld->getBody(pair.body);

    if (body.asleep || body.type == BodyType::Kinematic) {
        return;
    }

    for (const ColliderHandle& colH : body.colliderHandles) {
        Collider& collider = physicsWorld->getCollider(colH);

        const std::vector<Tri*>* candidates = &pair.tris;

        if (body.isCompound()) {
            terrainTriCandidates.clear();
            terrainTriCandidates.reserve(pair.tris.size());

            collectTerrainTriCandidates(collider, pair.tris, terrainTriCandidates);

            if (terrainTriCandidates.empty()) {
                continue;
            }

            candidates = &terrainTriCandidates;
        }

        for (Tri* tri : *candidates) {
            ColliderEndpointRef colliderRef{
                pair.body,
                colH,
                &body,
                &collider
            };

            std::optional<TerrainSweepHit> hit;

            if (collider.type == ColliderType::CUBOID) {
                hit = trySpeculativeBoxTriangle(
                    colliderRef,
                    tri,
                    dt,
                    pair.body
                );
            }
            else if (collider.type == ColliderType::SPHERE) {
                hit = trySpeculativeSphereTriangle(
                    colliderRef,
                    tri,
                    dt,
                    pair.body
                );
            }

            if (!hit) {
                continue;
            }

            pendingSweepHits.emplace_back(std::move(*hit));
        }
    }
}

//=======================================================
//     Collider and body handle packing for pair keys
//=======================================================
uint64_t NarrowphaseManager::packColliderHandle(ColliderHandle h) {
    return (uint64_t(h.slot) << 32) | uint64_t(h.gen);
}
uint64_t NarrowphaseManager::packBodyHandle(BodyHandle h) {
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
    Collider* colliderA, Collider* colliderB) const
{
    ContactRuntime rt;
    rt.bodyA = bodyA;
    rt.bodyB = bodyB;
    rt.colliderA = colliderA;
    rt.colliderB = colliderB;
    return rt;
}

ContactRuntime NarrowphaseManager::makeRuntimeData(
    RigidBody* bodyA,
    Collider* colliderA) const
{
    ContactRuntime rt;
    rt.bodyA = bodyA;
    rt.colliderA = colliderA;
    return rt;
}

//=======================================================
//     Terrain tri candidate collection
//=======================================================
void NarrowphaseManager::collectTerrainTriCandidates(
    Collider& collider,
    const std::vector<Tri*>& inputTris,
    std::vector<Tri*>& outCandidates)
{
    outCandidates.clear();

    const AABB& colAABB = collider.getAABB();
    for (Tri* tri : inputTris) {
        if (tri->getAABB().intersects(colAABB)) {
            outCandidates.push_back(tri);
        }
    }
}


}
