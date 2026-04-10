#include "pch.h"
#include "narrowphase_manager.h"

#include "dynamic_processing.h"
#include "terrain_processing.h"

//---------------------------------------------
//              Initialization
//---------------------------------------------
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

//---------------------------------------------
//               Narrow phase
//---------------------------------------------
void NarrowphaseManager::narrowPhase(const std::vector<TerrainPair>& terrainHits, const std::vector<DynamicPair>& dynamicHits) {
    externalContacts.clear();

    for (const TerrainPair& pair : terrainHits) {
        processTerrainPair(pair);
    }

    for (const DynamicPair& pair : dynamicHits) {
        processDynamicPair(pair);
    }
}

//----------------------------------------------
//     Dynamic pair processing
//----------------------------------------------
void NarrowphaseManager::processDynamicPair(const DynamicPair& pair) {
    RigidBody* bodyA = caches->bodies.get(pair.bodyA, FUNC_NAME);
    RigidBody* bodyB = caches->bodies.get(pair.bodyB, FUNC_NAME);

    if ((bodyA->type == BodyType::Static || bodyA->type == BodyType::Kinematic) and
        (bodyB->type == BodyType::Static || bodyB->type == BodyType::Kinematic)) {
        return;
    }

    // per collider per collider, send to SAT + collision manifold generation
    for (const ColliderHandle& colAH : bodyA->colliderHandles) {
        for (const ColliderHandle& colBH : bodyB->colliderHandles) {
            Collider* colliderA = caches->colliders.get(colAH, FUNC_NAME);
            Collider* colliderB = caches->colliders.get(colBH, FUNC_NAME);

            // if either body is compound, do mid phase AABB test before SAT to filter out unnecessary SAT tests
            if (bodyA->isCompound() || bodyB->isCompound()) {
                if (!colliderA->getAABB().intersects(colliderB->getAABB())) {
                    continue;
                }
            }

            SAT_resultsList.clear();
            SAT_resultsList.reserve(terrainTriCandidates.size());

            if (colliderA->type == ColliderType::CUBOID and colliderB->type == ColliderType::CUBOID) {
                processBoxBox(pair.bodyA, pair.bodyB, colAH, colBH, bodyA, bodyB, colliderA, colliderB);
            }
            else if ((colliderA->type == ColliderType::CUBOID and colliderB->type == ColliderType::SPHERE) or
                (colliderA->type == ColliderType::SPHERE and colliderB->type == ColliderType::CUBOID)) {
                processBoxSphere(pair.bodyA, pair.bodyB, colAH, colBH, bodyA, bodyB, colliderA, colliderB);
            }
            else if (colliderA->type == ColliderType::SPHERE and colliderB->type == ColliderType::SPHERE) {
                processSphereSphere(pair.bodyA, pair.bodyB, colAH, colBH, bodyA, bodyB, colliderA, colliderB);
            }
        }
    }
}


//----------------------------------------------
//     Terrain pair processing
//----------------------------------------------
void NarrowphaseManager::processTerrainPair(const TerrainPair& pair) {
    RigidBody* body = caches->bodies.get(pair.body, FUNC_NAME);
    if (body->asleep || body->type == BodyType::Kinematic) return;

    // per collider in body, collect candidate tris and send to SAT + collision manifold generation
    for (const ColliderHandle& colH : body->colliderHandles) {
        Collider* collider = caches->colliders.get(colH, FUNC_NAME);

        // if compound, do mid phase AABB tests to filter terrain tris before SAT, otherwise just send all terrain tris to SAT
        if (body->isCompound()) {
            terrainTriCandidates.clear();
            terrainTriCandidates.reserve(pair.tris.size());
            collectTerrainTriCandidates(collider, pair.tris, terrainTriCandidates);

            if (terrainTriCandidates.empty()) {
                continue;
            }
        }

        SAT_resultsList.clear();
        SAT_resultsList.reserve(terrainTriCandidates.size());

        switch (collider->type) {
        case ColliderType::CUBOID:
            processTerrainTriBox(collider->rigidBodyHandle, collider, body);
            break;

        case ColliderType::SPHERE:
            processTerrainTriSphere(collider->rigidBodyHandle, collider , body);
            break;
        }
    }
}

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


//----------------------------------------------
//      Helper functions
//----------------------------------------------
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