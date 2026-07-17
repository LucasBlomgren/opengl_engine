#include "pch.h"
#include "broadphase_manager.h"
#include "rigidbody.h"
#include "tri.h"
#include "bvh/query_treetree.h"
#include "bvh/query_sametree.h"
#include "sweep_and_prune.h"

//=======================================
//    Init & Clear
//=======================================
void BroadphaseManager::init(PhysicsWorld* world, RuntimeCaches* caches, std::vector<Tri>* terrainTris) {
    this->caches = caches;
    this->terrainTriangles = terrainTris;

    SlotMap<RigidBody, RigidBodyHandle>* bMap = caches->bodies.sm; // sm=slotmap
    size_t slotCap = bMap->dense().capacity();

    awakeBvh.init(world, caches, slotCap);
    asleepBvh.init(world, caches, slotCap);
    staticBvh.init(world, caches, slotCap);

    awakeHandles.reserve(slotCap * 2);
    asleepHandles.reserve(slotCap * 2);
    staticHandles.reserve(slotCap * 2);

    terrainBvh.build(*terrainTriangles);
}

void BroadphaseManager::clear() {
    awakeHandles.clear();
    asleepHandles.clear();
    staticHandles.clear();

    awakeBvh.clear();
    asleepBvh.clear();
    staticBvh.clear();

    pairsBufDynamic.clear();
    pairsBufTerrain.clear();
}

//=======================================
//    Set BVH render data flag
//=======================================
void BroadphaseManager::updateBVHRenderData(const BVHType& type, bool update) {
    switch (type) {
    case BVHType::Awake:
        awakeBvh.shouldUpdateRenderData = update;
        break;
    case BVHType::Asleep:
        asleepBvh.shouldUpdateRenderData = update;
        break;
    case BVHType::Static:
        staticBvh.shouldUpdateRenderData = update;
        break;
    }
}

//==================================================
//      Update BVHs
//==================================================
void BroadphaseManager::updateBVHs() {
    awakeBvh.update(awakeHandles);

    if (asleepBvh.dirty) asleepBvh.update(asleepHandles);
    if (staticBvh.dirty) staticBvh.build(staticHandles);
}


//==================================================
//     Brute force pair building for narrowphase
//==================================================
void BroadphaseManager::buildPairsBruteForce(
    const std::vector<RigidBodyHandle>& bodies,
    std::vector<DynamicPair>& outPairs)
{
    for (int i = 0; i < bodies.size(); ++i) {
        RigidBodyHandle aH = bodies[i];
        RigidBody* a = caches->bodies.get(aH, FUNC_NAME);

        for (int j = i + 1; j < bodies.size(); ++j) {
            RigidBodyHandle bH = bodies[j];
            RigidBody* b = caches->bodies.get(bH, FUNC_NAME);

            if (a->asleep && b->asleep) continue;
            if (!a->aabb.intersects(b->aabb)) continue;

            outPairs.push_back(DynamicPair{aH, bH});
        }
    }
}

//==================================================
//     Global pair building for narrowphase
//==================================================
void BroadphaseManager::buildPairs(PairBatch& batch)
{
    batch.dynamicPairs.reserve(BVHTree::MaxCollisionBuf);
    batch.terrainPairs.reserve(BVHTree::MaxCollisionBuf);

    // ----- dynamic vs terrain -----
    if (terrainBvh.rootIdx != -1) {
        pairsBufTerrain.reserve(BVHTree::MaxCollisionBuf);
        pairsBufTerrain.clear();
        treeVsTreeQuery(awakeBvh, terrainBvh, pairsBufTerrain);

        // Sort terrain pairs by RigidBody to avoid duplicates 
        std::unordered_map<RigidBodyHandle, std::vector<Tri*>> temp;
        temp.reserve(pairsBufTerrain.size());
        if (temp.bucket_count() < pairsBufTerrain.size())
            temp.reserve(pairsBufTerrain.size());

        for (int i = 0; i < pairsBufTerrain.size(); i++) {
            auto [rigidBody, tri] = pairsBufTerrain[i];
            temp[rigidBody].push_back(tri);
        }

        // Finalize terrain pairs
        batch.terrainPairs.clear();
        int cap = static_cast<int>(temp.size());
        batch.terrainPairs.resize(cap);
        int sp = 0;

        for (auto& [bodyHandle, trisVec] : temp) {
            batch.terrainPairs[sp++] = TerrainPair{ bodyHandle, std::move(trisVec) };
        }
        batch.terrainPairs.resize(sp);
    }

    batch.dynamicPairs.clear();
        
    // ----- dynamic vs dynamic -----
    if (awakeBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        //treeVsTreeQuery(awakeBvh, awakeBvh, pairsBufDynamic);
        treeVsSameTreeQuery(awakeBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        batch.dynamicPairs.resize(cap);
        int sp = 0;

        for (auto& hp : pairsBufDynamic) {
            batch.dynamicPairs[sp++] = DynamicPair{ hp.first, hp.second };
        }
        batch.dynamicPairs.resize(sp);
    }

    // ----- dynamic vs asleep -----
    if (asleepBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        treeVsTreeQuery(awakeBvh, asleepBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        batch.dynamicPairs.reserve(batch.dynamicPairs.size() + cap);
        int sp = 0;

        for (auto& hp : pairsBufDynamic) {
            batch.dynamicPairs.emplace_back(DynamicPair{ hp.first, hp.second });
        }
    }

    // ----- dynamic vs static -----
    if (staticBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        treeVsTreeQuery(awakeBvh, staticBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        batch.dynamicPairs.reserve(batch.dynamicPairs.size() + cap);
        int sp = 0;

        for (auto& hp : pairsBufDynamic) {
            batch.dynamicPairs.emplace_back(DynamicPair{ hp.first, hp.second });
        }
    }
}

//==================================================
//     Build speculative pairs for narrowphase
//==================================================
void BroadphaseManager::buildSpeculativePairs(float dt, PairBatch& batch, std::vector<AABB>& debugSweeps) {
    batch.speculativeDynamicPairs.clear();
    batch.speculativeTerrainPairs.clear();
    batch.speculativeDynamicPairs.reserve(BVHTree::MaxCollisionBuf);
    batch.speculativeTerrainPairs.reserve(BVHTree::MaxCollisionBuf);

    static std::vector<RigidBodyHandle> speculativeBodies;
    static std::vector<AABB> speculativeAABBs;
    speculativeBodies.reserve(awakeHandles.size());
    speculativeAABBs.reserve(awakeHandles.size());
    speculativeBodies.clear();
    speculativeAABBs.clear();
    determineSpeculativeBodies(dt, speculativeBodies, speculativeAABBs);

    // #TODO: Let debug renderer use speculativeAABBs directly instead of copying to debugSweeps.
    debugSweeps = speculativeAABBs;
    
    // ----- speculative dynamic vs speculative dynamic -----
    // #TODO: This is O(n^2) and could be improved with sweep-and-prune
    for (size_t i = 0; i < speculativeBodies.size(); ++i) {
        for (size_t j = i + 1; j < speculativeBodies.size(); ++j) {
            if (!speculativeAABBs[i].intersects(speculativeAABBs[j])) {
                continue;
            }

            RigidBodyHandle bodyA = speculativeBodies[i];
            RigidBodyHandle bodyB = speculativeBodies[j];

            if (bodyA == bodyB) {
                continue;
            }

            // Add once for A's sweep ownership.
            batch.speculativeDynamicPairs.emplace_back(
                SpeculativeDynamicPair{ bodyA, bodyB, bodyA }
            );

            // Add once for B's sweep ownership.
            // This lets min-TOI filtering consider this pair for both bodies.
            batch.speculativeDynamicPairs.emplace_back(
                SpeculativeDynamicPair{ bodyA, bodyB, bodyB }
            );
        }
    }

    static std::vector<RigidBodyHandle> candidates;
    static std::vector<Tri*> terrainCandidates;
    candidates.reserve(BVHTree::MaxCollisionBuf);
    terrainCandidates.reserve(BVHTree::MaxCollisionBuf);

    for (int i = 0; i < speculativeBodies.size(); ++i) {
        RigidBodyHandle bodyHandle = speculativeBodies[i];
        AABB& sweptAABB = speculativeAABBs[i];

        // ----- speculative dynamic vs awake -----
        if (awakeBvh.rootIdx != -1) { 
            candidates.clear(); 
            awakeBvh.singleQuery(sweptAABB, candidates);
            for (auto& otherHandle : candidates) {
                if (bodyHandle == otherHandle) continue;
                batch.speculativeDynamicPairs.emplace_back(
                    SpeculativeDynamicPair{ bodyHandle, otherHandle, bodyHandle });
            }
        }
        // ----- speculative dynamic vs asleep -----
        if (asleepBvh.rootIdx != -1) {
            candidates.clear();
            asleepBvh.singleQuery(sweptAABB, candidates);
            for (auto& otherHandle : candidates) {
                if (bodyHandle == otherHandle) continue;
                batch.speculativeDynamicPairs.emplace_back(
                    SpeculativeDynamicPair{ bodyHandle, otherHandle, bodyHandle }
                );
            }
        }
        //----- speculative dynamic vs static -----
        if (staticBvh.rootIdx != -1) {
            candidates.clear();
            staticBvh.singleQuery(sweptAABB, candidates);
            for (auto& otherHandle : candidates) {
                if (bodyHandle == otherHandle) continue;
                batch.speculativeDynamicPairs.emplace_back(
                    SpeculativeDynamicPair{ bodyHandle, otherHandle, bodyHandle }
                );
            }
        }

        // ----- speculative dynamic vs terrain -----
        if (terrainBvh.rootIdx != -1) {
            candidates.clear();
            terrainBvh.singleQuery(sweptAABB, terrainCandidates);
            batch.speculativeTerrainPairs.emplace_back(
                SpeculativeTerrainPair{ bodyHandle, std::move(terrainCandidates) }
            );
        }
    }
}

void BroadphaseManager::determineSpeculativeBodies(float dt, std::vector<RigidBodyHandle>& outBodies, std::vector<AABB>& outAABBs) {
    for (const RigidBodyHandle& handle : awakeHandles) {
        RigidBody* body = caches->bodies.get(handle, FUNC_NAME);
        Collider* mainCollider = caches->colliders.get(body->colliderHandles[0], FUNC_NAME);
        Transform* rootTransform = caches->transforms.get(body->rootTransformHandle, FUNC_NAME);

        constexpr float safeFraction = 0.50f;
        constexpr float minSafeDistance = 0.02f;

        // -----------------------------
        // 1. Estimate object size
        // -----------------------------
        glm::vec3& scale = rootTransform->scale;
        float minExtent = std::min(scale.x, std::min(scale.y, scale.z));
        float boundingRadius = 0.5f * glm::length(scale);
        float safeDistance = std::max(minExtent * safeFraction, minSafeDistance);

        // -----------------------------
        // 2. Estimate motion this frame
        // -----------------------------
        float linearMotion = glm::length(body->linearVelocity) * dt;
        float angularMotion = glm::length(body->angularVelocity) * boundingRadius * dt;
        float totalMotion = linearMotion + angularMotion;

        // Not fast enough to matter.
        if (totalMotion <= safeDistance) {
            continue;
        }

        outBodies.push_back(handle);

        // -----------------------------
        // 3. Build swept AABB
        // -----------------------------
        AABB& currentAABB = body->aabb;

        glm::vec3 delta = body->linearVelocity * dt;

        AABB endAABB = currentAABB;
        endAABB.worldMin += delta;
        endAABB.worldMax += delta;

        outAABBs.emplace_back();
        AABB& sweptAABB = outAABBs.back();
        sweptAABB.worldMin = glm::min(currentAABB.worldMin, endAABB.worldMin);
        sweptAABB.worldMax = glm::max(currentAABB.worldMax, endAABB.worldMax);

        // -----------------------------
        // 4. Expand for rotation
        // -----------------------------
        if (mainCollider->type != ColliderType::SPHERE || body->isCompound()) {
            float omega = glm::length(body->angularVelocity);
            if (omega > 1e-6f) {
                glm::vec3 axis = body->angularVelocity / omega;

                // Use the box's OWN oriented axes + local half-extents.
                // The world AABB is sign-agnostic: currentHalf is always the
                // all-positive corner, a phantom point that sits far from the
                // axis when the body's long dimension points "against" it.
                // Measuring that corner's distance to the axis is the bug.
                glm::mat3 R = glm::mat3_cast(rootTransform->orientation);
                glm::vec3 h = 0.5f * rootTransform->scale; // local half-extents

                // Max perpendicular distance of any box point to the spin axis
                // (conservative). A box axis PARALLEL to the spin axis contributes
                // 0 (that extent doesn't sweep); perpendicular contributes fully.
                float sweepRadius =
                    h.x * glm::length(glm::cross(R[0], axis)) +
                    h.y * glm::length(glm::cross(R[1], axis)) +
                    h.z * glm::length(glm::cross(R[2], axis));

                float theta = omega * dt;
                float arcExpansion = theta * sweepRadius;

                glm::vec3 currentHalf =
                    (currentAABB.worldMax - currentAABB.worldMin) * 0.5f;

                glm::vec3 maxRotExpansion =
                    glm::max(glm::vec3(0.0f),
                        glm::vec3(boundingRadius) - currentHalf);

                glm::vec3 angularExpansion =
                    glm::min(glm::vec3(arcExpansion), maxRotExpansion);

                sweptAABB.worldMin -= angularExpansion;
                sweptAABB.worldMax += angularExpansion;
            }
        }

        // Optional small skin.
        constexpr float sweptSkin = 0.01f;
        sweptAABB.worldMin -= glm::vec3(sweptSkin);
        sweptAABB.worldMax += glm::vec3(sweptSkin);
    }
}

//==================================================
//    List management
//==================================================
void BroadphaseManager::add(RigidBodyHandle& handle, BroadphaseBucket dst) {
    RigidBody* rigidBody = caches->bodies.get(handle, FUNC_NAME);
    auto& h = rigidBody->broadphaseHandle;

    auto& list = listFor(dst);
    list.push_back(handle);
    h.listIdx = (int)list.size() - 1;

    auto& bvh = bvhFor(dst);
    h.leafIdx = bvh.insertLeaf(handle);
    bvh.dirty = true;

    h.bucket = dst;
}

// Remove from current list
void BroadphaseManager::remove(RigidBodyHandle& handle) {
    RigidBody* rigidBody = caches->bodies.get(handle, FUNC_NAME);
    auto& h = rigidBody->broadphaseHandle;
    if (h.bucket == BroadphaseBucket::None) return;

    // remove from list
    swapAndPop(handle, listFor(h.bucket));

    // remove from BVH
    bvhFor(h.bucket).removeLeaf(h.leafIdx);
    bvhFor(h.bucket).dirty = true;

    h.leafIdx = -1;
    h.bucket = BroadphaseBucket::None;
}

// Move to awake
void BroadphaseManager::moveToAwake(RigidBodyHandle& handle) {
    RigidBody* body = caches->bodies.get(handle, FUNC_NAME);
    body->setAwake();

    if (body->broadphaseHandle.bucket == BroadphaseBucket::Awake) {
        return;
    }

    remove(handle);
    add(handle, BroadphaseBucket::Awake);
    awakeBvh.dirty = true;
}
// Move to asleep
void BroadphaseManager::moveToAsleep(RigidBodyHandle& handle) {
    RigidBody* body = caches->bodies.get(handle, FUNC_NAME);
    Transform* transform = caches->transforms.get(body->rootTransformHandle, FUNC_NAME);
    body->setAsleep(*transform);

    if (body->broadphaseHandle.bucket == BroadphaseBucket::Asleep) {
        return;
    }

    remove(handle);
    add(handle, BroadphaseBucket::Asleep);
    asleepBvh.dirty = true;
}
// Move to static
void BroadphaseManager::moveToStatic(RigidBodyHandle& handle) {
    RigidBody* body = caches->bodies.get(handle, FUNC_NAME);

    if (body->broadphaseHandle.bucket == BroadphaseBucket::Static)
        return;

    remove(handle);
    body->setStatic();
    add(handle, BroadphaseBucket::Static);
    staticBvh.dirty = true;
}

void BroadphaseManager::setBVHDirty(RigidBodyHandle& handle) {
    RigidBody* rigidBody = caches->bodies.get(handle, FUNC_NAME);
    auto& h = rigidBody->broadphaseHandle;
    if (h.bucket == BroadphaseBucket::None) return;
    bvhFor(h.bucket).dirty = true;
}

//==================================================
//     Helpers
//==================================================
// Swap and pop from list
void BroadphaseManager::swapAndPop(RigidBodyHandle& handle, std::vector<RigidBodyHandle>& list) {
    RigidBody* rigidBody = caches->bodies.get(handle, FUNC_NAME);

    int i = rigidBody->broadphaseHandle.listIdx;
    if (i == -1) return;

    int lastPos = (int)list.size() - 1;
    if (i != lastPos) {
        RigidBodyHandle movedHandle = list[lastPos];
        list[i] = movedHandle;

        RigidBody* moved = caches->bodies.get(movedHandle, FUNC_NAME);
        moved->broadphaseHandle.listIdx = i;
    }

    list.pop_back();
    rigidBody->broadphaseHandle.listIdx = -1;
}

// Get BVH for bucket
BVHTree& BroadphaseManager::bvhFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeBvh;
    if (b == BroadphaseBucket::Asleep) return asleepBvh;
    return staticBvh;
}

// Get list for bucket
std::vector<RigidBodyHandle>& BroadphaseManager::listFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeHandles;
    if (b == BroadphaseBucket::Asleep) return asleepHandles;
    return staticHandles;
}