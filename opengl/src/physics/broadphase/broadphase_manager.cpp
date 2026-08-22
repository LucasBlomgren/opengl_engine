#include <unordered_set>

#include "pch.h"
#include "broadphase_manager.h"

#include "physics/bodies/rigidbody.h"
#include "physics/colliders/tri.h"
#include "physics/bvh/query_treetree.h"
#include "physics/bvh/query_sametree.h"

namespace physics::internal {

//=======================================
//    Init & Clear
//=======================================
void BroadphaseManager::init(
    PhysicsWorld* world, 
    std::vector<Tri>* terrainTris) 
{
    this->world = world;
    this->terrainTriangles = terrainTris;

    SlotMap<RigidBody, BodyHandle>& bMap = world->bodyStorage();
    size_t slotCap = bMap.dense().capacity();

    awakeBvh.init(world, slotCap, true);
    asleepBvh.init(world, slotCap, true);
    staticBvh.init(world, slotCap, true);

    // false = speculative pairs don't need to write leaf indices
    speculativeBvh.init(world, slotCap, false);

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
    speculativeBvh.clear();

    pairsBufDynamic.clear();
    pairsBufTerrain.clear();
}

//=======================================
//    Set BVH render data flag
//=======================================
void BroadphaseManager::updateBVHRenderData(const physics::debug::BvhType& type, bool update) {
    switch (type) {
    case physics::debug::BvhType::Awake:
        awakeBvh.shouldUpdateRenderData = update;
        break;
    case physics::debug::BvhType::Asleep:
        asleepBvh.shouldUpdateRenderData = update;
        break;
    case physics::debug::BvhType::Static:
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
    const std::vector<BodyHandle>& bodies,
    std::vector<DynamicPair>& outPairs)
{
    for (int i = 0; i < bodies.size(); ++i) {
        BodyHandle aH = bodies[i];
        RigidBody& a = world->getBody(aH);

        for (int j = i + 1; j < bodies.size(); ++j) {
            BodyHandle bH = bodies[j];
            RigidBody& b = world->getBody(bH);

            if (a.asleep && b.asleep) continue;
            if (!a.aabb.intersects(b.aabb)) continue;

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
        std::unordered_map<BodyHandle, std::vector<Tri*>> temp;
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

void BroadphaseManager::buildSpeculativePairs(float dt, PairBatch& batch, std::vector<AABB>& debugSweeps) 
{
    batch.speculativeDynamicPairs.clear();
    batch.speculativeTerrainPairs.clear();
    batch.speculativeDynamicPairs.reserve(BVHTree::MaxCollisionBuf);
    batch.speculativeTerrainPairs.reserve(BVHTree::MaxCollisionBuf);

    static std::vector<BodyHandle> speculativeBodies;
    static std::vector<AABB> speculativeAABBs;
    speculativeBodies.reserve(awakeHandles.size());
    speculativeAABBs.reserve(awakeHandles.size());
    speculativeBodies.clear();
    speculativeAABBs.clear();
    determineSpeculativeBodies(dt, speculativeBodies, speculativeAABBs);

    speculativeBvh.clear();
    speculativeBvh.build(speculativeBodies, speculativeAABBs);

    struct SpecKey {
        uint64_t owner;
        uint64_t a;
        uint64_t b;

        bool operator==(const SpecKey& other) const {
            return owner == other.owner &&
                a == other.a &&
                b == other.b;
        }
    };

    struct SpecKeyHash {
        size_t operator()(const SpecKey& k) const {
            size_t h = std::hash<uint64_t>{}(k.owner);
            h ^= std::hash<uint64_t>{}(k.a + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2));
            h ^= std::hash<uint64_t>{}(k.b + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2));
            return h;
        }
    };

    static std::unordered_set<SpecKey, SpecKeyHash> seenSpecPairs;
    seenSpecPairs.clear();

    auto packHandle = [](BodyHandle h) -> uint64_t {
        return (uint64_t(h.slot) << 32) | uint64_t(h.gen);
        };

    auto addSpecPair = [&](
        BodyHandle a,
        BodyHandle b,
        BodyHandle owner)
        {
            if (a == b) {
                return;
            }

            uint64_t ak = packHandle(a);
            uint64_t bk = packHandle(b);
            uint64_t ok = packHandle(owner);

            if (bk < ak) {
                std::swap(ak, bk);
            }

            SpecKey key{};
            key.owner = ok;
            key.a = ak;
            key.b = bk;

            if (!seenSpecPairs.insert(key).second) {
                return;
            }

            batch.speculativeDynamicPairs.emplace_back(
                SpeculativeDynamicPair{ a, b, owner }
            );
        };

    // ----- dynamic vs terrain -----
    if (terrainBvh.rootIdx != -1) {
        pairsBufTerrain.reserve(BVHTree::MaxCollisionBuf);
        pairsBufTerrain.clear();
        treeVsTreeQuery(speculativeBvh, terrainBvh, pairsBufTerrain);

        // Sort terrain pairs by RigidBody to avoid duplicates 
        std::unordered_map<BodyHandle, std::vector<Tri*>> temp;
        temp.reserve(pairsBufTerrain.size());
        if (temp.bucket_count() < pairsBufTerrain.size())
            temp.reserve(pairsBufTerrain.size());

        for (int i = 0; i < pairsBufTerrain.size(); i++) {
            auto [rigidBody, tri] = pairsBufTerrain[i];
            temp[rigidBody].push_back(tri);
        }

        // Finalize terrain pairs
        batch.speculativeTerrainPairs.clear();
        int cap = static_cast<int>(temp.size());
        batch.speculativeTerrainPairs.resize(cap);
        int sp = 0;

        for (auto& [bodyHandle, trisVec] : temp) {
            batch.speculativeTerrainPairs[sp++] = SpeculativeTerrainPair{ bodyHandle, std::move(trisVec) };
        }
        batch.speculativeTerrainPairs.resize(sp);
    }

    // ----- speculative vs speculative -----
    if (speculativeBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        treeVsSameTreeQuery(speculativeBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

        for (auto& hp : pairsBufDynamic) {
            BodyHandle a = hp.first;
            BodyHandle b = hp.second;

            addSpecPair(a, b, a); // A's owner
            addSpecPair(a, b, b); // B's owner
        }
    }

    // ----- speculative vs awake -----
    if (awakeBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        treeVsTreeQuery(speculativeBvh, awakeBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

        for (auto& hp : pairsBufDynamic) {
            addSpecPair(hp.first, hp.second, hp.first);
        }
    }

    // ----- speculative vs asleep -----
    if (asleepBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        treeVsTreeQuery(speculativeBvh, asleepBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

        for (auto& hp : pairsBufDynamic) {
            batch.speculativeDynamicPairs.emplace_back(
                SpeculativeDynamicPair{ hp.first, hp.second, hp.first }
            );
        }
    }

    // ----- speculative vs static -----
    if (staticBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        treeVsTreeQuery(speculativeBvh, staticBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

        for (auto& hp : pairsBufDynamic) {
            batch.speculativeDynamicPairs.emplace_back(
                SpeculativeDynamicPair{ hp.first, hp.second, hp.first }
            );
        }
    }
}

void BroadphaseManager::determineSpeculativeBodies(float dt, std::vector<BodyHandle>& outBodies, std::vector<AABB>& outAABBs) {
    for (const BodyHandle& handle : awakeHandles) {
        RigidBody& body = world->getBody(handle);

        if (body.colliderHandles.empty()) {
            continue;
        }

        Collider& mainCollider = world->getCollider(body.colliderHandles[0]);

        if (body.motionControl != MotionControl::Physics) {
            continue;
        }

        constexpr float safeFraction = 0.50f;
        constexpr float minSafeDistance = 0.02f;

        // -----------------------------
        // 1. Estimate object size
        // -----------------------------
        const glm::vec3& scale = body.scale;
        float minExtent = std::min(scale.x, std::min(scale.y, scale.z));
        float boundingRadius = 0.5f * glm::length(scale);
        float safeDistance = std::max(minExtent * safeFraction, minSafeDistance);

        // -----------------------------
        // 2. Estimate motion this frame
        // -----------------------------
        float linearMotion = glm::length(body.linearVelocity) * dt;
        float angularMotion = glm::length(body.angularVelocity) * boundingRadius * dt;
        float totalMotion = linearMotion + angularMotion;

        // Not fast enough to matter.
        if (totalMotion <= safeDistance) {
            continue;
        }

        outBodies.push_back(handle);

        // -----------------------------
        // 3. Build swept AABB
        // -----------------------------
        AABB& currentAABB = body.aabb;

        glm::vec3 delta = body.linearVelocity * dt;

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
        if (mainCollider.type != ColliderType::SPHERE || body.isCompound()) {
            float omega = glm::length(body.angularVelocity);
            if (omega > 1e-6f) {
                glm::vec3 axis = body.angularVelocity / omega;

                // Use the box's OWN oriented axes + local half-extents.
                // The world AABB is sign-agnostic: currentHalf is always the
                // all-positive corner, a phantom point that sits far from the
                // axis when the body's long dimension points "against" it.
                // Measuring that corner's distance to the axis is the bug.
                glm::mat3 R = glm::mat3_cast(body.pose.orientation);
                glm::vec3 h = 0.5f * body.scale; // local half-extents

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
void BroadphaseManager::add(const BodyHandle& handle, BroadphaseBucket dst) {
    RigidBody& rigidBody = world->getBody(handle);
    if (dst == BroadphaseBucket::None) {
        return;
    }

    auto& h = rigidBody.broadphaseHandle;
    if (h.bucket != BroadphaseBucket::None) {
        return;
    }

    auto& list = listFor(dst);
    list.push_back(handle);
    h.listIdx = (int)list.size() - 1;

    auto& bvh = bvhFor(dst);
    h.leafIdx = bvh.insertLeaf(handle);
    bvh.dirty = true;

    h.bucket = dst;
}

// Remove from current list
void BroadphaseManager::remove(const BodyHandle& handle) {
    RigidBody& rigidBody = world->getBody(handle);

    auto& h = rigidBody.broadphaseHandle;
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
void BroadphaseManager::moveToAwake(const BodyHandle& handle) {
    RigidBody& body = world->getBody(handle);

    body.setAwake();

    if (body.broadphaseHandle.bucket == BroadphaseBucket::Awake) {
        return;
    }

    remove(handle);
    add(handle, BroadphaseBucket::Awake);
    awakeBvh.dirty = true;
}
// Move to asleep
void BroadphaseManager::moveToAsleep(const BodyHandle& handle) {
    RigidBody& body = world->getBody(handle);

    body.setAsleep();

    if (body.broadphaseHandle.bucket == BroadphaseBucket::Asleep) {
        return;
    }

    remove(handle);
    add(handle, BroadphaseBucket::Asleep);
    asleepBvh.dirty = true;
}
// Move to static
void BroadphaseManager::moveToStatic(const BodyHandle& handle) {
    RigidBody& body = world->getBody(handle);
    if (body.broadphaseHandle.bucket == BroadphaseBucket::Static) {
        return;
    }

    remove(handle);
    body.setStatic();
    add(handle, BroadphaseBucket::Static);
    staticBvh.dirty = true;
}

void BroadphaseManager::setBVHDirty(const BodyHandle& handle) {
    RigidBody& rigidBody = world->getBody(handle);
    auto& h = rigidBody.broadphaseHandle;
    if (h.bucket == BroadphaseBucket::None) return;
    bvhFor(h.bucket).dirty = true;
}

//==================================================
//     Helpers
//==================================================
// Swap and pop from list
void BroadphaseManager::swapAndPop(
    const BodyHandle& handle, 
    std::vector<BodyHandle>& list) 
{
    RigidBody& rigidBody = world->getBody(handle);

    int i = rigidBody.broadphaseHandle.listIdx;
    if (i == -1) return;

    if (i < 0 ||
        i >= static_cast<int>(list.size()) ||
        !(list[static_cast<size_t>(i)] == handle)) {
        auto position = std::find(list.begin(), list.end(), handle);
        if (position == list.end()) {
            rigidBody.broadphaseHandle.listIdx = -1;
            return;
        }

        i = static_cast<int>(std::distance(list.begin(), position));
        rigidBody.broadphaseHandle.listIdx = i;
    }

    int lastPos = (int)list.size() - 1;
    if (i != lastPos) {
        BodyHandle movedHandle = list[lastPos];
        list[i] = movedHandle;

        RigidBody& moved = world->getBody(movedHandle);
        moved.broadphaseHandle.listIdx = i;
    }

    list.pop_back();
    rigidBody.broadphaseHandle.listIdx = -1;
}

// Get BVH for bucket
BVHTree& BroadphaseManager::bvhFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeBvh;
    if (b == BroadphaseBucket::Asleep) return asleepBvh;
    return staticBvh;
}

// Get list for bucket
std::vector<BodyHandle>& BroadphaseManager::listFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeHandles;
    if (b == BroadphaseBucket::Asleep) return asleepHandles;
    return staticHandles;
}

}
