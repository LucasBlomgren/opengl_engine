#include "pch.h"
#include "broadphase_manager.h"
#include "game_object.h"
#include "rigid_body.h"
#include "tri.h"

// -----------------------------------
//      Main functions
// -----------------------------------

void BroadphaseManager::init(SlotMap<Collider, ColliderHandle>* cMap, SlotMap<RigidBody, RigidBodyHandle>* bMap, std::vector<Tri>* terrainTris) {
    colliderMap = cMap;
    bodyMap = bMap;
    terrainTriangles = terrainTris;

    uint32_t slotCap = cMap->dense().capacity();

    awakeBvh.init(cMap, slotCap);
    asleepBvh.init(cMap, slotCap);
    staticBvh.init(cMap, slotCap);

    awakeHandles.clear();
    asleepHandles.clear();
    staticHandles.clear();

    awakeHandles.reserve(slotCap * 2);
    asleepHandles.reserve(slotCap * 2);
    staticHandles.reserve(slotCap * 2);

    terrainBvh.build(*terrainTriangles);
}

// Update BVHs if dirty
void BroadphaseManager::updateBVHs() {
    awakeBvh.update(awakeHandles);

    if (asleepBvh.dirty) {
        asleepBvh.update(asleepHandles);
        asleepBvh.dirty = false;
    }

    staticBvh.update(staticHandles);


    //std::cout << awakeBvh.nodes.size() << " awake nodes, "
    //          << asleepBvh.nodes.size() << " asleep nodes, "
    //    << staticBvh.nodes.size() << " static nodes.\n";
}   

// Compute pairs for this frame
void BroadphaseManager::computePairs() {
    // ----- dynamic vs terrain -----
    if (terrainBvh.rootIdx != -1) {
        pairsBufTerrain.reserve(BVHTree::MaxCollisionBuf);
        pairsBufTerrain.clear();
        treeVsTreeQuery(awakeBvh, terrainBvh, pairsBufTerrain);

        // Sort terrain pairs by Collider to avoid duplicates 
        std::unordered_map<ColliderHandle, std::vector<Tri*>> temp;
        temp.reserve(pairsBufTerrain.size());
        if (temp.bucket_count() < pairsBufTerrain.size())
            temp.reserve(pairsBufTerrain.size());

        for (int i = 0; i < pairsBufTerrain.size(); i++) {
            auto [collider, tri] = pairsBufTerrain[i];
            temp[collider].push_back(tri);
        }

        // Finalize terrain pairs
        terrainPairs.clear();
        int cap = static_cast<int>(temp.size());
        terrainPairs.resize(cap);
        int sp = 0;

        for (auto& [colliderHandle, trisVec] : temp) {
            terrainPairs[sp++] = TerrainPair{ colliderHandle, std::move(trisVec) };
        }
        terrainPairs.resize(sp);
    }

    dynamicPairs.clear();

    // ----- dynamic vs dynamic -----
    if (awakeBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        treeVsTreeQuery(awakeBvh, awakeBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        dynamicPairs.resize(cap);
        int sp = 0;

        for (auto& hp : pairsBufDynamic) {
            dynamicPairs[sp++] = DynamicPair{ hp.first, hp.second };
        }
        dynamicPairs.resize(sp);
    }

    // ----- dynamic vs asleep -----
    if (asleepBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        treeVsTreeQuery(awakeBvh, asleepBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        dynamicPairs.reserve(dynamicPairs.size() + cap);
        int sp = 0;

        for (auto& hp : pairsBufDynamic) {
            dynamicPairs.emplace_back(DynamicPair{ hp.first, hp.second });
        }
    }

    // ----- dynamic vs static -----
    if (staticBvh.rootIdx != -1) {
        pairsBufDynamic.clear();
        treeVsTreeQuery(awakeBvh, staticBvh, pairsBufDynamic);

        int cap = static_cast<int>(pairsBufDynamic.size());
        dynamicPairs.reserve(dynamicPairs.size() + cap);
        int sp = 0;

        for (auto& hp : pairsBufDynamic) {
            dynamicPairs.emplace_back(DynamicPair{ hp.first, hp.second });
        }
    }
}

// Add to list
void BroadphaseManager::add(ColliderHandle& handle, BroadphaseBucket dst) {
    if (!handle.isValid()) {
        std::cout << "[BroadPhaseManager::add] Error: Invalid collider handle\n";
        return;
    }
    Collider* colliderPtr = colliderMap->try_get(handle);
    auto& h = colliderPtr->broadphaseHandle;

    auto& list = listFor(dst);
    list.push_back(handle);
    h.listIdx = (int)list.size() - 1;

    auto& bvh = bvhFor(dst);
    h.leafIdx = bvh.insertLeaf(handle);
    bvh.dirty = true;

    h.bucket = dst;
}

// Remove from current list
void BroadphaseManager::remove(ColliderHandle& handle) {
    if (!handle.isValid()) {
        std::cout << "[BroadPhaseManager::remove] Error: Invalid collider handle\n";
        return;
    }

    Collider* colliderPtr = colliderMap->try_get(handle);
    auto& h = colliderPtr->broadphaseHandle;
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
void BroadphaseManager::moveToAwake(ColliderHandle& handle) {
    if (!handle.isValid()) {
        std::cout << "[BroadPhaseManager::moveToAwake] Error: Invalid collider handle\n";
        return;
    }
    Collider* colliderPtr = colliderMap->try_get(handle);

    if (!colliderPtr->rigidBodyHandle.isValid()) {
        std::cout << "[BroadPhaseManager::moveToAwake] Error: Invalid rigid body handle\n";
        return;
    }
    RigidBody* bodyPtr = bodyMap->try_get(colliderPtr->rigidBodyHandle);

    if (colliderPtr->broadphaseHandle.bucket == BroadphaseBucket::Awake) {
        bodyPtr->setAwake();
        return;
    }

    remove(handle);
    bodyPtr->setAwake();
    add(handle, BroadphaseBucket::Awake);
    awakeBvh.dirty = true;
}
// Move to asleep
void BroadphaseManager::moveToAsleep(ColliderHandle& handle) {
    if (!handle.isValid()) {
        std::cout << "[BroadPhaseManager::moveToAsleep] Error: Invalid collider handle\n";
        return;
    }
    Collider* colliderPtr = colliderMap->try_get(handle);

    if (!colliderPtr->rigidBodyHandle.isValid()) {
        std::cout << "[BroadPhaseManager::moveToAsleep] Error: Invalid rigid body handle\n";
        return;
    }
    RigidBody* bodyPtr = bodyMap->try_get(colliderPtr->rigidBodyHandle);

    if (colliderPtr->broadphaseHandle.bucket == BroadphaseBucket::Asleep) {
        bodyPtr->setAsleep();
        return;
    }

    remove(handle);
    bodyPtr->setAsleep();
    add(handle, BroadphaseBucket::Asleep);
    asleepBvh.dirty = true;
}
// Move to static
void BroadphaseManager::moveToStatic(ColliderHandle& handle) {
    if (!handle.isValid()) {
        std::cout << "[BroadPhaseManager::moveToStatic] Error: Invalid collider handle\n";
        return;
    }
    Collider* colliderPtr = colliderMap->try_get(handle);

    if (!colliderPtr->rigidBodyHandle.isValid()) {
        std::cout << "[BroadPhaseManager::moveToStatic] Error: Invalid rigid body handle\n";
        return;
    }
    RigidBody* bodyPtr = bodyMap->try_get(colliderPtr->rigidBodyHandle);

    if (colliderPtr->broadphaseHandle.bucket == BroadphaseBucket::Static)
        return;

    remove(handle);
    bodyPtr->setStatic();
    add(handle, BroadphaseBucket::Static);
    staticBvh.dirty = true;
}

// ---------------------------------
//     Helpers
// ---------------------------------

// Swap and pop from list
void BroadphaseManager::swapAndPop(ColliderHandle& handle, std::vector<ColliderHandle>& list) {
    if (!handle.isValid()) {
        std::cout << "[BroadPhaseManager::swapAndPop] Error: Invalid collider handle\n";
        return;
    }
    Collider* colliderPtr = colliderMap->try_get(handle);

    int i = colliderPtr->broadphaseHandle.listIdx;
    if (i == -1) return;

    int lastPos = (int)list.size() - 1;
    if (i != lastPos) {
        ColliderHandle movedHandle = list[lastPos];
        list[i] = movedHandle;

        if (!movedHandle.isValid()) {
            std::cout << "[BroadPhaseManager::swapAndPop] Error: Invalid collider handle\n";
            return;
        }
        Collider* movedPtr = colliderMap->try_get(handle);

        movedPtr->broadphaseHandle.listIdx = i;
    }

    list.pop_back();
    colliderPtr->broadphaseHandle.listIdx = -1;
}

// Get BVH for bucket
BVHTree& BroadphaseManager::bvhFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeBvh;
    if (b == BroadphaseBucket::Asleep) return asleepBvh;
    return staticBvh;
}

// Get list for bucket
std::vector<ColliderHandle>& BroadphaseManager::listFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeHandles;
    if (b == BroadphaseBucket::Asleep) return asleepHandles;
    return staticHandles;
}