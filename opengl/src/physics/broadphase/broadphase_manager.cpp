#include "pch.h"
#include "broadphase_manager.h"
#include "game_object.h"
#include "tri.h"

// -----------------------------------
//      Main functions
// -----------------------------------

void BroadphaseManager::init(SlotMap<GameObject, GameObjectHandle>* sm, std::vector<Tri>* terrainTris) {
    slotMap = sm;
    terrainTriangles = terrainTris;

    uint32_t slotCap = sm->dense().capacity();

    awakeBvh.init(sm, slotCap);
    asleepBvh.init(sm, slotCap);
    staticBvh.init(sm, slotCap);

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

        // Sort terrain pairs by GameObject to avoid duplicates 
        std::unordered_map<GameObjectHandle, std::vector<Tri*>> temp;
        temp.reserve(pairsBufTerrain.size());
        if (temp.bucket_count() < pairsBufTerrain.size())
            temp.reserve(pairsBufTerrain.size());

        for (int i = 0; i < pairsBufTerrain.size(); i++) {
            auto [obj, tri] = pairsBufTerrain[i];
            temp[obj].push_back(tri);
        }

        // Finalize terrain pairs
        terrainPairs.clear();
        int cap = static_cast<int>(temp.size());
        terrainPairs.resize(cap);
        int sp = 0;

        for (auto& [objHandle, trisVec] : temp) {
            terrainPairs[sp++] = TerrainPair{ objHandle, std::move(trisVec) };
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
void BroadphaseManager::add(GameObjectHandle& handle, BroadphaseBucket dst) {
    GameObject* objPtr = slotMap->try_get(handle);
    auto& h = objPtr->broadphaseHandle;

    auto& list = listFor(dst);
    list.push_back(handle);
    h.listIdx = (int)list.size() - 1;

    auto& bvh = bvhFor(dst);
    h.leafIdx = bvh.insertLeaf(handle);
    bvh.dirty = true;

    h.bucket = dst;
}

// Remove from current list
void BroadphaseManager::remove(GameObjectHandle& handle) {
    GameObject* objPtr = slotMap->try_get(handle);

    auto& h = objPtr->broadphaseHandle;
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
void BroadphaseManager::moveToAwake(GameObjectHandle& handle) {
    GameObject* objPtr = slotMap->try_get(handle);

    if (objPtr->broadphaseHandle.bucket == BroadphaseBucket::Awake) {
        objPtr->setAwake();
        return;
    }

    remove(handle);
    objPtr->setAwake();
    add(handle, BroadphaseBucket::Awake);
    awakeBvh.dirty = true;
}
// Move to asleep
void BroadphaseManager::moveToAsleep(GameObjectHandle& handle) {
    GameObject* objPtr = slotMap->try_get(handle);

    if (objPtr->broadphaseHandle.bucket == BroadphaseBucket::Asleep) {
        objPtr->setAsleep();
        return;
    }

    remove(handle);
    objPtr->setAsleep();
    add(handle, BroadphaseBucket::Asleep);
    asleepBvh.dirty = true;
}
// Move to static
void BroadphaseManager::moveToStatic(GameObjectHandle& handle) {
    GameObject* objPtr = slotMap->try_get(handle);

    if (objPtr->broadphaseHandle.bucket == BroadphaseBucket::Static)
        return;

    remove(handle);
    objPtr->setStatic();
    add(handle, BroadphaseBucket::Static);
    staticBvh.dirty = true;
}

// ---------------------------------
//     Helpers
// ---------------------------------

// Swap and pop from list
void BroadphaseManager::swapAndPop(GameObjectHandle& obj, std::vector<GameObjectHandle>& list) {
    GameObject* objPtr = slotMap->try_get(obj);

    int i = objPtr->broadphaseHandle.listIdx;
    if (i == -1) return;

    int lastPos = (int)list.size() - 1;

    if (i != lastPos) {
        GameObjectHandle movedHandle = list[lastPos];   // handle in dynamicObjects
        list[i] = movedHandle;

        GameObject* movedPtr = slotMap->try_get(movedHandle);
        movedPtr->broadphaseHandle.listIdx = i;
    }

    list.pop_back();
    objPtr->broadphaseHandle.listIdx = -1;
}

// Get BVH for bucket
BVHTree& BroadphaseManager::bvhFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeBvh;
    if (b == BroadphaseBucket::Asleep) return asleepBvh;
    return staticBvh;
}

// Get list for bucket
std::vector<GameObjectHandle>& BroadphaseManager::listFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeHandles;
    if (b == BroadphaseBucket::Asleep) return asleepHandles;
    return staticHandles;
}