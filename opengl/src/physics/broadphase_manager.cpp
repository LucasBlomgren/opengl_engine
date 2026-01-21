#include "pch.h"
#include "broadphase_manager.h"
#include "game_object.h"
#include "tri.h"

// -----------------------------------
//      Main functions
// -----------------------------------

void BroadphaseManager::init(std::vector<GameObject>* gameObjects, std::vector<Tri>* terrainTris) {
    this->dynamicObjects = gameObjects;

    // #TODO: Skapa generell freelist struktur för att undvika massiva vektorer med tomma platser
    // och jobba med indexer istället för pekare överallt

    awakeIds.clear();
    awakeIds.reserve(5000000);
    asleepIds.clear();
    asleepIds.reserve(5000000);
    staticIds.clear();
    staticIds.reserve(5000000);

    awakeBvh.nodes.reserve(5000000);
    awakeBvh.rootIdx = -1;
    awakeBvh.nodes.clear();

    asleepBvh.nodes.reserve(5000000);
    asleepBvh.rootIdx = -1;
    asleepBvh.nodes.clear();

    staticBvh.nodes.reserve(5000000);
    staticBvh.rootIdx = -1;
    staticBvh.nodes.clear();

    awakeBvh.build(*dynamicObjects, awakeIds, false);
    asleepBvh.build(*dynamicObjects, asleepIds, false);
    staticBvh.build(*dynamicObjects, staticIds, false);

    this->terrainTriangles = terrainTris;
    std::vector<int> placeholder;
    terrainBvh.build(*terrainTris, placeholder, true);
}

// Update BVHs if dirty
void BroadphaseManager::updateBVHs() {
    //if (awakeBvh.dirty) {
        awakeBvh.update(*dynamicObjects, awakeIds, false);
        //awakeBvh.dirty = false;
    //}
    if (asleepBvh.dirty) {
        asleepBvh.update(*dynamicObjects, asleepIds, false);
        asleepBvh.dirty = false;
    }
    //if (staticBvh.dirty) {
        staticBvh.update(*dynamicObjects, staticIds, false);
        //staticBvh.dirty = false;
    //}

    std::cout << awakeBvh.nodes.size() << " awake nodes, "
              << asleepBvh.nodes.size() << " asleep nodes, "
        << staticBvh.nodes.size() << " static nodes.\n";
}   

// Compute pairs for this frame
void BroadphaseManager::computePairs() {
    // ----- dynamic vs terrain -----
    if (terrainBvh.rootIdx != -1) {
        pairsBufTerrain.reserve(BVHTree<Tri>::MaxCollisionBuf);
        pairsBufTerrain.clear();
        treeVsTreeQuery(awakeBvh, terrainBvh, pairsBufTerrain);

        // Sort terrain pairs by GameObject to avoid duplicates 
        std::unordered_map<GameObject*, std::vector<Tri*>> temp;
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

        for (auto& [obj, trisVec] : temp) {
            terrainPairs[sp++] = TerrainPair{ obj, std::move(trisVec) };
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
            GameObject* A = hp.first, * B = hp.second;

            if (A == B) continue;
            dynamicPairs[sp++] = DynamicPair{ A,B };
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
            GameObject* A = hp.first, * B = hp.second;

            if (A == B) continue;
            dynamicPairs.emplace_back(DynamicPair{ A,B });
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
            GameObject* A = hp.first, * B = hp.second;

            if (A == B) continue;
            dynamicPairs.emplace_back(DynamicPair{ A,B });
        }
    }
}

// Add to list
void BroadphaseManager::add(GameObject& obj, BroadphaseBucket dst) {
    auto& h = obj.broadphaseHandle;

    auto& list = listFor(dst);
    list.push_back(obj.dynamicObjectIdx);
    h.listIdx = (int)list.size() - 1;

    auto& bvh = bvhFor(dst);
    h.leafIdx = bvh.insertLeaf(&obj);   // ideal: return handle
    bvh.dirty = true;

    h.bucket = dst;
}

// Remove from current list
void BroadphaseManager::remove(GameObject& obj) {
    auto& h = obj.broadphaseHandle;
    if (h.bucket == BroadphaseBucket::None) return;

    // remove från lista
    swapAndPop(obj, listFor(h.bucket));

    // remove från BVH
    bvhFor(h.bucket).removeLeaf(h.leafIdx);
    bvhFor(h.bucket).dirty = true;

    h.leafIdx = -1;
    h.bucket = BroadphaseBucket::None;
}

// Move to awake
void BroadphaseManager::moveToAwake(GameObject& obj) {
    if (obj.broadphaseHandle.bucket == BroadphaseBucket::Awake) {
        obj.setAwake();
        return;
    }

    remove(obj);
    obj.setAwake();
    add(obj, BroadphaseBucket::Awake);
    awakeBvh.dirty = true;
}
// Move to asleep
void BroadphaseManager::moveToAsleep(GameObject& obj) {
    if (obj.broadphaseHandle.bucket == BroadphaseBucket::Asleep) {
        obj.setAsleep();
        return;
    }

    remove(obj);
    obj.setAsleep();
    add(obj, BroadphaseBucket::Asleep);
    asleepBvh.dirty = true;
}
// Move to static
void BroadphaseManager::moveToStatic(GameObject& obj) {
    if (obj.broadphaseHandle.bucket == BroadphaseBucket::Static)
        return;

    remove(obj);
    obj.setStatic();
    add(obj, BroadphaseBucket::Static);
    staticBvh.dirty = true;
}

// ---------------------------------
//     Helpers
// ---------------------------------

// Swap and pop from list
void BroadphaseManager::swapAndPop(GameObject& obj, std::vector<int>& list) {
    int i = obj.broadphaseHandle.listIdx;
    if (i == -1) return;

    int lastPos = (int)list.size() - 1;
    if (i != lastPos) {
        int movedObjIndex = list[lastPos];                 // index i dynamicObjects
        list[i] = movedObjIndex;

        GameObject& movedObj = (*dynamicObjects)[movedObjIndex];
        movedObj.broadphaseHandle.listIdx = i;
    }

    list.pop_back();
    obj.broadphaseHandle.listIdx = -1;
}

// Get BVH for bucket
BVHTree<GameObject>& BroadphaseManager::bvhFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeBvh;
    if (b == BroadphaseBucket::Asleep) return asleepBvh;
    return staticBvh;
}

// Get list for bucket
std::vector<int>& BroadphaseManager::listFor(BroadphaseBucket b) {
    if (b == BroadphaseBucket::Awake)  return awakeIds;
    if (b == BroadphaseBucket::Asleep) return asleepIds;
    return staticIds;
}