#include "pch.h"
#include "broadphase_manager.h"
#include "game_object.h"
#include "rigid_body.h"
#include "tri.h"

void BroadphaseManager::init(PhysicsWorld* world, RuntimeCaches* caches, std::vector<Tri>* terrainTris) {
    this->caches = caches;
    this->terrainTriangles = terrainTris;

    SlotMap<RigidBody, RigidBodyHandle>* bMap = caches->bodies.sm; // sm=slotmap
    uint32_t slotCap = bMap->dense().capacity();

    awakeBvh.init(world, caches, slotCap);
    asleepBvh.init(world, caches, slotCap);
    staticBvh.init(world, caches, slotCap);
    awakeHandles.clear();
    asleepHandles.clear();
    staticHandles.clear();

    awakeHandles.reserve(slotCap * 2);
    asleepHandles.reserve(slotCap * 2);
    staticHandles.reserve(slotCap * 2);

    terrainBvh.build(*terrainTriangles);
}

// -----------------------------------
//      Main functions
// -----------------------------------

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
        terrainPairs.clear();
        int cap = static_cast<int>(temp.size());
        terrainPairs.resize(cap);
        int sp = 0;

        for (auto& [bodyHandle, trisVec] : temp) {
            terrainPairs[sp++] = TerrainPair{ bodyHandle, std::move(trisVec) };
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

// ---------------------------------
//     Helpers
// ---------------------------------

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