#include "pch.h"
#include "broadphase_manager.h"
#include "rigid_body.h"
#include "tri.h"
#include "bvh/query_treetree.h"
#include "bvh/query_sametree.h"

#include "sweep_and_prune.h"
#include "SAP_test.h"

//=======================================
//    Init & Clear
//=======================================
void BroadphaseManager::debugTest() {
    static int frameCounter = 0;
    frameCounter++;

    switch (frameCounter % 3) {
    case 0:
        measureBVH(debugSubsteps);
        measureBrute(debugSubsteps);
        measureSweepAndPrune(debugSubsteps);
        break;

    case 1:
        measureBrute(debugSubsteps);
        measureSweepAndPrune(debugSubsteps);
        measureBVH(debugSubsteps);
        break;

    case 2:
        measureSweepAndPrune(debugSubsteps);
        measureBVH(debugSubsteps);
        measureBrute(debugSubsteps);
        break;
    }
}

void BroadphaseManager::measureBVH(int substeps) {
    double start = glfwGetTime();

    tempIslandBvh.init(caches, awakeHandles.capacity());
    tempIslandBvh.build(awakeHandles);

    for (int step = 0; step < substeps; ++step)
    {
        pairsBufTerrain.clear();
        pairsBufDynamic.clear();

        if (step > 0) {
            tempIslandBvh.update(awakeHandles);
        }

        treeVsTreeQuery(tempIslandBvh, terrainBvh, pairsBufTerrain);
        treeVsTreeQuery(tempIslandBvh, asleepBvh, pairsBufDynamic);
        treeVsSameTreeQuery(tempIslandBvh, pairsBufDynamic);
    }

    double end = glfwGetTime();
    double bvhTotal = (end - start) * 1000.0;

    debugResultBvh.push_back(bvhTotal);
}

void BroadphaseManager::measureBrute(int substeps) {
    static std::vector<Tri*> collisionsTris; 
    static std::vector<RigidBodyHandle> collisionsBodies; 
    static std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>> collisionsBodyPairs;

    double start = glfwGetTime();

    for (int step = 0; step < substeps; ++step)
    {
        collisionsTris.clear();
        collisionsBodies.clear();
        collisionsBodyPairs.clear();

        for (int i = 0; i < awakeHandles.size(); i++)
        {
            RigidBody& rbA = *caches->bodies.get(awakeHandles[i], FUNC_NAME);
            AABB& boxA = rbA.aabb;

            terrainBvh.singleQuery(boxA, collisionsTris);
            asleepBvh.singleQuery(boxA, collisionsBodies);

            for (int k = i + 1; k < awakeHandles.size(); k++) {
                RigidBody& rbB = *caches->bodies.get(awakeHandles[k], FUNC_NAME);
                AABB& boxB = rbB.aabb;

                if (boxA.intersects(boxB)) {
                    collisionsBodyPairs.push_back({ awakeHandles[i], awakeHandles[k] });
                }
            }
        }
    }

    double end = glfwGetTime();
    double bruteTotal = (end - start) * 1000.0;

    debugResultBruteForce.push_back(bruteTotal);
}

void BroadphaseManager::measureSweepAndPrune(int substeps) {
    double start = glfwGetTime();

    static std::vector<SapEdge> edges;
    static std::vector<int> active;
    static std::vector<AABB> boxes;

    static std::vector<Tri*> collisionsTris;
    static std::vector<RigidBodyHandle> collisionsBodies;

    const int bodyCount = static_cast<int>(awakeHandles.size());

    if (bodyCount == 0) {
        debugResultSweep.push_back((glfwGetTime() - start) * 1000.0);
        return;
    }

    boxes.resize(bodyCount);

    auto refreshBoxes = [&]() {
        for (int i = 0; i < bodyCount; ++i) {
            RigidBody& rb = *caches->bodies.get(awakeHandles[i], FUNC_NAME);
            boxes[i] = rb.aabb;
        }
        };

    refreshBoxes();

    const int axis = chooseLargestExtentAxis(boxes);

    edges.clear();
    edges.reserve(bodyCount * 2);

    for (int i = 0; i < bodyCount; ++i) {
        edges.push_back(SapEdge{
            .value = boxes[i].worldMin[axis],
            .bodyIdx = i,
            .isMin = true
            });

        edges.push_back(SapEdge{
            .value = boxes[i].worldMax[axis],
            .bodyIdx = i,
            .isMin = false
            });
    }

    std::sort(edges.begin(), edges.end(), sapEdgeLess);

    for (int step = 0; step < substeps; ++step) {
        pairsBufTerrain.clear();
        pairsBufDynamic.clear();

        collisionsTris.clear();
        collisionsBodies.clear();

        // In real substeps bodies move between substeps.
        // For benchmarken kan detta vara nästan gratis om inget rör sig,
        // men insertion sort är ändå rätt modell för SAP.
        if (step > 0) {
            refreshBoxes();

            for (SapEdge& edge : edges) {
                const AABB& box = boxes[edge.bodyIdx];
                edge.value = edge.isMin
                    ? box.worldMin[axis]
                    : box.worldMax[axis];
            }

            insertionSortSapEdges(edges);
        }

        // Same terrain/asleep work as brute-force style.
        for (int i = 0; i < bodyCount; ++i) {
            const AABB& boxA = boxes[i];
            RigidBodyHandle bodyA = awakeHandles[i];

            collisionsTris.clear();
            terrainBvh.singleQuery(boxA, collisionsTris);

            for (Tri* tri : collisionsTris) {
                pairsBufTerrain.emplace_back(bodyA, tri);
            }

            collisionsBodies.clear();
            asleepBvh.singleQuery(boxA, collisionsBodies);

            for (RigidBodyHandle bodyB : collisionsBodies) {
                pairsBufDynamic.emplace_back(bodyA, bodyB);
            }
        }

        // Sweep dynamic-vs-dynamic inside awake island.
        active.clear();

        for (const SapEdge& edge : edges) {
            const int aIdx = edge.bodyIdx;

            if (edge.isMin) {
                const AABB& boxA = boxes[aIdx];

                for (int bIdx : active) {
                    const AABB& boxB = boxes[bIdx];

                    // Full AABB test. SAP only guarantees overlap on selected axis.
                    if (boxA.intersects(boxB)) {
                        pairsBufDynamic.emplace_back(
                            awakeHandles[aIdx],
                            awakeHandles[bIdx]
                        );
                    }
                }

                active.push_back(aIdx);
            }
            else {
                removeActiveIndex(active, aIdx);
            }
        }
    }

    double end = glfwGetTime();
    debugResultSweep.push_back((end - start) * 1000.0);
}

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

    terrainPairs.clear();
    dynamicPairs.clear();

    pairsBufDynamic.clear();
    pairsBufTerrain.clear();
}

//==================================================
//      Update BVHs
//==================================================
void BroadphaseManager::updateBVHs() {
    awakeBvh.update(awakeHandles);

    if (asleepBvh.dirty) {
        asleepBvh.update(asleepHandles);
        asleepBvh.dirty = false;
    }

    if (staticBvh.dirty) {
        staticBvh.build(staticHandles);
        staticBvh.dirty = false;
    }

    //std::cout << awakeBvh.nodes.size() << " awake nodes, "
    //          << asleepBvh.nodes.size() << " asleep nodes, "
    //    << staticBvh.nodes.size() << " static nodes.\n";
}

//==================================================
//     Broadphase queries
//==================================================
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
        //treeVsTreeQuery(awakeBvh, awakeBvh, pairsBufDynamic);
        treeVsSameTreeQuery(awakeBvh, pairsBufDynamic);

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