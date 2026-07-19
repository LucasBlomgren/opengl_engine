#pragma once
#include <vector>

#include "core/slot_map.h"
#include "../runtime_caches.h"
#include "bvh/bvh.h"
#include "bvh/bvh_terrain.h"
#include "rigidbody_broadphase_types.h"
#include "broadphase_types.h"
#include "broadphase/sweep_and_prune.h"

class RigidBody;
class Tri;

class BroadphaseManager {
public:
    void init(PhysicsWorld* world, RuntimeCaches* caches, std::vector<Tri>* terrainTris);
    void clear();

    // update BVHs if dirty
    void updateBVHs();

    //   Build pairs for narrowphase
    void buildPairs(PairBatch& batch);
    void buildPairsBruteForce(
        const std::vector<RigidBodyHandle>& bodies,
        std::vector<DynamicPair>& outPairs
    );

    void buildSpeculativePairs(
        float dt, 
        PairBatch& batch,
        std::vector<AABB>& debugSweeps
    );
    void determineSpeculativeBodies(
        float dt, 
        std::vector<RigidBodyHandle>& outBodies, 
        std::vector<AABB>& outAABBs
    );

    // add/remove from current list
    void add(RigidBodyHandle& handle, BroadphaseBucket dst);
    void remove(RigidBodyHandle& handle);

    // move between lists
    void moveToAwake(RigidBodyHandle& handle);
    void moveToAsleep(RigidBodyHandle& handle);
    void moveToStatic(RigidBodyHandle& handle);

    void setBVHDirty(RigidBodyHandle& handle);

    // get lists of indices
    const std::vector<RigidBodyHandle>& getAwakeList()  const { return awakeHandles; }
    const std::vector<RigidBodyHandle>& getAsleepList() const { return asleepHandles; }
    const std::vector<RigidBodyHandle>& getStaticList() const { return staticHandles; }

    // get bvhs
    const BVHTree& getAwakeBVH()  const { return awakeBvh; }
    const BVHTree& getAsleepBVH() const { return asleepBvh; }
    const BVHTree& getStaticBVH() const { return staticBvh; }
    const TerrainBVH& getTerrainBVH() const { return terrainBvh; }

    void updateBVHRenderData(const BVHType& type, bool update);

private:
    // references to pointer caches and terrain triangles
    RuntimeCaches* caches = nullptr;
    std::vector<Tri>* terrainTriangles = nullptr;

    // lists of indices into dynamicObjects
    std::vector<RigidBodyHandle> awakeHandles;
    std::vector<RigidBodyHandle> asleepHandles;
    std::vector<RigidBodyHandle> staticHandles;

    // trees 
    BVHTree awakeBvh;
    BVHTree asleepBvh;
    BVHTree staticBvh;
    BVHTree speculativeBvh;
    TerrainBVH terrainBvh;

    sap::SweepAndPrune sap;

    // pairs buffers
    std::vector<std::pair<RigidBodyHandle, Tri*>> pairsBufTerrain;
    std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>> pairsBufDynamic;

    // helpers
    void swapAndPop(RigidBodyHandle& handle, std::vector<RigidBodyHandle>& list);
    BVHTree& bvhFor(BroadphaseBucket b);
    std::vector<RigidBodyHandle>& listFor(BroadphaseBucket b);
};