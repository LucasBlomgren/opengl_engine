#pragma once

#include <vector>

#include "rigidbody_broadphase_types.h"
#include "broadphase_types.h"

#include "core/slot_map.h"

#include "physics/world/runtime_caches.h"
#include "physics/bvh/bvh.h"
#include "physics/bvh/bvh_terrain.h"

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
    void add(const RigidBodyHandle& handle, BroadphaseBucket dst);
    void remove(const RigidBodyHandle& handle);

    // move between lists
    void moveToAwake(const RigidBodyHandle& handle);
    void moveToAsleep(const RigidBodyHandle& handle);
    void moveToStatic(const RigidBodyHandle& handle);

    void setBVHDirty(const RigidBodyHandle& handle);

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

    // pairs buffers
    std::vector<std::pair<RigidBodyHandle, Tri*>> pairsBufTerrain;
    std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>> pairsBufDynamic;

    // helpers
    void swapAndPop(const RigidBodyHandle& handle, std::vector<RigidBodyHandle>& list);
    BVHTree& bvhFor(BroadphaseBucket b);
    std::vector<RigidBodyHandle>& listFor(BroadphaseBucket b);
};
