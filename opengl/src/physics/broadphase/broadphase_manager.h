#pragma once
#include <vector>

#include "physics.h"
#include "bvh/bvh.h"
#include "bvh/bvh_terrain.h"
#include "bvh/treetree_query.h"
#include "broadphase_types.h"
#include "broadphase_pairs.h"

class RigidBody;
struct Tri;

class BroadphaseManager {
public:
    void init(PhysicsWorld* world, RuntimeCaches* caches, std::vector<Tri>* terrainTris);

    // update BVHs if dirty
    void updateBVHs();

    // compute pairs for this frame and get results
    void computePairs();
    const std::vector<TerrainPair>& getTerrainPairs() const { return terrainPairs; }
    const std::vector<DynamicPair>& getDynamicPairs() const { return dynamicPairs; }

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
    TerrainBVH terrainBvh;

    // computePairs results
    std::vector<TerrainPair> terrainPairs;
    std::vector<DynamicPair> dynamicPairs;

    // pairs buffers
    std::vector<std::pair<RigidBodyHandle, Tri*>> pairsBufTerrain;
    std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>> pairsBufDynamic;

    // helpers
    void swapAndPop(RigidBodyHandle& handle, std::vector<RigidBodyHandle>& list);
    BVHTree& bvhFor(BroadphaseBucket b);
    std::vector<RigidBodyHandle>& listFor(BroadphaseBucket b);
};