#pragma once
#include <vector>

#include "bvh/bvh.h"
#include "bvh/bvh_terrain.h"
#include "bvh/treetree_query.h"
#include "broadphase_types.h"
#include "broadphase_pairs.h"

class GameObject;
struct Tri;

class BroadphaseManager {
public:
    void init(SlotMap<GameObject, GameObjectHandle>* s, std::vector<Tri>* terrainTris);

    // update BVHs if dirty
    void updateBVHs();

    // compute pairs for this frame and get results
    void computePairs();
    const std::vector<TerrainPair>& getTerrainPairs() const { return terrainPairs; }
    const std::vector<DynamicPair>& getDynamicPairs() const { return dynamicPairs; }

    // add/remove from current list
    void add(GameObjectHandle& handle, BroadphaseBucket dst);
    void remove(GameObjectHandle& handle);

    // move between lists
    void moveToAwake(GameObjectHandle& handle);
    void moveToAsleep(GameObjectHandle& handle);
    void moveToStatic(GameObjectHandle& handle);

    // get lists of indices
    const std::vector<GameObjectHandle>& getAwakeList()  const { return awakeHandles; }
    const std::vector<GameObjectHandle>& getAsleepList() const { return asleepHandles; }
    const std::vector<GameObjectHandle>& getStaticList() const { return staticHandles; }

    // get bvhs
    const BVHTree& getAwakeBVH()  const { return awakeBvh; }
    const BVHTree& getAsleepBVH() const { return asleepBvh; }
    const BVHTree& getStaticBVH() const { return staticBvh; }
    const TerrainBVH& getTerrainBVH() const { return terrainBvh; }

private:
    // reference to all objects
    SlotMap<GameObject, GameObjectHandle>* slotMap = nullptr;
    std::vector<Tri>* terrainTriangles = nullptr;

    // lists of indices into dynamicObjects
    std::vector<GameObjectHandle> awakeHandles;
    std::vector<GameObjectHandle> asleepHandles;
    std::vector<GameObjectHandle> staticHandles;

    // trees 
    BVHTree awakeBvh;
    BVHTree asleepBvh;
    BVHTree staticBvh;
    TerrainBVH terrainBvh;

    // computePairs results
    std::vector<TerrainPair> terrainPairs;
    std::vector<DynamicPair> dynamicPairs;

    // pairs buffers
    std::vector<std::pair<GameObjectHandle, Tri*>> pairsBufTerrain;
    std::vector<std::pair<GameObjectHandle, GameObjectHandle>> pairsBufDynamic;

    // helpers
    void swapAndPop(GameObjectHandle& handle, std::vector<GameObjectHandle>& list);
    BVHTree& bvhFor(BroadphaseBucket b);
    std::vector<GameObjectHandle>& listFor(BroadphaseBucket b);
};