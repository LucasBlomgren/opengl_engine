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
    void init(std::vector<GameObject>* gameObjects, std::vector<Tri>* terrainTris);

    // update BVHs if dirty
    void updateBVHs();

    // compute pairs for this frame and get results
    void computePairs();
    const std::vector<TerrainPair>& getTerrainPairs() const { return terrainPairs; }
    const std::vector<DynamicPair>& getDynamicPairs() const { return dynamicPairs; }

    // add/remove from current list
    void add(GameObject& obj, BroadphaseBucket dst);
    void remove(GameObject& obj);

    // move between lists
    void moveToAwake(GameObject& obj);
    void moveToAsleep(GameObject& obj);
    void moveToStatic(GameObject& obj);

    // get lists of indices
    const std::vector<int>& getAwakeList()  const { return awakeIds; }
    const std::vector<int>& getAsleepList() const { return asleepIds; }
    const std::vector<int>& getStaticList() const { return staticIds; }

    // get bvhs
    const BVHTree& getAwakeBVH()  const { return awakeBvh; }
    const BVHTree& getAsleepBVH() const { return asleepBvh; }
    const BVHTree& getStaticBVH() const { return staticBvh; }
    const TerrainBVH& getTerrainBVH() const { return terrainBvh; }

private:
    // reference to all game objects
    std::vector<GameObject>* dynamicObjects = nullptr;
    std::vector<Tri>* terrainTriangles = nullptr;

    // lists of indices into dynamicObjects
    std::vector<int> awakeIds;
    std::vector<int> asleepIds;
    std::vector<int> staticIds;

    // trees 
    BVHTree awakeBvh;
    BVHTree asleepBvh;
    BVHTree staticBvh;
    TerrainBVH terrainBvh;

    // computePairs results
    std::vector<TerrainPair> terrainPairs;
    std::vector<DynamicPair> dynamicPairs;

    // pairs buffers
    std::vector<std::pair<GameObject*, Tri*>> pairsBufTerrain;
    std::vector<std::pair<GameObject*, GameObject*>> pairsBufDynamic;

    // helpers
    void swapAndPop(GameObject& obj, std::vector<int>& list);
    BVHTree& bvhFor(BroadphaseBucket b);
    std::vector<int>& listFor(BroadphaseBucket b);
};