#pragma once
#include <vector>

#include "bvh.h"
#include "broadphase_types.h"
#include "broadphase_pairs.h"

class GameObject;  // forward declare
struct Tri;        // forward declare

class BroadphaseManager {
    // reference to all game objects
    std::vector<GameObject>* dynamicObjects = nullptr;
    std::vector<Tri>* terrainTriangles = nullptr;

    // trees 
    BVHTree<GameObject> awakeBvh;
    BVHTree<GameObject> asleepBvh;
    BVHTree<GameObject> staticBvh;
    BVHTree<Tri> terrainBvh;

    // lists of indices into dynamicObjects
    std::vector<int> awakeIds;
    std::vector<int> asleepIds;
    std::vector<int> staticIds;

    // results
    std::vector<TerrainPair> terrainPairs;
    std::vector<DynamicPair> dynamicPairs;

    // pairs buffers
    std::vector<std::pair<GameObject*, Tri*>> pairsBufTerrain;
    std::vector<std::pair<GameObject*, GameObject*>> pairsBufDynamic;

    // helpers
    void swapAndPop(GameObject& obj, std::vector<int>& list);
    BVHTree<GameObject>& bvhFor(BroadphaseBucket b);
    std::vector<int>& listFor(BroadphaseBucket b);

public:
    BroadphaseManager() = default;
    void init(std::vector<GameObject>* gameObjects, std::vector<Tri>* terrainTris);

    // update BVHs if dirty
    void updateBVHs();

    // compute pairs for this frame
    void computePairs();
    // get results of last computation
    const std::vector<TerrainPair>& getTerrainPairs() const { return terrainPairs; }
    const std::vector<DynamicPair>& getDynamicPairs() const { return dynamicPairs; }

    // remove from current list
    void add(GameObject& obj, BroadphaseBucket dst);
    void remove(GameObject& obj);

    // move between lists
    void moveToAwake(GameObject& obj);
    void moveToAsleep(GameObject& obj);
    void moveToStatic(GameObject& obj);

    // get lists of indices
    const std::vector<int>& awakeList()  const { return awakeIds; }
    const std::vector<int>& asleepList() const { return asleepIds; }
    const std::vector<int>& staticList() const { return staticIds; }

    // get bvhs
    const BVHTree<GameObject>& getAwakeBVH()  const { return awakeBvh; }
    const BVHTree<GameObject>& getAsleepBVH() const { return asleepBvh; }
    const BVHTree<GameObject>& getStaticBVH() const { return staticBvh; }
    const BVHTree<Tri>& getTerrainBVH() const { return terrainBvh; }
};