#pragma once
#include <vector>

#include "pointer_cache.h"
#include "bvh/bvh.h"
#include "bvh/bvh_terrain.h"
#include "bvh/treetree_query.h"
#include "broadphase_types.h"
#include "broadphase_pairs.h"

class Collider;
struct Tri;

class BroadphaseManager {
public:
    void init(
        PointerCache<GameObject, GameObjectHandle>* gameObjectPtrCache,
        PointerCache<Collider, ColliderHandle>* colliderPtrCache,
        PointerCache<RigidBody, RigidBodyHandle>* bodyPtrCache,
        std::vector<Tri>* terrainTris
    );

    // update BVHs if dirty
    void updateBVHs();

    // compute pairs for this frame and get results
    void computePairs();
    const std::vector<TerrainPair>& getTerrainPairs() const { return terrainPairs; }
    const std::vector<DynamicPair>& getDynamicPairs() const { return dynamicPairs; }

    // add/remove from current list
    void add(ColliderHandle& handle, BroadphaseBucket dst);
    void remove(ColliderHandle& handle);

    // move between lists
    void moveToAwake(ColliderHandle& handle);
    void moveToAsleep(ColliderHandle& handle);
    void moveToStatic(ColliderHandle& handle);

    void setBVHDirty(ColliderHandle& handle);

    // get lists of indices
    const std::vector<ColliderHandle>& getAwakeList()  const { return awakeHandles; }
    const std::vector<ColliderHandle>& getAsleepList() const { return asleepHandles; }
    const std::vector<ColliderHandle>& getStaticList() const { return staticHandles; }

    // get bvhs
    const BVHTree& getAwakeBVH()  const { return awakeBvh; }
    const BVHTree& getAsleepBVH() const { return asleepBvh; }
    const BVHTree& getStaticBVH() const { return staticBvh; }
    const TerrainBVH& getTerrainBVH() const { return terrainBvh; }

private:
    // references to pointer caches and terrain triangles
    PointerCache<GameObject, GameObjectHandle>* gameObjectCache;
    PointerCache<Collider, ColliderHandle>* colliderCache;
    PointerCache<RigidBody, RigidBodyHandle>* bodyCache;
    std::vector<Tri>* terrainTriangles = nullptr;

    // lists of indices into dynamicObjects
    std::vector<ColliderHandle> awakeHandles;
    std::vector<ColliderHandle> asleepHandles;
    std::vector<ColliderHandle> staticHandles;

    // trees 
    BVHTree awakeBvh;
    BVHTree asleepBvh;
    BVHTree staticBvh;
    TerrainBVH terrainBvh;

    // computePairs results
    std::vector<TerrainPair> terrainPairs;
    std::vector<DynamicPair> dynamicPairs;

    // pairs buffers
    std::vector<std::pair<ColliderHandle, Tri*>> pairsBufTerrain;
    std::vector<std::pair<ColliderHandle, ColliderHandle>> pairsBufDynamic;

    // helpers
    void swapAndPop(ColliderHandle& handle, std::vector<ColliderHandle>& list);
    BVHTree& bvhFor(BroadphaseBucket b);
    std::vector<ColliderHandle>& listFor(BroadphaseBucket b);
};