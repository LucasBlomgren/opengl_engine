#pragma once
#include <vector>

#include "core/slot_map.h"
#include "../physics_step_types.h"
#include "bvh/bvh.h"
#include "bvh/bvh_terrain.h"
#include "rigidbody_broadphase_types.h"
#include "broadphase_types.h"

class RigidBody;
class Tri;

class BroadphaseManager {
public:
    void init(PhysicsWorld* world, RuntimeCaches* caches, std::vector<Tri>* terrainTris);
    void clear();

    // update BVHs if dirty
    void updateBVHs();

    // compute pairs for this frame and get results
    void buildPairs(const StepScope& scope, PairBatch& batch);

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
    TerrainBVH terrainBvh;

    // pairs buffers
    std::vector<std::pair<RigidBodyHandle, Tri*>> pairsBufTerrain;
    std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>> pairsBufDynamic;

    // helpers
    void swapAndPop(RigidBodyHandle& handle, std::vector<RigidBodyHandle>& list);
    BVHTree& bvhFor(BroadphaseBucket b);
    std::vector<RigidBodyHandle>& listFor(BroadphaseBucket b);
};