#pragma once

#include <vector>

#include "rigidbody_types.h"
#include "contact_types.h"

#include "physics/bvh/bvh.h"
#include "physics/bvh/bvh_terrain.h"

#include "physics/public/debug_types.h"

namespace physics::internal {

class RigidBody;
class Tri;

class BroadphaseManager {
public:
    void init(
        PhysicsWorld* world, 
        std::vector<Tri>* terrainTris
    );
    void clear();

    // update BVHs if dirty
    void updateBVHs();

    //   Build pairs for narrowphase
    void buildPairs(PairBatch& batch);

    void buildSpeculativePairs(
        float dt,
        PairBatch& batch,
        std::vector<AABB>& debugSweeps
    );
    void determineSpeculativeBodies(
        float dt,
        std::vector<BodyHandle>& outBodies,
        std::vector<AABB>& outAABBs
    );

    // add/remove from current list
    void add(const BodyHandle& handle, BroadphaseBucket dst);
    void remove(const BodyHandle& handle);

    // move between lists
    void moveToAwake(const BodyHandle& handle);
    void moveToAsleep(const BodyHandle& handle);
    void moveToStatic(const BodyHandle& handle);

    void setBVHDirty(const BodyHandle& handle);

    // get lists of indices
    const std::vector<BodyHandle>& getAwakeList()  const { return awakeHandles; }
    const std::vector<BodyHandle>& getAsleepList() const { return asleepHandles; }
    const std::vector<BodyHandle>& getStaticList() const { return staticHandles; }

    // get bvhs
    const BVHTree& getAwakeBVH()  const { return awakeBvh; }
    const BVHTree& getAsleepBVH() const { return asleepBvh; }
    const BVHTree& getStaticBVH() const { return staticBvh; }
    const TerrainBVH& getTerrainBVH() const { return terrainBvh; }

    void updateBVHRenderData(const physics::debug::BvhType& type, bool update);

private:
    // references to world and terrain
    PhysicsWorld* world = nullptr;
    std::vector<Tri>* terrainTriangles = nullptr;

    // lists of handles
    std::vector<BodyHandle> awakeHandles;
    std::vector<BodyHandle> asleepHandles;
    std::vector<BodyHandle> staticHandles;

    // trees
    BVHTree awakeBvh;
    BVHTree asleepBvh;
    BVHTree staticBvh;
    BVHTree speculativeBvh;
    TerrainBVH terrainBvh;

    // pairs buffers
    std::vector<std::pair<BodyHandle, Tri*>> pairsBufTerrain;
    std::vector<std::pair<BodyHandle, BodyHandle>> pairsBufDynamic;

    // helpers
    void swapAndPop(const BodyHandle& handle, std::vector<BodyHandle>& list);
    BVHTree& bvhFor(BroadphaseBucket b);
    std::vector<BodyHandle>& listFor(BroadphaseBucket b);
};

}
