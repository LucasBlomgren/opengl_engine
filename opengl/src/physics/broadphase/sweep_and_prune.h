#pragma once

#include <vector>

#include "runtime_caches.h"
#include "rigid_body.h"
#include "colliders/aabb.h"
#include "bvh/bvh_terrain.h"
#include "broadphase_pairs.h"

namespace sap {

    struct SapItem {
        RigidBodyHandle handle;
        AABB box;
    };

    void buildItems(
        RuntimeCaches* caches,
        const std::vector<RigidBodyHandle>& handles,
        std::vector<SapItem>& out
    );

    void querySameSet(
        const std::vector<SapItem>& items,
        std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>>& out
    );

    void queryTwoSets(
        const std::vector<SapItem>& aItems,
        const std::vector<SapItem>& bItems,
        std::vector<std::pair<RigidBodyHandle, RigidBodyHandle>>& out
    );

    void queryTerrain(
        const TerrainBVH& terrainBvh,
        const std::vector<SapItem>& dynamicItems,
        std::vector<TerrainPair>& out
    );

    void computePairs(
        RuntimeCaches* caches,
        const TerrainBVH& terrainBvh,
        const std::vector<RigidBodyHandle>& awakeHandles,
        const std::vector<RigidBodyHandle>& asleepHandles,
        const std::vector<RigidBodyHandle>& staticHandles,
        std::vector<TerrainPair>& terrainPairs,
        std::vector<DynamicPair>& dynamicPairs
    );

}