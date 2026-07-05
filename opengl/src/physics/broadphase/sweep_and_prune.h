#pragma once

#include <vector>

#include "substeps/physics_step_types.h"
#include "colliders/aabb.h"
#include "broadphase_types.h"

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
        std::vector<DynamicPair>& out
    );

    void queryTwoSets(
        const std::vector<SapItem>& aItems,
        const std::vector<SapItem>& bItems,
        std::vector<DynamicPair>& out
    );
}