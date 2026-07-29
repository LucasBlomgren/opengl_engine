#pragma once

#include <vector>

#include "physics/public/physics_handles.h"

namespace physics::internal {

class Tri;

struct TerrainPair {
    BodyHandle body;
    std::vector<Tri*> tris;
};
struct DynamicPair {
    BodyHandle bodyA;
    BodyHandle bodyB;
};

struct SpeculativeDynamicPair {
    BodyHandle bodyA;
    BodyHandle bodyB;
    BodyHandle sweepOwner;
};

struct SpeculativeTerrainPair {
    BodyHandle body;
    std::vector<Tri*> tris;
};

struct PairBatch {
    std::vector<DynamicPair> dynamicPairs;
    std::vector<TerrainPair> terrainPairs;

    std::vector<SpeculativeDynamicPair> speculativeDynamicPairs;
    std::vector<SpeculativeTerrainPair> speculativeTerrainPairs;

    void clear() {
        dynamicPairs.clear();
        terrainPairs.clear();

        speculativeDynamicPairs.clear();
        speculativeTerrainPairs.clear();
    }

    size_t totalDynamicPairs() const {
        return dynamicPairs.size() + speculativeDynamicPairs.size();
    }

    size_t totalTerrainPairs() const {
        return terrainPairs.size() + speculativeTerrainPairs.size();
    }
};

}
