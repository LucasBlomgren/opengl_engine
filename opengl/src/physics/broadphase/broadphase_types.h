#pragma once

#include <vector>

#include "core/slot_map.h"

class Tri;

struct TerrainPair {
    RigidBodyHandle body;
    std::vector<Tri*> tris;
};
struct DynamicPair {
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
};

struct SpeculativeDynamicPair {
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
    RigidBodyHandle sweepOwner;
};

struct SpeculativeTerrainPair {
    RigidBodyHandle body;
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