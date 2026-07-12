#pragma once
#include <vector>
#include <slot_map.h>

class Tri;

struct TerrainPair {
    RigidBodyHandle body;
    std::vector<Tri*> tris;
};
struct DynamicPair {
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
};

struct PairBatch {
    std::vector<DynamicPair> dynamicPairs;
    std::vector<TerrainPair> terrainPairs;

    std::vector<DynamicPair> speculativeDynamicPairs;
    std::vector<TerrainPair> speculativeTerrainPairs;

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