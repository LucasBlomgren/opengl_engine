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

    void clear() {
        dynamicPairs.clear();
        terrainPairs.clear();
    }
};