#pragma once
#include <vector>

struct Tri;

struct TerrainPair {
    GameObjectHandle objHandle;
    std::vector<Tri*> tris;
};
struct DynamicPair {
    GameObjectHandle A;
    GameObjectHandle B;
};