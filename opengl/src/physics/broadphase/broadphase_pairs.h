#pragma once
#include <vector>

struct Tri;

struct TerrainPair {
    ColliderHandle colliderHandle;
    std::vector<Tri*> tris;
};
struct DynamicPair {
    ColliderHandle A;
    ColliderHandle B;
};