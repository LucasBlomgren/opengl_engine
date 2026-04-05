#pragma once
#include <vector>

struct Tri;

struct TerrainPair {
    RigidBodyHandle body;
    std::vector<Tri*> tris;
};
struct DynamicPair {
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
};