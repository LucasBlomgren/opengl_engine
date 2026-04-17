#pragma once
#include <vector>

class Tri;

struct TerrainPair {
    RigidBodyHandle body;
    std::vector<Tri*> tris;
};
struct DynamicPair {
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
};