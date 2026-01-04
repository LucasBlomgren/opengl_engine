#pragma once
#include <vector>

class GameObject;
struct Tri;

struct TerrainPair {
    GameObject* obj;
    std::vector<Tri*> tris;
};
struct DynamicPair {
    GameObject* A;
    GameObject* B;
};