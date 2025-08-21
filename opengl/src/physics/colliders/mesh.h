#pragma once
#include <vector>
#include "tri.h"
#include "bvh.h"

class Mesh {
public:
    std::vector<Tri> tris;
    std::vector<int> indexes; // optional, for indexed meshes
    BVHTree<Tri> bvh;

    Mesh(const std::vector<Tri>& t)
        : tris(t)
    {
        bvh.build(tris, indexes);
    }
};