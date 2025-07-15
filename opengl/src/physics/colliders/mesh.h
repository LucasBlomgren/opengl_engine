#pragma once
#include <vector>
#include "tri.h"
#include "bvh.h"

class Mesh {
public:
    std::vector<Tri> tris;
    BVHTree<Tri> bvh;

    Mesh(const std::vector<Tri>& t)
        : tris(t)
    {
        bvh.build(tris);
    }
};