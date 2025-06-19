#pragma once
#include <vector>
#include "tri.h"
#include "bvh.h"

class TriMesh {
public:
    std::vector<Tri> tris;
    BVHTree<Tri> bvh;

    TriMesh(const std::vector<Tri>& t)
        : tris(t)
    {
        bvh.build(tris);
    }

    void updateBvh() {
        bvh.update(tris);
    }
};