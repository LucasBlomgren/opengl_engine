#pragma once
#include "physics/colliders/aabb.h"

class Tri;

class TerrainBVH {
public:
    using Element = Tri*;
    int rootIdx = -1;

    // public för att rendera AABBs
    struct Node {
        bool alive = true;
        bool isLeaf = false;

        int selfIdx = -1;
        int parentIdx = -1;
        int childAIdx = -1;
        int childBIdx = -1;

        int   start = -1;
        int   count = -1;
        bool  dirty = false;
        AABB  fatBox;
    };
    std::vector<Node> nodes;

    // tree vs tree query
    static constexpr int MaxStackSize = 256;
    static constexpr int MaxCollisionBuf = 25000;

    void build(std::vector<Tri>& tris);
    void singleQuery(const AABB& qBox, std::vector<Tri*>& out) const;
    bool queryAny(const AABB& qBox) const;

    template<class Func>
    void forEachLeafElement(const Node& leaf, Func&& fn) const
    {
        const int begin = leaf.start;
        const int end = leaf.start + leaf.count;

        for (int i = begin; i < end; ++i) {
            const BVHPrimitive& prim = prims[i];
            fn(prim.element, prim.box);
        }
    }

private:
    // settings
    const int   leafThreshold = 32;
    const glm::vec3 fatBoxMargin{ 0.2f };

    struct BVHPrimitive {
        AABB box;
        Element element;
    };
    std::vector<BVHPrimitive> prims;

    void initChild(int parentIdx, int nodeIdx, bool isLeft, int start, int end);
    void createPrimitives(std::vector<Tri>& tris);
    void makeLeaf(int leafIdx);
    void split(int parentIdx, int depth);
    void updateLeaves();
    void refitNode(int nodeIdx);
    void updateRenderData(Node& n);
};