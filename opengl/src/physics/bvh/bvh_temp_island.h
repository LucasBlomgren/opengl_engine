#pragma once

#include "runtime_caches.h"
#include "colliders/aabb.h"

class TempIslandBVH {
public:
    using Element = RigidBodyHandle;
    void init(RuntimeCaches* caches, int allocSize);

    bool dirty = false;
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

    void build(std::vector<RigidBodyHandle>& objects);
    void update(std::vector<RigidBodyHandle>& objects);
    void singleQuery(const AABB& qBox, std::vector<RigidBodyHandle>& out) const;

    static constexpr int MaxStackSize = 256;
    static constexpr int MaxCollisionBuf = 25000;

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
    RuntimeCaches* caches = nullptr;

    // settings
    const int   leafThreshold = 8;
    const glm::vec3 fatBoxMargin{ 0.2f };

    struct BVHPrimitive {
        AABB box;
        Element element;
    };
    std::vector<BVHPrimitive> prims;

    void initChild(int parentIdx, int nodeIdx, bool isLeft, int start, int end);
    void createPrimitives(std::vector<RigidBodyHandle>& objectHandles);
    void makeLeaf(int leafIdx);
    void split(int parentIdx, int depth);
    void updateLeaves();
    void refitNode(int nodeIdx);
};