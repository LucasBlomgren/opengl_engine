#pragma once
#include "colliders/aabb.h"

class Tri;

class TerrainBVH {
public:
    using Element = Tri*;
    int rootIdx = -1;

    // public för att rendera AABBs
    struct Node {
        bool alive = true;
        bool isLeaf = false;
        int childAIdx = -1;
        int childBIdx = -1;
        int   start;
        int   count;
        bool  dirty;

        Element element;
        AABB  tightBox;     // endast för lövnoder
        AABB  fatBox;       // för alla noder
    };
    std::vector<Node> nodes;

    // tree vs tree query
    static constexpr int MaxStackSize = 256;
    static constexpr int MaxCollisionBuf = 25000;

    void build(std::vector<Tri>& tris);
    void singleQuery(const AABB& qBox, std::vector<Tri*>& out) const;

private:
    struct BVHPrimitive {
        glm::vec3 min;
        glm::vec3 max;
        glm::vec3 centroid;
        Element element;
    };

    int leafThreshold = 1;
    glm::vec3 fatBoxMargin{ 0.2f };

    std::vector<BVHPrimitive> prims;
    void createPrimitives(std::vector<Tri>& tris);

    void split(int parentIdx, int depth);
    void initChild(int parentIdx, int nodeIdx, bool isLeft, int start, int end, int count);
    void makeLeaf(int leafIdx);
    void refitNode(int nodeIdx);
    void updateRenderData(Node& n);
};