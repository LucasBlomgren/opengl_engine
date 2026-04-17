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

        int   start = -1;
        int   count = -1;
        bool  dirty = false;

        Element element;
        AABB  tightBox;     // only for leaf nodes
        AABB  fatBox;       // for all nodes
    };
    std::vector<Node> nodes;

    // tree vs tree query
    static constexpr int MaxStackSize = 256;
    static constexpr int MaxCollisionBuf = 25000;

    void build(std::vector<Tri>& tris);
    void singleQuery(const AABB& qBox, std::vector<Tri*>& out) const;

private:
    struct BVHPrimitive {
        glm::vec3 min{ 0.0f };
        glm::vec3 max{ 0.0f };
        glm::vec3 centroid{ 0.0f };
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