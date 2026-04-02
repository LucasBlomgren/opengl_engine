#pragma once
#include "colliders/aabb.h"
#include "slot_map.h"
#include "world.h"

class BVHTree {
public:
    BVHTree() = default;
    using Element = ColliderHandle;

    void init(SlotMap<Collider, ColliderHandle>* s, int allocSize);

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

    void build(std::vector<ColliderHandle>& objects);
    void update(std::vector<ColliderHandle>& objects);
    void singleQuery(const AABB& qBox, std::vector<ColliderHandle>& out) const;

    int insertLeaf(ColliderHandle handle);
    int findBestSibling(AABB& box);
    int createLeaf(ColliderHandle handle, Collider* objPtr);

    void removeLeaf(int leafIdx);
    void refitParents(int leafIdx);

private:
    SlotMap<Collider, ColliderHandle>* slotMap;
    int   leafThreshold = 1;
    int   numRefits = 0;
    int   rebuildThreshold = 0;        // räknas om i build()
    int   minRebuildThreshold = 5;        // min refits innan rebuild
    float rebuildRatio = 0.40f;    // % av lövkorrektioner innan rebuild
    glm::vec3 fatBoxMargin{ 0.2f };

    struct BVHPrimitive {
        glm::vec3 min;
        glm::vec3 max;
        glm::vec3 centroid;
        Element element;
    };
    std::vector<BVHPrimitive> prims;

    void initChild(int parentIdx, int nodeIdx, bool isLeft, int start, int end, int count);
    void createPrimitives(std::vector<ColliderHandle>& objectHandles);
    void makeLeaf(int leafIdx);
    void split(int parentIdx, int depth);
    void updateLeaves();
    void refitNode(int nodeIdx);
    void updateRenderData(Node& n);
};