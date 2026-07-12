#pragma once

#include "../runtime_caches.h"
#include "physics_world.h"
#include "rigidbody.h"
#include "colliders/aabb.h"

// #TODO: optimize nodes for cache size:
// Het array — bara det traverseringen rör. Indexeras parallellt med cold[].
//struct BVHNodeHot {
//    float fatMin[3];   // 12
//    float fatMax[3];   // 12
//    int   childA;      // 4   (-1 => leaf)
//    int   childB;      // 4
//    RigidBodyHandle element; // 8  (giltig bara för löv)
//};                      // 40 B → ryms i en cache-line, ~1,5 nod/line
//
//// Kall array — allt övrigt, rörs bara i build/update/refit, aldrig i query.
//struct BVHNodeCold {
//    AABB tightBox;
//    int parentIdx, start, count, selfIdx;
//    bool alive, dirty;
//    // local-koordinater etc.
//};

enum class BVHType {
    Awake,
    Asleep,
    Static
};

class BVHTree {
public:
    BVHTree() = default;

    // #rigidbody vector: bvh should use body handles instead of collider to work with compound colliders
    using Element = RigidBodyHandle;
    void init(PhysicsWorld* world, RuntimeCaches* caches, size_t allocSize);
    void clear();

    bool dirty = false;
    int rootIdx = -1;
    bool shouldUpdateRenderData = false;

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

        Element element;
        AABB  tightBox;     // only for leaf nodes
        AABB  fatBox;       // for all nodes
    };
    std::vector<Node> nodes;

    // tree vs tree query
    static constexpr int MaxStackSize = 512;
    static constexpr int MaxCollisionBuf = 25000;

    void build(std::vector<RigidBodyHandle>& objects);
    void update(std::vector<RigidBodyHandle>& objects);
    void singleQuery(const AABB& qBox, std::vector<RigidBodyHandle>& out) const;
    bool queryAny(const AABB& qBox, RigidBodyHandle ignoreBody) const;

    int insertLeaf(RigidBodyHandle handle);
    int findBestSibling(AABB& box);
    int createLeaf(RigidBodyHandle handle, RigidBody* body);

    void removeLeaf(int leafIdx);
    void refitParents(int leafIdx);

    template<class Func>
    void forEachLeafElement(const Node& leaf, Func&& fn) const {
        fn(leaf.element, leaf.tightBox);
    }

private:
    PhysicsWorld* world = nullptr;
    RuntimeCaches* caches = nullptr;

    int rebuildCooldown = 10;
    int rebuildCooldownCounter = 0;
    int numRefits = 0;
    int rebuildThreshold = 0; // recalculated in build() as log2(n) * rebuildRatio

    // settings
    const int   leafThreshold = 1;
    const int   minRebuildThreshold = 5;      // min refits before rebuild, to avoid rebuilding too early when n is small
    const float rebuildRatio = 0.40f;         // % of leaf corrections before rebuild
    const glm::vec3 fatBoxMargin{ 0.2f };

    struct BVHPrimitive {
        glm::vec3 min{ 0.0f };
        glm::vec3 max{ 0.0f };
        glm::vec3 centroid{ 0.0f };
        Element element;
    };
    std::vector<BVHPrimitive> prims;

    void initChild(int parentIdx, int nodeIdx, bool isLeft, int start, int end);
    void createPrimitives(std::vector<RigidBodyHandle>& objectHandles);
    void makeLeaf(int leafIdx);
    void split(int parentIdx, int depth);
    void updateLeaves();
    void refitNode(int nodeIdx);
    void updateRenderData(Node& n);
};