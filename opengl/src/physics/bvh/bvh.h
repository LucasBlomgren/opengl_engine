#pragma once

#include "physics/world/physics_world.h"
#include "physics/bodies/rigidbody.h"
#include "physics/colliders/aabb.h"

namespace physics::internal {

// #TODO: optimize nodes for cache size:
// Het array — bara det traverseringen rör. Indexeras parallellt med cold[].
//struct BVHNodeHot {
//    float fatMin[3];   // 12
//    float fatMax[3];   // 12
//    int   childA;      // 4   (-1 => leaf)
//    int   childB;      // 4
//    BodyHandle element; // 8  (giltig bara för löv)
//};                      // 40 B → ryms i en cache-line, ~1,5 nod/line
//
//// Kall array — allt övrigt, rörs bara i build/update/refit, aldrig i query.
//struct BVHNodeCold {
//    AABB tightBox;
//    int parentIdx, start, count, selfIdx;
//    bool alive, dirty;
//    // local-koordinater etc.
//};

class BVHTree {
public:
    BVHTree() = default;

    // #rigidbody vector: bvh should use body handles instead 
    // of collider to work with compound colliders
    using Element = BodyHandle;
    void init(
        PhysicsWorld* world, 
        size_t allocSize, 
        bool writeBodyLeafIndices
    );
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

    void build(const std::vector<BodyHandle>& objects);

    // for speculative pairs, using speculative AABBs
    void build(
        const std::vector<BodyHandle>& handles,
        const std::vector<AABB>& boxes
    );

    void createPrimitivesFromBodyAABBs(const std::vector<BodyHandle>& handles);
    void createPrimitivesFromExternalAABBs(
        const std::vector<BodyHandle>& handles, 
        const std::vector<AABB>& boxes
    );
    void buildFromPrimitives();

    void update(std::vector<BodyHandle>& objects);
    void singleQuery(const AABB& qBox, std::vector<BodyHandle>& out) const;
    bool queryAny(const AABB& qBox, BodyHandle ignoreBody) const;

    int insertLeaf(BodyHandle handle);
    int findBestSibling(AABB& box);
    int createLeaf(BodyHandle handle, RigidBody& body);

    void removeLeaf(int leafIdx);
    void refitParents(int leafIdx);

    template<class Func>
    void forEachLeafElement(const Node& leaf, Func&& fn) const {
        fn(leaf.element, leaf.tightBox);
    }

private:
    PhysicsWorld* world = nullptr;

    bool writeBodyLeafIndices = true;

    int rebuildCooldown = 5;
    int rebuildCooldownCounter = 0;
    int numRefits = 0;
    int rebuildThreshold = 0; // recalculated in build() as log2(n) * rebuildRatio

    // settings
    const int   leafThreshold = 1;

    // min refits before rebuild, to avoid rebuilding too early when n is small
    const int   minRebuildThreshold = 5;

    const float rebuildRatio = 0.40f; // % of leaf corrections before rebuild
    const glm::vec3 fatBoxMargin{ 0.2f };

    struct BVHPrimitive {
        glm::vec3 min{ 0.0f };
        glm::vec3 max{ 0.0f };
        glm::vec3 centroid{ 0.0f };
        Element element;
    };
    std::vector<BVHPrimitive> prims;

    void initChild(int parentIdx, int nodeIdx, bool isLeft, int start, int end);
    void createPrimitives(std::vector<BodyHandle>& objectHandles);
    void makeLeaf(int leafIdx);
    void split(int parentIdx, int depth);
    void updateLeaves();
    void refitNode(int nodeIdx);
    void updateRenderData(Node& n);
};


}
