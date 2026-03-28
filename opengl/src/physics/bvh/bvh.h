#pragma once
#include "colliders/aabb.h"
#include "slot_map.h"
#include "world.h"

class BVHTree {
public:
    BVHTree() = default;

    void init(SlotMap<Collider, ColliderHandle>* s, int allocSize);

    using Element = ColliderHandle;
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

//template<typename Ea, typename Eb>
//void treeVsTreeQuery(
//    const BVHTree<Ea>& a,
//    const BVHTree<Eb>& b,
//    std::vector<std::pair<Ea*, Eb*>>& out) noexcept
//{
//    if (a.nodes.empty() || b.nodes.empty() || a.rootIdx == -1 || b.rootIdx == -1) {
//        return;
//    }
//
//    out.clear();
//    // Rimlig gissning: lika många löv ungefär som noder/2
//    out.reserve((std::min)(a.nodes.size(), b.nodes.size()));
//
//    // Samma typ? (för att undvika dubletter när man jämför samma träd)
//    constexpr bool sameType = std::is_same_v<Ea, Eb>;
//    const bool sameTree = sameType && (static_cast<const void*>(&a) == static_cast<const void*>(&b));
//    const bool needOrderCheck = sameType && sameTree;
//
//    struct Entry { int ai; int bi; };
//    constexpr int MaxStack =
//        (BVHTree<Ea>::MaxStackSize > BVHTree<Eb>::MaxStackSize)
//        ? BVHTree<Ea>::MaxStackSize : BVHTree<Eb>::MaxStackSize;
//
//    Entry stack[MaxStack];
//    int sp = 0;
//    stack[sp++] = { a.rootIdx, b.rootIdx };
//
//    auto sah2 = [](const AABB& box) {
//        // billig SAH-proxy: 2*(xy + yz + zx), konstantfaktorn spelar ingen roll
//        const glm::vec3 e = box.wMax - box.wMin;
//        const float xy = e.x * e.y, yz = e.y * e.z, zx = e.z * e.x;
//        return xy + yz + zx;
//        };
//
//    while (sp) {
//        const auto [ai, bi] = stack[--sp];
//        const auto& nA = a.nodes[ai];
//        const auto& nB = b.nodes[bi];
//
//        // leaf–leaf
//        if (nA.isLeaf && nB.isLeaf) {
//            if (nA.tightBox.intersects(nB.tightBox)) {
//                if constexpr (sameType) {
//                    if (!needOrderCheck || ai < bi) {
//                        out.emplace_back(nA.element, nB.element);
//                    }
//                }
//                else {
//                    out.emplace_back(nA.element, nB.element);
//                }
//            }
//            continue;
//        }
//        // node–node eller node–leaf
//        else {
//            if (!nA.fatBox.intersects(nB.fatBox)) continue;
//        }
//
//        // expandera den sida som är "större" (billig SAH-proxy) – push:a bara 2 par
//        const bool expandA = !nA.isLeaf && (nB.isLeaf || sah2(nA.fatBox) >= sah2(nB.fatBox));
//        if (expandA) {
//            const int a0 = nA.childAIdx, a1 = nA.childBIdx;
//            // förpush-prune på barnens fatBox
//            if (a0 != -1 && a.nodes[a0].fatBox.intersects(nB.fatBox)) {
//                if (sp < MaxStack) stack[sp++] = { a0, bi };
//            }
//            if (a1 != -1 && a.nodes[a1].fatBox.intersects(nB.fatBox)) {
//                if (sp < MaxStack) stack[sp++] = { a1, bi };
//            }
//        }
//        else {
//            const int b0 = nB.childAIdx, b1 = nB.childBIdx;
//            if (b0 != -1 && b.nodes[b0].fatBox.intersects(nA.fatBox)) {
//                if (sp < MaxStack) stack[sp++] = { ai, b0 };
//            }
//            if (b1 != -1 && b.nodes[b1].fatBox.intersects(nA.fatBox)) {
//                if (sp < MaxStack) stack[sp++] = { ai, b1 };
//            }
//        }
//    }
//}
//
