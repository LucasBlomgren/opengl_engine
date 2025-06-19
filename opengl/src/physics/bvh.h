#pragma once
#include <glm/glm.hpp>
#include <unordered_set>
#include <algorithm>
#include <vector>

#include "aabb.h"
#include "aabb_renderer.h"

// kan ta in två olika element (Collider eller Tri)
template<typename E>
class BVHTree {
    float updateInterval             = 120.0f;
    int   leafThreshold              = 1;
    int   numRefits                  = 0;        // debug
    int   numIterationsSinceRebuild  = 0;
    int   rebuildThreshold           = 0;        // räknas om i build()
    int   minRebuildThreshold        = 5;        // min refits innan rebuild
    float rebuildRatio               = 0.20f;    // % av lövkorrektioner innan rebuild
    glm::vec3 fatBoxMargin { 0.2f };

    struct BVHPrimitive {
        glm::vec3 min;
        glm::vec3 max;
        glm::vec3 centroid;
        E* element;
    };

public:
    struct Node {
        bool isLeaf = false;
        AABB  tightBox;               // endast för lövnoder
        AABB  fatBox;                 // för alla noder
        Node* parent = nullptr;       // för att kunna gå uppåt i trädet
        Node* childA = nullptr;
        Node* childB = nullptr;
        int   start;
        int   count;
        bool  dirty;
        E* element;
        AABBRenderer aabbRenderer;
    };

    struct NodePair {
        Node* A;
        Node* B;
    };

    Node* root = nullptr;
    std::vector<Node> nodes;

    void build(std::vector<E>& elements);
    void update(std::vector<E>& elements);
    void singleQuery(const AABB& qBox, std::vector<E*>& out); 

    // debug
    int numRebuilds = 0;

    // tree vs tree query
    static constexpr int MaxStackSize = 64;
    static constexpr int MaxCollisionBuf = 25000;

private:
    std::vector<BVHPrimitive> prims;
    void initChild(Node& parent, Node* node, bool isLeft, int start, int end, int count);
    void createPrimitives(std::vector<E>& elements);
    void makeLeaf(Node& parent);
    void split(Node& parent, int depth);
    void updateLeaves();
    void refitNode(Node* node);

    // single query
    std::vector<Node*> queryStack;
};

template<typename Ea, typename Eb>
void treeVsTreeQuery(const BVHTree<Ea>& a, const BVHTree<Eb>& b, std::vector<std::pair<Ea*, Eb*>>& out)
{
    constexpr bool sameType = std::is_same_v<Ea, Eb>;
    constexpr int  MaxStack = BVHTree<Ea>::MaxStackSize;
    constexpr int  MaxBuf = BVHTree<Ea>::MaxCollisionBuf;

    // --- FIXED‐SIZE traversal‐stack på stacken (ingen heap‐allokering) ---
    std::pair<typename BVHTree<Ea>::Node*, typename BVHTree<Eb>::Node*> stack[MaxStack];
    int sp = 0;
    stack[sp++] = { a.root, b.root };

    out.clear();

    // Endast relevant när Ea==Eb
    bool sameTree = false;
    if constexpr (sameType) {
        sameTree = std::addressof(a) == std::addressof(b);
    }

    while (sp > 0) {
        auto [nA, nB] = stack[--sp];

        // Prune
        if (!nA->fatBox.intersects(nB->fatBox))
            continue;

        // Leaf–leaf
        if (nA->isLeaf && nB->isLeaf) {
            if (!nA->tightBox.intersects(nB->tightBox))
                continue;

            if constexpr (sameType) {
                if (!sameTree || nA->element < nB->element)
                    out.emplace_back(nA->element, nB->element);
            }
            else {
                out.emplace_back(nA->element, nB->element);
            }
            continue;
        }

        // Expand children, precis som förut
        if (nA->isLeaf) {
            if (nB->childA) stack[sp++] = { nA, nB->childA };
            if (nB->childB) stack[sp++] = { nA, nB->childB };
        }
        else if (nB->isLeaf) {
            if (nA->childA) stack[sp++] = { nA->childA, nB };
            if (nA->childB) stack[sp++] = { nA->childB, nB };
        }
        else {
            if (nA->childA && nB->childA) stack[sp++] = { nA->childA, nB->childA };
            if (nA->childA && nB->childB) stack[sp++] = { nA->childA, nB->childB };
            if (nA->childB && nB->childA) stack[sp++] = { nA->childB, nB->childA };
            if (nA->childB && nB->childB) stack[sp++] = { nA->childB, nB->childB };
        }
    }
}