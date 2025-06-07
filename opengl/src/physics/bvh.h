#pragma once
#include <glm/glm.hpp>
#include <unordered_set>
#include <algorithm>

#include "game_object.h"
#include "aabb.h"
#include "aabb_renderer.h"

// kan ta in två olika element (Collider eller Tri)
template<typename E>
class BVHTree {
    int minOverlapDepth              = 6;
    float updateInterval             = 120.0f;
    int   leafThreshold              = 1;
    int   numRefits                  = 0;
    int   numIterationsSinceRebuild  = 0;
    int   rebuildThreshold           = 0;        // räknas om i build()
    int   minRebuildThreshold        = 5;        // min 10 refits innan rebuild
    float rebuildRatio               = 0.20f;    // 40 % av lövkorrektioner → rebuild
    glm::vec3 fatBoxMargin { 0.2f };

    struct BVHPrimitive {
        glm::vec3 min;
        glm::vec3 max;
        glm::vec3 centroid;
        E* element;
    };

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

    struct CollisionPair {
        E* A;
        E* B;
    };

public:
    Node* root;
    std::vector<Node> nodes;

    void build(std::vector<E>& elements);
    // void build(std::vector<GameObject>& objects);

    void update(std::vector<E>& elements);
    int treeVsTreeQuery(BVHTree<E>& bvhA, BVHTree<E>& bvhB);
    int singleQuery(AABB& qBox);

    std::vector<E*> collisions;

    // debug
    int numRebuilds = 0;
    CollisionPair* collisionBuf = new CollisionPair[MaxCollisionBuf];

private:
    std::vector<BVHPrimitive> prims;
    void createPrimitives(std::vector<E>& elements);
    void makeLeaf(Node& parent);
    void initChild(Node& parent, Node* node, bool isLeft, int start, int end, int count);
    int chooseAxisByMinOverlap(int start, int count, const AABB& fatBox);
    void split(Node& parent, int depth);
    void updateLeaves();
    void refitNode(Node* node);

    // single query
    std::vector<Node*> queryStack;

    // tree vs tree query
    static constexpr int MaxStackSize = 1 << 14;
    static constexpr int MaxCollisionBuf = 1 << 16;
    NodePair* nodeStack = new NodePair[MaxStackSize];
};