#pragma once
#include <glm/glm.hpp>
#include "game_object.h"
#include "aabb.h"
#include "aabb_renderer.h"
#include <unordered_set>

struct BVHPrimitive {
    glm::vec3 min;      
    glm::vec3 max;      
    glm::vec3 centroid; 
    GameObject* object;
};

struct Node {
    GameObject* object;
    bool isLeaf = false;
    AABB  tightBox;               // endast för lövnoder
    AABB  fatBox;                 // för alla noder
    Node* parent = nullptr;       // för att kunna gå uppåt i trädet
    Node* childA = nullptr;
    Node* childB = nullptr;
    int   start;
    int   count;
    bool  dirty;
    AABBRenderer aabbRenderer;
};

struct NodePair { 
    Node* A; 
    Node* B; 
};

struct CollisionPair {
    GameObject* A;
    GameObject* B;
};

class BVHTree {
    int minOverlapDepth              = 6;
    float updateInterval             = 120.0f;
    int   leafThreshold              = 1;
    int   numRefits                  = 0;
    int   numIterationsSinceRebuild  = 0;
    int   rebuildThreshold           = 0;        // räknas om i build()
    int   minRebuildThreshold        = 5;        // min 10 refits innan rebuild
    float rebuildRatio               = 0.20f;    // 40 % av lövkorrektioner → rebuild
    glm::vec3 fatBoxMargin { 2.0f };

public:
    Node* root;
    std::vector<Node> nodes;
    void build(std::vector<GameObject>& objects);
    void update(std::vector<GameObject>& objects);
    int treeVsTreeQuery(BVHTree& bvhA, BVHTree& bvhB);

    int singleQuery(AABB& qBox);
    std::vector<GameObject*> collisions;

    // debug
    int numRebuilds = 0;
    CollisionPair* collisionBuf = new CollisionPair[MaxCollisionBuf];

    void printNode(const Node* node,
        const std::string& prefix,
        bool isLast) const;

private:
    std::vector<BVHPrimitive> prims;
    void createPrimitives(std::vector<GameObject>& objects);
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