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
    AABB* tightBox = nullptr;     // endast för lövnoder
    AABB  fatBox;                 // för alla noder
    Node* parent = nullptr;       // för att kunna gå uppåt i trädet
    Node* childA = nullptr;
    Node* childB = nullptr;
    int   start;
    int   count;
    bool  dirty;
    AABBRenderer aabbRenderer;
};

class BVHTree {
    float updateInterval             = 1.0f / 180.0f;
    int   leafThreshold              = 1;
    int   fatBoxMargin               = 1;
    int   numRefits                  = 0;
    int   numIterationsSinceRebuild  = 0;
    int   rebuildThreshold           = 0;        // räknas om i build()
    int   minRebuildThreshold        = 5;        // min 10 refits innan rebuild
    float rebuildRatio               = 0.20f;    // 20 % av lövkorrektioner → rebuild

public:
    Node* root;
    std::vector<Node> nodes;
    void build(std::vector<GameObject>& objects);
    void update(std::vector<GameObject>& objects);

    void printNodeASCII(const Node* node, const std::string& prefix, bool isLeft) const;
    int numRebuilds = 0;
    
private:
    std::vector<BVHPrimitive> prims;

    void createPrimitives(std::vector<GameObject>& objects);
    void split(Node& parent);

    void updateLeaves();
    void refitNode(Node* node);
};