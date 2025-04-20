#pragma once
#include <vector>   
#include <unordered_set>
#include "ray.h"
#include "aabb.h"
#include "sweep_and_prune.h"
#include <glm/glm.hpp>

struct Raybox {
    glm::vec3 min;
    glm::vec3 max;
};

struct RaycastHit {
    int objIndex = -1;       
    glm::vec3 point;  
    glm::vec3 normal;
    float t;             
};

RaycastHit rayCastFinite(Physics::Ray& ray, std::vector<GameObject>* gameObjectList, std::vector<Edge>* edges, int selectedAxis);
void rayBroadphase(std::vector<int>& collisionCandidates, std::vector<Edge>* sortedEdges, std::vector<Edge>& edgesCopy, std::pair<Edge, Edge>& edgePair);
void createEdgePair(int selectedAxis, std::pair<Edge, Edge>& edgePair, Raybox& rayAABB);
int findInsertPos(const std::vector<Edge>& edges, float newCoord);
void raySweepAndPrune(const std::vector<Edge>& edges, std::vector<int>& collisionCandidates);
bool rayCastCheckOtherAxes(GameObject& objA, int selectedAxis, Raybox& rayAABB);