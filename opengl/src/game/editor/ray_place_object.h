#pragma once
#include <vector>   
#include <unordered_set>
#include "aabb.h"
#include "sweep_and_prune.h"

void rayBroadphase(std::vector<int>& collisionCandidates, std::vector<Edge>* sortedEdges, std::vector<Edge>& edgesCopy, std::pair<Edge, Edge>& edgePair);
void createRayAABB(float rayLength, glm::vec3& camPos, glm::vec3& camFront, int selectedAxis, std::pair<Edge, Edge>& edgePair);
int findInsertPos(const std::vector<Edge>& edges, float newCoord);
void raySweepAndPrune(const std::vector<Edge>& edges, std::vector<int>& collisionCandidates);