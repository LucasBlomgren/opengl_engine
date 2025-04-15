#pragma once
#include "game_object.h"
#include <unordered_set>

void updateEdgePos(const std::vector<GameObject>& GameObjectList, std::vector<Edge>& allEdgesX, std::vector<Edge>& allEdgesY, std::vector<Edge>& allEdgesZ);
void insertionSort(std::vector<Edge>& edges);
float calculateVariance(const std::vector<Edge>& edges);
std::vector<Edge>* findMaxVarianceAxis(const float& variancex, const float& variancey, const float& variancez, std::vector<Edge>& alledgesx, std::vector<Edge>& alledgesy, std::vector<Edge>& alledgesz);
void findOverlap(const std::vector<Edge>& edges, std::vector<std::pair<int, int>>& collisionCouplesList);
bool checkOtherAxes(int axisOrder, GameObject& objA, GameObject& objB);