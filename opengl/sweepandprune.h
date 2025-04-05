#pragma once
#include "Mesh.h"
#include <unordered_set>

void updateEdgePos(const std::vector<Mesh>& meshList, std::vector<Edge>& allEdgesX, std::vector<Edge>& allEdgesY, std::vector<Edge>& allEdgesZ);
void insertionSort(std::vector<Edge>& edges);
float calculateVariance(const std::vector<Edge>& edges);
std::vector<Edge>* findMaxVarianceAxis(const float& variancex, const float& variancey, const float& variancez, std::vector<Edge>& alledgesx, std::vector<Edge>& alledgesy, std::vector<Edge>& alledgesz);
std::vector<std::pair<int, int>> findOverlap(const std::vector<Edge>& edges);
bool checkOtherAxes(int axisOrder, Mesh& objA, Mesh& objB);