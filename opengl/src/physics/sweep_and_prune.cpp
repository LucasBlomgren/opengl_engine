#include "sweep_and_prune.h"

void updateEdgePos(const std::vector<GameObject>& GameObjectList, std::vector<Edge>& allEdgesX, std::vector<Edge>& allEdgesY, std::vector<Edge>& allEdgesZ)
{
    // Pointers = ingen update required
    for (Edge& edge : allEdgesX)
    {
        if (edge.isMin) { edge.coord = GameObjectList[edge.id].AABB.Box.min.x.coord; }
        else            { edge.coord = GameObjectList[edge.id].AABB.Box.max.x.coord; }
    }
    for (Edge& edge : allEdgesY)
    {
        if (edge.isMin) { edge.coord = GameObjectList[edge.id].AABB.Box.min.y.coord; }
        else            { edge.coord = GameObjectList[edge.id].AABB.Box.max.y.coord; }
    }
    for (Edge& edge : allEdgesZ)
    {
        if (edge.isMin) { edge.coord = GameObjectList[edge.id].AABB.Box.min.z.coord; }
        else            { edge.coord = GameObjectList[edge.id].AABB.Box.max.z.coord; }
    }
}

void insertionSort(std::vector<Edge>& edges) 
{
    for (int i = 1; i < edges.size(); i++)
        for (int j = i - 1; j >= 0; j--) {
            if (edges[j].coord < edges[j + 1].coord) break;
                std::swap(edges[j], edges[j + 1]);
        }
}

float calculateVariance(const std::vector<Edge>& edges) 
{
    // step 1: calculate the mean (average)
    float sum = 0.0;
    for (Edge edge : edges) {
        sum += edge.coord;
    }
    float mean = sum / edges.size();

    // step 2: calculate the sum of squared differences from the mean
    float sumsquareddiffs = 0.0;
    for (Edge edge : edges) {
        float diff = edge.coord - mean;
        sumsquareddiffs += diff * diff;
    }

    // step 3: calculate the variance (divide by the number of elements)
    float variance = sumsquareddiffs / edges.size();

    return variance;
}

std::vector<Edge>* findMaxVarianceAxis(const float& variancex, const float& variancey, const float& variancez, std::vector<Edge>& alledgesx, std::vector<Edge>& alledgesy, std::vector<Edge>& alledgesz)
{
    std::vector<Edge>* selectededges;
    if ((variancex > variancey) and (variancex > variancez))
        selectededges = &alledgesx;
    else if ((variancey > variancex) and (variancey > variancez))
        selectededges = &alledgesy;
    else
        selectededges = &alledgesz;

    return selectededges;
}

void findOverlap(const std::vector<Edge>& edges, std::vector<std::pair<int, int>>& collisionCouplesList)
{
    std::unordered_set<int> currentTouching;
    collisionCouplesList.reserve(edges.size());

    for (const Edge& edge : edges) {
        if (edge.isMin) {
            for (const int& touchingid : currentTouching) {
                collisionCouplesList.emplace_back(edge.id, touchingid);
            }
            currentTouching.insert(edge.id);
        }
        else
            currentTouching.erase(edge.id);
    }
}

bool checkOtherAxes(int axisOrder, GameObject& objA, GameObject& objB)
{
    Box& boxA = objA.AABB.Box;
    Box& boxB = objB.AABB.Box;

    if (axisOrder == 0)
    {
        if ((boxA.min.y.coord < boxB.max.y.coord) && (boxA.max.y.coord > boxB.min.y.coord))
            if ((boxA.min.z.coord < boxB.max.z.coord) && (boxA.max.z.coord > boxB.min.z.coord))
                return true;
    }
    else if (axisOrder == 1)
    {
        if ((boxA.min.x.coord < boxB.max.x.coord) && (boxA.max.x.coord > boxB.min.x.coord))
            if ((boxA.min.z.coord < boxB.max.z.coord) && (boxA.max.z.coord > boxB.min.z.coord))
                return true;
    }
    else
    {
        if ((boxA.min.x.coord < boxB.max.x.coord) && (boxA.max.x.coord > boxB.min.x.coord))
            if ((boxA.min.y.coord < boxB.max.y.coord) && (boxA.max.y.coord > boxB.min.y.coord))
                return true;
    }

    return false;
}