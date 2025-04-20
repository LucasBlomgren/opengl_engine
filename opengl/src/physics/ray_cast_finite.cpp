#include "ray_cast_finite.h"
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

RaycastHit rayCastFinite(Physics::Ray& ray, std::vector<GameObject>* gameObjectList, std::vector<Edge>* edges, int selectedAxis)
{
    if (!edges) {
        RaycastHit emptyHit;
        return emptyHit;
    }

    // Skapa rayAABB och Edge-paret för sweep and prune
    Raybox rayAABB{ glm::min(ray.start, ray.end), glm::max(ray.start, ray.end) };

    std::pair<Edge, Edge> edgePair;
    createEdgePair(selectedAxis, edgePair, rayAABB);

    // Sweep and prune
    std::vector<Edge> edgesCopy = *edges;
    std::vector<int> collisionCandidates;
    rayBroadphase(collisionCandidates, edges, edgesCopy, edgePair);

    // Check remaining two axes
    std::vector<int> aabbCollisions;
    for (int i = 0; i < collisionCandidates.size(); i++) {
        int objIndex = collisionCandidates[i];
        GameObject& obj = (*gameObjectList)[objIndex];

        if (rayCastCheckOtherAxes(obj, selectedAxis, rayAABB)) {
            aabbCollisions.push_back(objIndex);
        }
    }

    // Narrow phase
    int bestObjIndex = -1;
    float bestT = std::numeric_limits<float>::max();

    for (int i = 0; i < aabbCollisions.size(); i++) {
        int objIndex = aabbCollisions[i];
        GameObject& obj = (*gameObjectList)[objIndex];

        glm::vec3 min = { obj.AABB.Box.min.x.coord, obj.AABB.Box.min.y.coord, obj.AABB.Box.min.z.coord };
        glm::vec3 max = { obj.AABB.Box.max.x.coord, obj.AABB.Box.max.y.coord, obj.AABB.Box.max.z.coord };

        glm::vec3 tMin = (min - ray.start) / ray.direction;
        glm::vec3 tMax = (max - ray.start) / ray.direction;

        glm::vec3 t1 = glm::min(tMin, tMax);
        glm::vec3 t2 = glm::max(tMin, tMax);

        float tNear = glm::compMax(t1);
        float tFar = glm::compMin(t2);

        if (!(tNear > tFar || tFar < 0.0f)) {
            if (tNear < bestT) {
                bestT = tNear;
                bestObjIndex = objIndex;
            }
        }
    }

    if (bestObjIndex == -1) {
        RaycastHit emptyHit;
        return emptyHit;
    }

    // Create hit data
    RaycastHit hitData;
    GameObject& obj = (*gameObjectList)[bestObjIndex];
    hitData.point = ray.start + ray.direction * bestT;
    hitData.objIndex = bestObjIndex;
    hitData.t = bestT;
    hitData.normal = ray.direction;

    return hitData;
}


void createEdgePair(int selectedAxis, std::pair<Edge, Edge>& edgePair, Raybox& rayAABB)
{
    float lowCoord;
    float highCoord;
    if (selectedAxis == 0) {
        lowCoord = rayAABB.min.x;
        highCoord = rayAABB.max.x;
    }
    else if (selectedAxis == 1) {
        lowCoord = rayAABB.min.y;
        highCoord = rayAABB.max.y;
    }
    else {
        lowCoord = rayAABB.min.z;
        highCoord = rayAABB.max.z;
    }

    edgePair = std::make_pair(
        Edge(-1, true, lowCoord),
        Edge(-1, false, highCoord)
    );

}

int findInsertPos(const std::vector<Edge>& edges, float newCoord) {
    int low = 0;
    int high = static_cast<int>(edges.size()) - 1;

    while (low <= high) {
        int mid = (low + high) / 2;
        if (edges[mid].coord < newCoord)
            low = mid + 1;
        else
            high = mid - 1;
    }
    return low;
}

void rayBroadphase(std::vector<int>& collisionCandidates, std::vector<Edge>* sortedEdges, std::vector<Edge>& edgesCopy, std::pair<Edge, Edge>& edgePair)
{
    float lowCoord = edgePair.first.coord;
    float highCoord = edgePair.second.coord;

    size_t lowIndex = findInsertPos(*sortedEdges, lowCoord);
    edgesCopy.insert(edgesCopy.begin() + lowIndex, edgePair.first);

    size_t highIndex = findInsertPos(edgesCopy, highCoord);
    edgesCopy.insert(edgesCopy.begin() + highIndex, edgePair.second);

    raySweepAndPrune(edgesCopy, collisionCandidates);
}

void raySweepAndPrune(const std::vector<Edge>& edges, std::vector<int>& collisionCandidates)
{
    std::unordered_set<int> currentTouching;
    collisionCandidates.reserve(edges.size());

    bool insideRay = false;

    for (const Edge& edge : edges) {
        if (edge.id == -1) {
            if (edge.isMin) {
                insideRay = true;
                continue;
            }
            else {
                break;
            }
        }

        if (edge.isMin) {
            currentTouching.insert(edge.id);
        }
        else if (!insideRay) {
            currentTouching.erase(edge.id);
        }
    }

    collisionCandidates.insert(collisionCandidates.end(), currentTouching.begin(), currentTouching.end());
}


bool rayCastCheckOtherAxes(GameObject& objA, int selectedAxis, Raybox& rayAABB) {
    Box& boxA = objA.AABB.Box;

    if (selectedAxis == 0)
    {
        if ((boxA.min.y.coord < rayAABB.max.y) && (boxA.max.y.coord > rayAABB.min.y))
            if ((boxA.min.z.coord < rayAABB.max.z) && (boxA.max.z.coord > rayAABB.min.z))
                return true;
    }
    else if (selectedAxis == 1)
    {
        if ((boxA.min.x.coord < rayAABB.max.x) && (boxA.max.x.coord > rayAABB.min.x))
            if ((boxA.min.z.coord < rayAABB.max.z) && (boxA.max.z.coord > rayAABB.min.z))
                return true;
    }
    else
    {
        if ((boxA.min.x.coord < rayAABB.max.x) && (boxA.max.x.coord > rayAABB.min.x))
            if ((boxA.min.y.coord < rayAABB.max.y) && (boxA.max.y.coord > rayAABB.min.y))
                return true;
    }

    return false;
}