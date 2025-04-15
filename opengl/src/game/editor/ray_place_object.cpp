#include "ray_place_object.h"

struct Raybox {
    glm::vec3 min;
    glm::vec3 max;
};

void createRayAABB(float rayLength, glm::vec3& camPos, glm::vec3& camFront, int selectedAxis, std::pair<Edge, Edge>& edgePair)
{
    glm::vec3 rayStart = camPos;
    glm::vec3 rayEnd = camPos + camFront * rayLength;

    Raybox rayAABB{
        glm::min(rayStart, rayEnd),
        glm::max(rayStart, rayEnd)
    };

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

    int lowIndex = findInsertPos(*sortedEdges, lowCoord);
    int highIndex = findInsertPos(*sortedEdges, highCoord);

    if (lowIndex == highIndex) {
        highIndex++;
    }

    edgesCopy.insert(edgesCopy.begin() + lowIndex, edgePair.first);
    edgesCopy.insert(edgesCopy.begin() + highIndex + 1, edgePair.second);

    insertionSort(edgesCopy);
    raySweepAndPrune(edgesCopy, collisionCandidates);
}

void raySweepAndPrune(const std::vector<Edge>& edges, std::vector<int>& collisionCandidates)
{
    collisionCandidates.reserve(edges.size());
    bool rayLowAdded = false;

    for (const Edge& edge : edges)
    {
        if (edge.id == -1 and edge.isMin) {
            rayLowAdded = true;
        }

        if (!rayLowAdded)
            continue;

        if (edge.id == -1 and !edge.isMin) {
            return;
        }

        if (edge.id != -1 and edge.isMin) {
            collisionCandidates.emplace_back(edge.id);
        }
    }
}