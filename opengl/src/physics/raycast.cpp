#include "pch.h"
#include "raycast.h"
#include "aabb.h"

RaycastHit raycast(Ray& ray, const BVHTree& bvh, SlotMap<GameObject, GameObjectHandle>* slotmap)
{
    AABB rayAABB;
    rayAABB.wMin = { glm::min(ray.start.x, ray.end.x), glm::min(ray.start.y, ray.end.y), glm::min(ray.start.z, ray.end.z) };
    rayAABB.wMax = { glm::max(ray.start.x, ray.end.x), glm::max(ray.start.y, ray.end.y), glm::max(ray.start.z, ray.end.z) };

    GameObjectHandle bestObjHandle;
    float bestT = std::numeric_limits<float>::max();

    std::vector<GameObjectHandle> collisions;
    bvh.singleQuery(rayAABB, collisions);

    bool noHit = true;
    for (GameObjectHandle& handle : collisions) {
        GameObject* objPtr = slotmap->try_get(handle);

        if (!objPtr) {
            std::cout << "Error: Object handle in raycast is invalid. Function: raycast\n";
            continue;
        }

        if (objPtr->player) continue;

        glm::vec3 min = objPtr->aabb.wMin;
        glm::vec3 max = objPtr->aabb.wMax;

        glm::vec3 tMin = (min - ray.start) / ray.direction;
        glm::vec3 tMax = (max - ray.start) / ray.direction;

        glm::vec3 t1 = glm::min(tMin, tMax);
        glm::vec3 t2 = glm::max(tMin, tMax);

        float tNear = glm::compMax(t1);
        float tFar = glm::compMin(t2);

        if (!(tNear > tFar || tFar < 0.0f)) {
            if (tNear < bestT) {
                bestT = tNear;
                bestObjHandle = handle;
                noHit = false;
            }
        }
    }

    // no hit
    if (noHit) {
        RaycastHit emptyHit;
        return emptyHit;
    }

    // Create hit data
    RaycastHit hitData;
    hitData.hit = true;
    hitData.point = ray.start + ray.direction * bestT;
    hitData.objectHandle = bestObjHandle;
    hitData.t = bestT;

    // Åter­beräkna t1 per axel
    GameObject* bestObjPtr = slotmap->try_get(bestObjHandle);   
    glm::vec3 min = bestObjPtr->aabb.wMin;
    glm::vec3 max = bestObjPtr->aabb.wMax;
    glm::vec3 tMin = (min - ray.start) / ray.direction;
    glm::vec3 tMax = (max - ray.start) / ray.direction;
    glm::vec3 t1 = glm::min(tMin, tMax);  // entry‐tider
    //float tNear  = glm::compMax(t1);      // redan beräknat som bestT

    // Bestäm vilken axel som gav entry‐träffen
    int   hitAxis;
    if (t1.x > t1.y && t1.x > t1.z) hitAxis = 0;
    else if (t1.y > t1.z)                hitAxis = 1;
    else                                 hitAxis = 2;

    // Bygg normalen som ± enhets­vektor längs den axeln
    glm::vec3 n(0.0f);
    switch (hitAxis) {
    case 0: n.x = (ray.direction.x > 0.0f) ? -1.0f : 1.0f; break;
    case 1: n.y = (ray.direction.y > 0.0f) ? -1.0f : 1.0f; break;
    case 2: n.z = (ray.direction.z > 0.0f) ? -1.0f : 1.0f; break;
    }

    hitData.normal = n;

    return hitData;
}