#include "pch.h"
#include "raycast.h"
#include "aabb.h"

RaycastHit raycast(Ray& ray, const BVHTree& bvh, SlotMap<Collider, ColliderHandle>* colliderMap, SlotMap<GameObject, GameObjectHandle>* goMap)
{
    AABB rayAABB;
    rayAABB.wMin = { glm::min(ray.start.x, ray.end.x), glm::min(ray.start.y, ray.end.y), glm::min(ray.start.z, ray.end.z) };
    rayAABB.wMax = { glm::max(ray.start.x, ray.end.x), glm::max(ray.start.y, ray.end.y), glm::max(ray.start.z, ray.end.z) };

    ColliderHandle bestColliderHandle;
    float bestT = std::numeric_limits<float>::max();

    std::vector<ColliderHandle> collisions;
    bvh.singleQuery(rayAABB, collisions);

    bool noHit = true;
    for (ColliderHandle& handle : collisions) {
        Collider* collider = colliderMap->try_get(handle);
        GameObject* obj = goMap->try_get(collider->gameObjectHandle);
        if (obj->player) continue;

        glm::vec3 min = collider->aabb.wMin;
        glm::vec3 max = collider->aabb.wMax;

        glm::vec3 tMin = (min - ray.start) / ray.direction;
        glm::vec3 tMax = (max - ray.start) / ray.direction;

        glm::vec3 t1 = glm::min(tMin, tMax);
        glm::vec3 t2 = glm::max(tMin, tMax);

        float tNear = glm::compMax(t1);
        float tFar = glm::compMin(t2);

        if (!(tNear > tFar || tFar < 0.0f)) {
            if (tNear < bestT) {
                bestT = tNear;
                bestColliderHandle = handle;
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
    hitData.colliderHandle = bestColliderHandle;
    hitData.t = bestT;

    // Åter­beräkna t1 per axel
    Collider* bestColliderPtr = colliderMap->try_get(bestColliderHandle);
    glm::vec3 min = bestColliderPtr->aabb.wMin;
    glm::vec3 max = bestColliderPtr->aabb.wMax;
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