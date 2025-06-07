#include "raycast.h"
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

RaycastHit raycast(Ray& ray, std::vector<GameObject>* gameObjectList, BVHTree<GameObject>* bvhTree)
{
    AABB rayAABB;
    rayAABB.wMin = { glm::min(ray.start.x, ray.end.x), glm::min(ray.start.y, ray.end.y), glm::min(ray.start.z, ray.end.z) };
    rayAABB.wMax = { glm::max(ray.start.x, ray.end.x), glm::max(ray.start.y, ray.end.y), glm::max(ray.start.z, ray.end.z) };

    GameObject* bestObj = nullptr;
    float bestT = std::numeric_limits<float>::max();

    int collisionCounter = bvhTree->singleQuery(rayAABB);

    for (int i = 0; i < collisionCounter; i++) {
        GameObject& obj = *bvhTree->collisions[i];

        glm::vec3 min = obj.aabb.wMin;
        glm::vec3 max = obj.aabb.wMax;

        glm::vec3 tMin = (min - ray.start) / ray.direction;
        glm::vec3 tMax = (max - ray.start) / ray.direction;

        glm::vec3 t1 = glm::min(tMin, tMax);
        glm::vec3 t2 = glm::max(tMin, tMax);

        float tNear = glm::compMax(t1);
        float tFar = glm::compMin(t2);

        if (!(tNear > tFar || tFar < 0.0f)) {
            if (tNear < bestT) {
                bestT = tNear;
                bestObj = &obj;
            }
        }
    }

    // no hit
    if (bestObj == nullptr) {
        RaycastHit emptyHit;
        return emptyHit;
    }

    // Create hit data
    RaycastHit hitData;
    hitData.point = ray.start + ray.direction * bestT;
    hitData.object = bestObj;
    hitData.t = bestT;

    // Åter­beräkna t1 per axel
    glm::vec3 min = bestObj->aabb.wMin;
    glm::vec3 max = bestObj->aabb.wMax;
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