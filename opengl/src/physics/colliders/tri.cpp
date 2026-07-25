#include "tri.h"

//AABB& Tri::getAABB() {
//    return aabb;
//}


AABB& Tri::getAABB() {
    AABB b;
    b.worldMin = glm::min(glm::min(vertices[0], vertices[1]), vertices[2]);
    b.worldMax = glm::max(glm::max(vertices[0], vertices[1]), vertices[2]);
    b.worldCenter = (b.worldMin + b.worldMax) * 0.5f;
    return b;
}