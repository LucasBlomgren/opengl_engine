#include "tri.h"

AABB& Tri::getAABB() {
    return aabb;
}

//
//AABB Tri::getAABB() const {
//    AABB b;
//    b.wMin = glm::min(glm::min(vertices[0], vertices[1]), vertices[2]);
//    b.wMax = glm::max(glm::max(vertices[0], vertices[1]), vertices[2]);
//    b.centroid = (b.wMin + b.wMax) * 0.5f;
//    return b;
//}