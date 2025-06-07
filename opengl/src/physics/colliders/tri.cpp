#include "tri.h"

AABB Tri::getAABB() const {
    AABB b;
    b.wMin = glm::min(glm::min(v0, v1), v2);
    b.wMax = glm::max(glm::max(v0, v1), v2);
    b.centroid = (b.wMin + b.wMax) * 0.5f;
    return b;
}