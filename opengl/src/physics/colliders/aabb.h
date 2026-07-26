#pragma once

#include <vector>
#include "collider_transform_cache.h"

class AABB {
public:
    // local
    glm::vec3 localMin{ 0.0f };
    glm::vec3 localMax{ 0.0f };
    // world
    glm::vec3 worldMin{ 0.0f };
    glm::vec3 worldMax{ 0.0f };

    glm::vec3 worldCenter{ 0.0f };
    glm::vec3 worldHalfExtents{ 0.0f };

    void init(const std::vector<glm::vec3>& vertices);
    void update(const ColliderTransformCache& transformCache);
    bool intersects(const AABB& b) const;

    // BVH functions
    bool contains(const AABB& other) const;
    void grow(glm::vec3 m);
    void growToInclude(const glm::vec3& p);
    float getMergedSurfaceArea(const AABB& a, const AABB& b);
    float getSurfaceArea() const;

    // Editor functions
    glm::vec3 getCollisionNormal(const AABB& other) const;
    glm::vec3 getOverlapDepth(const AABB& other) const;
    float getMinOverlapDepth(const AABB& other) const;

private:
    // Transformations
    void transform_noRotation(const glm::mat4& M, const glm::vec3& T, const glm::vec3 S);
    void transform_withRotation(const glm::mat3& M, const glm::vec3& T);
};