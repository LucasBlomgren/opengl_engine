#pragma once

#include <array>
#include <vector>

#include "game/transform_utils.h"
#include "shaders/shader.h"
#include "vertex.h"
#include "game/transform.h"

class AABB {
public:
    // local
    glm::vec3 localMin; 
    glm::vec3 localMax;
    // world
    glm::vec3 worldMin; 
    glm::vec3 worldMax;

    glm::vec3 worldCenter;
    glm::vec3 worldHalfExtents;
    float surfaceArea = 0.0f;

    void init(const std::vector<glm::vec3>& vertices);
    void update(const Transform& t);
    bool intersects(const AABB& b) const;

    // BVH functions
    bool contains(const AABB& other) const;
    void grow(glm::vec3 m);
    void growToInclude(const glm::vec3& p);
    float getMergedSurfaceArea(const AABB& a, const AABB& b);
    void setSurfaceArea();

    // Editor functions
    glm::vec3 getCollisionNormal(const AABB& other) const;
    glm::vec3 getOverlapDepth(const AABB& other) const;
    float getMinOverlapDepth(const AABB& other) const;

private:
    // Transformations
    void transform_noRotation(const glm::mat4& M, const glm::vec3& T, const glm::vec3 S);
    void transform_withRotation(const glm::mat3& M, const glm::vec3& T);
};