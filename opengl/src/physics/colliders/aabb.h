#pragma once

#include <array>
#include <vector>

#include "shaders/shader.h"
#include "vertex.h"

struct Faces {
    std::array<glm::vec3, 4> minX, maxX;
    std::array<glm::vec3, 4> minY, maxY;
    std::array<glm::vec3, 4> minZ, maxZ;
};

class AABB {
public:
    Faces lFaces;
    Faces wFaces;

    // local
    glm::vec3 lMin; 
    glm::vec3 lMax;
    // world
    glm::vec3 wMin; 
    glm::vec3 wMax;

    glm::vec3 centroid;
    glm::vec3 halfExtents;
    float surfaceArea = 0.0f;

    void Init(const std::vector<glm::vec3>& vertices);
    void update(glm::mat4& model, glm::vec3& pos, glm::vec3& scale, bool hasRotated);
    bool intersects(const AABB& b) const;

    // ----- bvh funktioner -----
    bool contains(const AABB& other) const;
    void grow(glm::vec3 m);
    void growToInclude(const glm::vec3& p);
    float getMergedSurfaceArea(const AABB& a, const AABB& b);
    void setSurfaceArea();

    // ----- editor funktioner -----
    glm::vec3 getCollisionNormal(const AABB& other) const;
    glm::vec3 getOverlapDepth(const AABB& other) const;
    float getMinOverlapDepth(const AABB& other) const;

private:
    // ----- transformations -----
    void transform_noRotation(const glm::mat4& M, const glm::vec3& T, const glm::vec3 S);
    void transform_withRotation(const glm::mat3& M, const glm::vec3& T);
    void computeFromVertices(const std::vector<glm::vec3>& vertices);
    void setLocalFaces();
};