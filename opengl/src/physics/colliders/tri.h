#pragma once
#include <glm/vec3.hpp>

#include "aabb.h"

class Tri {
public:
    int id;
    glm::vec3 v0, v1, v2;
    std::vector<glm::vec3> vertices;
    glm::vec3 normal;
    std::vector<glm::vec3> axes; // For SAT
    glm::vec3 centroid;

    float avgHeight;

    // constructor
    Tri(const int id_, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
        : id(id_), v0(v0), v1(v1), v2(v2) 
    {
        vertices = { v0, v1, v2 };
        centroid = (v0 + v1 + v2) / 3.0f;

        // Calculate the normal using the cross product
        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 edge3 = v2 - v1; 
        normal = glm::normalize(glm::cross(edge1, edge2));
        axes = { normal, edge1, edge2, edge3 };

        avgHeight = (v0.y + v1.y + v2.y) / 3.0f;
    }

    Tri(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
        : v0(v0), v1(v1), v2(v2)
    {}

    AABB getAABB() const;
};