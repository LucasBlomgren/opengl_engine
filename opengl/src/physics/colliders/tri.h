#pragma once
#include <glm/vec3.hpp>

#include "aabb.h"

class Tri {
public:
    int id;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> edges;
    std::vector<glm::vec3> axes; // For SAT
    glm::vec3 normal;
    glm::vec3 centroid;

    // constructor
    Tri(const int id_, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
        : id(id_)
    {
        vertices = { v0, v1, v2 };
        centroid = (v0 + v1 + v2) / 3.0f;

        // Calculate the normal using the cross product
        glm::vec3 edgeA = v1 - v0;
        glm::vec3 edgeB = v2 - v1;
        glm::vec3 edgeC = v0 - v2; 
        edges = { edgeA, edgeB, edgeC };

        normal = glm::normalize(glm::cross(edgeA, edgeB)); 
        axes = { normal, edgeA, edgeB, edgeC };
    }

    Tri(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
    {
        vertices = { v0, v1, v2 };
    }

    AABB getAABB() const;
};