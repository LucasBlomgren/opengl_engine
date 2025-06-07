#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <array>
#include <vector>

#include "shader.h"
#include "vertex.h"
#include "draw_line.h"

class OOBB {
public:
    std::array<glm::vec3, 8> vertices;
    std::array<glm::vec3, 8> transformedVertices;
    std::array<glm::vec3, 3> normals;

    glm::vec3 centroid;
    glm::vec3 halfExtents;
    glm::vec3 lMin;
    glm::vec3 lMax;

    void Init(std::vector<glm::vec3>& vertices, const glm::mat4& M);
    void update(std::vector<glm::vec3>& verticesPositions, const glm::mat4& M);

private:
    void getTransformedVertices(const glm::mat4& M);
    void createVertices();
    void computeFromVertices(const std::vector<glm::vec3>& vertices);
    void updateNormals(const glm::mat4& M);
};