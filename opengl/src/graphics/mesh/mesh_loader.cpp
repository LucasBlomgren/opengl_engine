#include "pch.h"
#include "mesh_loader.h"

std::vector<Vertex> loadVerticesFromTxt(const std::string& path) 
{
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Could not open " + path);
    }

    std::vector<Vertex> vertices;
    Vertex v;
    while (file 
        >> v.position.x 
        >> v.position.y 
        >> v.position.z
        >> v.normal.x 
        >> v.normal.y 
        >> v.normal.z
        >> v.texCoords.x
        >> v.texCoords.y
        ) {
        vertices.push_back(v);
    }
    return vertices;
}

std::vector<unsigned int> loadIndicesFromTxt(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Could not open " + path);
    }

    std::vector<unsigned int> indices;
    unsigned int idx;
    while (file >> idx) {
        indices.push_back(idx);
    }
    return indices;
}

std::vector<Vertex> recenterMesh(std::vector<Vertex>& vertices) {
    glm::vec3 minPos(+FLT_MAX);
    glm::vec3 maxPos(-FLT_MAX);
    for (const auto& v : vertices) {
        minPos = glm::min(minPos, v.position);
        maxPos = glm::max(maxPos, v.position);
    }
    glm::vec3 center = (minPos + maxPos) * 0.5f;
    std::vector<Vertex> recenteredVertices = vertices;
    for (auto& v : recenteredVertices) {
        v.position -= center;
    }
    return recenteredVertices;
}