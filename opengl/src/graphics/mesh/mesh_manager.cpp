#include "pch.h"
#include "mesh_manager.h"

std::unordered_map<std::string, Mesh> MeshManager::meshes;

Mesh* MeshManager::getMesh(const std::string& name) {
    auto it = meshes.find(name);

    if (it == meshes.end()) {
        std::cerr << "[MeshManager] No mesh called \"" << name << "\"\n";
        return nullptr; // eller assert, eller fallback-mesh-id
    }

    return &it->second;
}

void MeshManager::recenterVertices(std::vector<Vertex>& vertices) {
    glm::vec3 min(+FLT_MAX);
    glm::vec3 max(-FLT_MAX);

    for (const auto& v : vertices) {
        min = glm::min(min, v.position);
        max = glm::max(max, v.position);
    }

    glm::vec3 center = 0.5f * (min + max);

    for (auto& v : vertices) {
        v.position -= center;
    }
}