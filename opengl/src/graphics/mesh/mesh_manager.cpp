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