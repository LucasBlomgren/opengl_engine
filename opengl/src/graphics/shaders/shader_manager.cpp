#include "pch.h"
#include "shader_manager.h"

std::unordered_map<std::string, Shader> ShaderManager::shaders;

Shader* ShaderManager::getShader(const std::string& name) {
    auto it = shaders.find(name);

    if (it == shaders.end()) {
        std::cerr << "[ShaderManager] No shader called \"" << name << "\"\n";
        return nullptr; // eller assert, eller fallback-shader-id
    }

    return &it->second;
}

unsigned int ShaderManager::getShaderId(const std::string& name) {
    auto it = shaders.find(name);

    if (it == shaders.end()) {
        std::cerr << "[ShaderManager] No shader called \"" << name << "\"\n";
        return 0; // eller assert, eller fallback-shader-id
    }

    return it->second.ID;
}
