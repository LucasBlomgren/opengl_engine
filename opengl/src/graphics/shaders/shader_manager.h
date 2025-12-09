#pragma once

class ShaderManager {
public:
    ShaderManager() {
        Shader defaultShader("src/graphics/shaders/default.vert", "src/graphics/shaders/default.frag");
        Shader defaultInstancedShader("src/graphics/shaders/default_instanced.vert", "src/graphics/shaders/default.frag");

        Shader shadowShader("src/graphics/shaders/shadow.vert", "src/graphics/shaders/shadow.frag");
        Shader shadowInstancedShader("src/graphics/shaders/shadow_instanced.vert", "src/graphics/shaders/shadow.frag");

        Shader debugShader("src/graphics/shaders/debug.vert", "src/graphics/shaders/debug.frag");
        Shader skyboxShader("src/graphics/shaders/skybox.vert", "src/graphics/shaders/skybox.frag");

        shaders["default"] = defaultShader;
        shaders["default_instanced"] = defaultInstancedShader;
        shaders["shadow"] = shadowShader;
        shaders["shadow_instanced"] = shadowInstancedShader;
        shaders["debug"]   = debugShader;
        shaders["skybox"]  = skyboxShader;

        shaders["default"].instancedVariant = &shaders["default_instanced"];
        shaders["shadow"].instancedVariant = &shaders["shadow_instanced"];
    };

    Shader* getShader(const std::string& name);

private:
    static std::unordered_map<std::string, Shader> shaders;
};