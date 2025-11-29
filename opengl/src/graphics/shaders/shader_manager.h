#pragma once

class ShaderManager {
public:
    ShaderManager() {
        Shader defaultShader("src/graphics/shaders/object.vert", "src/graphics/shaders/object.frag");
        Shader debugShader("src/graphics/shaders/debug.vert", "src/graphics/shaders/debug.frag");
        Shader shadowShader("src/graphics/shaders/shadow.vert", "src/graphics/shaders/shadow.frag");
        Shader skyboxShader("src/graphics/shaders/skybox.vert", "src/graphics/shaders/skybox.frag");

        shaders["default"] = defaultShader;
        shaders["debug"]   = debugShader;
        shaders["shadow"]  = shadowShader;
        shaders["skybox"]  = skyboxShader;
    };

    Shader* getShader(const std::string& name);
    unsigned int getShaderId(const std::string& name);

private:
    static std::unordered_map<std::string, Shader> shaders;
};