#pragma once

#include "graphics/shaders/shader.h"

class SkyboxManager {
private:
    unsigned int skyboxVAO, skyboxVBO; 
    unsigned int cubemapTexture;

public:
    void init();
    void render(Shader& shader);
    void toggleTexture();
};