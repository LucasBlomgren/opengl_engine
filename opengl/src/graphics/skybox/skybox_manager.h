#pragma once

#include "shaders/shader.h"
#include "camera.h"

class SkyboxManager {
private:
    unsigned int skyboxVAO, skyboxVBO; 
    unsigned int cubemapTexture;

public:
    void init();
    void render(Shader& shader);
    void toggleTexture();
};