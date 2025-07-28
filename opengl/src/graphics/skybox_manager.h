#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <string>

#include "shader.h"
#include "camera.h"

class SkyboxManager 
{
private:
    unsigned int skyboxVAO, skyboxVBO; 
    unsigned int cubemapTexture;

public:
    void init();
    void render(Shader& shader);
    void toggleTexture();
};