#pragma once

#include <iostream>
#include <string>
#include <unordered_map>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stb/stb_image.h>

class TextureManager {
public:
    static unsigned int LoadTexture(const std::string& name, const std::string& path);
    static unsigned int GetTexture(const std::string& name);

private:
    static std::unordered_map<std::string, unsigned int> textures;
};