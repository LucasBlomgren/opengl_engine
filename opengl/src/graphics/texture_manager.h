#pragma once

#include <string>

class TextureManager {
public:
    static unsigned int loadTexture(const std::string& name, const std::string& path);
    static unsigned int getTexture(const std::string& name);

    static unsigned int loadCubemap(const std::string& name, const std::vector<std::string>& faces);
    static unsigned int getCubemap(const std::string& name);

private:
    static std::unordered_map<std::string, unsigned int> textures;
    static std::unordered_map<std::string, unsigned int> cubemaps;
};