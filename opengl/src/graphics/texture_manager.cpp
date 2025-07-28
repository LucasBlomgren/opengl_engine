#include "texture_manager.h"

std::unordered_map<std::string, unsigned int> TextureManager::textures;
std::unordered_map<std::string, unsigned int> TextureManager::cubemaps;

// ----------------------------
// ----- regular textures -----
// ----------------------------
unsigned int TextureManager::loadTexture(const std::string& name, const std::string& path) {
    // Check if already loaded
    if (textures.find(name) != textures.end())
        return textures[name];

    unsigned int textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    int width, height, nrChannels;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &nrChannels, 4);

    if (data) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else {
        std::cerr << "Failed to load texture: " << path << std::endl;
    }

    stbi_image_free(data);

    textures[name] = textureID;
    return textureID;
}

unsigned int TextureManager::getTexture(const std::string& name) {
    return textures[name];
}

// ----------------------------
// ----- cubemap textures -----
// ----------------------------
unsigned int TextureManager::loadCubemap(const std::string& name, const std::vector<std::string>& faces) {
    // Check if already loaded
    if (cubemaps.find(name) != cubemaps.end())
        return cubemaps[name];

    stbi_set_flip_vertically_on_load(false);

    unsigned int texID;
    glGenTextures(1, &texID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, texID);

    int width, height, nrChannels;
    for (unsigned int i = 0; i < faces.size(); i++) {
        unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 3);
        if (data) {
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        }
        else {
            std::cerr << "Cubemap face failed to load at path: " << faces[i] << "\n";
        }

        stbi_image_free(data); 
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    stbi_set_flip_vertically_on_load(true);

    cubemaps[name] = texID;
    return texID;
}

unsigned int TextureManager::getCubemap(const std::string& name) {
    return cubemaps.at(name);
}