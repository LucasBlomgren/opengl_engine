#include "skybox_manager.h"
#include "geometry/sky_box_data.h"
#include "graphics/texture_manager.h"

void SkyboxManager::init() {
    glGenVertexArrays(1, &skyboxVAO);
    glGenBuffers(1, &skyboxVBO);
    glBindVertexArray(skyboxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, skyboxVertices.size() * sizeof(float), skyboxVertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0); 
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0); 

    cubemapTexture = TextureManager::getCubemap("skybox_default"); 
}

void SkyboxManager::render(Shader& shader) {
    shader.use();

    glBindVertexArray(skyboxVAO); 
    glActiveTexture(GL_TEXTURE0); 
    glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture); 
    glDrawArrays(GL_TRIANGLES, 0, 36); 
}

void SkyboxManager::toggleTexture() {
    if (cubemapTexture == TextureManager::getCubemap("skybox_night")) {
        cubemapTexture = TextureManager::getCubemap("skybox_default");
    }
    else {
        cubemapTexture = TextureManager::getCubemap("skybox_night");
    }
}