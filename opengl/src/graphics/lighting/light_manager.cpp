#include "pch.h"
#include "light_manager.h"

void LightManager::addLight(const Light& light) {
    lights.push_back(light);
}

void LightManager::clearLights() {
    lights.clear();
}

void LightManager::clearDirectionalLight() {
    directionalLight = DirectionalLight();
}

const std::vector<Light>& LightManager::getLights() const {
    return lights;
}

void LightManager::setDirectionalLight(const glm::vec3& direction, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular) {
    directionalLight.direction = direction;
    directionalLight.ambient = ambient;
    directionalLight.diffuse = diffuse;
    directionalLight.specular = specular;
}

DirectionalLight& LightManager::getDirectionalLight() {
    return directionalLight;
}