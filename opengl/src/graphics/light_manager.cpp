#include "light_manager.h"

void LightManager::AddLight(const Light& light) {
    lights.push_back(light);
}

void LightManager::Clear() {
    lights.clear();
}

const std::vector<Light>& LightManager::GetLights() const {
    return lights;
}

void LightManager::setDirectionalLight(const glm::vec3& direction, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular) {
    directionalLight.direction = direction;
    directionalLight.ambient = ambient;
    directionalLight.diffuse = diffuse;
    directionalLight.specular = specular;
}

const DirectionalLight& LightManager::GetDirectionalLight() const {
    return directionalLight;
}