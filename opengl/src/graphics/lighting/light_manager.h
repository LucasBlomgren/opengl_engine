#pragma once

#include <vector>
#include "light.h"

struct DirectionalLight {
    glm::vec3 direction;
    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;
};

class LightManager {
public:
    void addLight(const Light& light);
    void clearLights();
    void clearDirectionalLight();
    const std::vector<Light>& getLights() const;

    void setDirectionalLight(const glm::vec3& direction, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular);
    DirectionalLight& getDirectionalLight();

private:
    std::vector<Light> lights;
    DirectionalLight directionalLight;
};