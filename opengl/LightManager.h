#pragma once

#include <vector>
#include "Light.h"

struct DirectionalLight {
    glm::vec3 direction;
    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;
};

class LightManager {
public:
    void AddLight(const Light& light);
    void Clear();
    const std::vector<Light>& GetLights() const;

    void setDirectionalLight(const glm::vec3& direction, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular);
    const DirectionalLight& GetDirectionalLight() const;

private:
    std::vector<Light> lights;
    DirectionalLight directionalLight;
};