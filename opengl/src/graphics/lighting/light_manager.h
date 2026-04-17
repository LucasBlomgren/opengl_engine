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
    std::vector<Light> lights{};
    DirectionalLight directionalLight{
        .direction = glm::vec3(-0.2f, -1.0f, -0.3f),
        .ambient = glm::vec3(0.05f),
        .diffuse = glm::vec3(0.4f),
        .specular = glm::vec3(0.5f)
    };
};