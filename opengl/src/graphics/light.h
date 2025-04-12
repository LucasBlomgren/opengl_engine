#pragma once

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/glm.hpp>
#include <vector>
#include "shader.h"

class Light {
public:
    glm::vec3 position;
    glm::vec3 scale;
    glm::vec3 color;
    float intensity;

    glm::vec3 ambient = glm::vec3(0.05f);
    glm::vec3 diffuse = glm::vec3(0.8f);
    glm::vec3 specular = glm::vec3(0.05f);

    float constant = 1.0f;
    float linear = 0.0145f;
    float quadratic = 0.00175f;

    Light(const glm::vec3& pos, const glm::vec3& scale, const glm::vec3& col, float intensity);

    void draw(Shader& shader);
    void setPosition(const glm::vec3& pos);
    const glm::vec3& getPosition() const;

private:
    unsigned int VAO, VBO, EBO;

    void setupMesh();
};