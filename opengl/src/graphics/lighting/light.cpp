#include "pch.h"
#include "light.h"

static std::vector<float> vertices = {
    -0.5f, -0.5f, -0.5f,  
     0.5f, -0.5f, -0.5f, 
     0.5f,  0.5f, -0.5f,  
    -0.5f,  0.5f, -0.5f,  
    -0.5f, -0.5f,  0.5f,  
     0.5f, -0.5f,  0.5f,  
     0.5f,  0.5f,  0.5f,  
    -0.5f,  0.5f,  0.5f   
};
static std::vector<unsigned int> indices = {
    0, 1, 2,    2, 3, 0,    // Back face
    4, 6, 5,    6, 4, 7,    // Front face
    4, 0, 3,    3, 7, 4,    // Left face
    1, 5, 6,    6, 2, 1,    // Right face
    3, 2, 6,    6, 7, 3,    // Top face
    4, 5, 1,    1, 0, 4     // Bottom face
};

Light::Light(const glm::vec3& pos, const glm::vec3& scale, const glm::vec3& col, float intensity)
    : position(pos), scale(scale), color(col), intensity(intensity) {
    setupMesh();
}

void Light::setPosition(const glm::vec3& pos) {
    position = pos;
}

const glm::vec3& Light::getPosition() const {
    return position;
}

void Light::setupMesh() {
    glGenVertexArrays(1, &VAO); glcount::incVAO();
    glGenBuffers(1, &VBO); glcount::incVBO();
    glGenBuffers(1, &EBO); glcount::incEBO();

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

void Light::render(const Shader& shader) const
{
    glm::mat4 model = glm::translate(glm::mat4(1.0f), position);
    model = glm::scale(model, this->scale); 

    shader.setMat4("model", model);
    shader.setBool("useTexture", false);
    shader.setBool("useUniformColor", true);
    shader.setVec3("uColor", color * intensity);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
}