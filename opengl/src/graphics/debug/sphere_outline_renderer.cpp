#include "pch.h"
#include "sphere_outline_renderer.h"

void SphereOutlineRenderer::init() {
    int   step = 4;  
    for (int i = 0; i < 360; i += step) {
        float a = glm::radians((float)i);
        unitCircle.emplace_back(cos(a), sin(a), 0.0f); 
    }
    
    glGenVertexArrays(1, &VAO); glcount::incVAO();
    glGenBuffers(1, &VBO); glcount::incVBO();

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, unitCircle.size() * sizeof(glm::vec3), unitCircle.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glDisableVertexAttribArray(1);
}

void SphereOutlineRenderer::render(
    Shader& shader, 
    glm::vec3& cameraPos, 
    glm::vec3& sphereCenter, 
    float radius, 
    const bool asleep, 
    const bool isStatic, 
    const bool selected, 
    const bool hovered) 
{
    glm::vec3 viewDir = glm::normalize(cameraPos - sphereCenter);

    glm::vec3 up = glm::abs(glm::dot(viewDir, glm::vec3(0, 1, 0))) > 0.99f
        ? glm::vec3(1, 0, 0)
        : glm::vec3(0, 1, 0);

    glm::vec3 U = glm::normalize(glm::cross(viewDir, up));
    glm::vec3 V = glm::normalize(glm::cross(viewDir, U));

    shader.setInt("debug.objectType", 1);
    if (selected) { shader.setVec3("debug.uColor", glm::vec3(0, 1, 0)); }
    else if (hovered) { shader.setVec3("debug.uColor", glm::vec3(1.0f, 0.65f, 0.0f)); }
    else if (isStatic) { shader.setVec3("debug.uColor", glm::vec3(0.30f, 0.95f, 0.50f)); }
    else if (!asleep) { shader.setVec3("debug.uColor", glm::vec3(1, 0, 0)); }
    else { shader.setVec3("debug.uColor", glm::vec3(0, 0, 1)); }

    shader.setVec3("debug.uCenter", sphereCenter);
    shader.setVec3("debug.uU", U * radius);
    shader.setVec3("debug.uV", V * radius);

    glLineWidth(4.0f);
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINE_LOOP, 0, unitCircle.size());
}

void SphereOutlineRenderer::destroy() {
    if (VBO) {
        glDeleteBuffers(1, &VBO);
        VBO = 0;
        glcount::decVBO();
    }
    if (VAO) {
        glDeleteVertexArrays(1, &VAO);
        VAO = 0;
        glcount::decVAO();
    }
}
