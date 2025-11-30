#include "pch.h"
#include "aabb_renderer.h"

unsigned int AABBRenderer::sVAO = 0;
unsigned int AABBRenderer::sVBO = 0;
bool         AABBRenderer::sInitialized = false;

void AABBRenderer::InitShared() {
    if (sInitialized) return;
    sInitialized = true;

    float cubeWire[72] = {
        // bottom
        -0.5f, -0.5f, -0.5f, -0.5f, -0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f,  0.5f, -0.5f,  0.5f,
         0.5f, -0.5f,  0.5f,  0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f, -0.5f, -0.5f, -0.5f,
        // top 
        -0.5f, 0.5f, -0.5f, -0.5f, 0.5f,  0.5f,
        -0.5f, 0.5f,  0.5f,  0.5f, 0.5f,  0.5f,
         0.5f, 0.5f,  0.5f,  0.5f, 0.5f, -0.5f,
         0.5f, 0.5f, -0.5f, -0.5f, 0.5f, -0.5f,
        // vertical
        -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,  0.5f, 0.5f, -0.5f,
         0.5f, -0.5f,  0.5f,  0.5f, 0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f, -0.5f, 0.5f,  0.5f
    };

    glGenVertexArrays(1, &sVAO);
    glGenBuffers(1, &sVBO);

    glBindVertexArray(sVAO);
    glBindBuffer(GL_ARRAY_BUFFER, sVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeWire), cubeWire, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void AABBRenderer::CleanupShared() {
    if (!sInitialized) return;
    glDeleteBuffers(1, &sVBO);
    glDeleteVertexArrays(1, &sVAO);
    sInitialized = false;
}

void AABBRenderer::render(const glm::vec3& color, Shader& shader) const
{
    shader.setMat4("model", model);
    shader.setInt("debug.objectType", 0);
    shader.setVec3("debug.uColor", color);

    glDrawArrays(GL_LINES, 0, 24);
}

void AABBRenderer::updateModel(const AABB& box, const bool asleep) {
    if (asleep) return;

    model =
        glm::translate(glm::mat4(1.0f), box.centroid) * 
        glm::scale(glm::mat4(1.0f), box.halfExtents * 2.0f);
}