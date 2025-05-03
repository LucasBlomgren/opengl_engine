#include "aabb_renderer.h"

void AABBRenderer::updateModel(const AABB& box, const bool asleep)
{
    if (asleep)
        return;

    static constexpr float epsilon = 0.07f;
    glm::vec3 liftedCenter = box.centroid + glm::vec3(0, epsilon, 0);

    model = glm::translate(glm::mat4(1.0f), liftedCenter)
        * glm::scale(glm::mat4(1.0f), box.halfExtents * 2.0f);
}

void AABBRenderer::drawBox(Shader& shader)
{
    shader.use();
    shader.setMat4("model", model);
    shader.setInt("debug.objectType", 0);
    shader.setBool("debug.useUniformColor", true);
    shader.setVec3("debug.uColor", color);

    glLineWidth(2.0f);
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, 24);
}

void AABBRenderer::setupWireframeBox(const AABB& box)
{
    model = glm::translate(glm::mat4(1.0f), box.centroid) * glm::scale(glm::mat4(1.0f), box.halfExtents * 2.0f);

    // lines
    float cubeWire[72] = {
        // bottom
       -0.5f, -0.5f, -0.5f,  -0.5f, -0.5f,  0.5f,
       -0.5f, -0.5f,  0.5f,   0.5f, -0.5f,  0.5f,
        0.5f, -0.5f,  0.5f,   0.5f, -0.5f, -0.5f,
        0.5f, -0.5f, -0.5f,  -0.5f, -0.5f, -0.5f,
        // top 
       -0.5f, 0.5f, -0.5f,   -0.5f, 0.5f,  0.5f,
       -0.5f, 0.5f,  0.5f,    0.5f, 0.5f,  0.5f,
        0.5f, 0.5f,  0.5f,    0.5f, 0.5f, -0.5f,
        0.5f, 0.5f, -0.5f,   -0.5f, 0.5f, -0.5f,
        // vertical
       -0.5f, -0.5f, -0.5f,  -0.5f, 0.5f, -0.5f,
        0.5f, -0.5f, -0.5f,   0.5f, 0.5f, -0.5f,
        0.5f, -0.5f,  0.5f,   0.5f, 0.5f,  0.5f,
       -0.5f, -0.5f,  0.5f,  -0.5f, 0.5f,  0.5f
    };

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeWire), cubeWire, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void AABBRenderer::cleanup() {
    if (VAO) { glDeleteVertexArrays(1, &VAO); VAO = 0; }
    if (VBO) { glDeleteBuffers(1, &VBO); VBO = 0; }
}