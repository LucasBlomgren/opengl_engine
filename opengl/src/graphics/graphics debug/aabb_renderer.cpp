#include "aabb_renderer.h"

void AABBRenderer::updateModel(const AABB& box, const bool asleep)
{
    if (asleep)
        return;

    model = glm::translate(glm::mat4(1.0f), box.worldCenter) * glm::scale(glm::mat4(1.0f), box.halfExtents * 2.0f);
}

void AABBRenderer::drawBox(Shader& shader)
{
    shader.use();
    shader.setMat4("model", model);
    shader.setInt("debug.objectType", 0);
    shader.setBool("debug.useUniformColor", true);
    shader.setVec3("debug.uColor", color);

    glLineWidth(1.0f);
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, 24);
}

void AABBRenderer::setupWireframeBox(const AABB& box)
{
    glm::vec3 min{ box.Box.min.x.coord, box.Box.min.y.coord, box.Box.min.z.coord };
    glm::vec3 max{ box.Box.max.x.coord, box.Box.max.y.coord, box.Box.max.z.coord };

    glm::vec3 c = (min + max) * 0.5f;
    glm::vec3 he = (max - min) * 0.5f;

    model = glm::translate(glm::mat4(1.0f), c) * glm::scale(glm::mat4(1.0f), he * 2.0f);

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
