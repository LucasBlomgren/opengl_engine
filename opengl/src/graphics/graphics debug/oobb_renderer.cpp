#include "oobb_renderer.h"

void OOBBRenderer::drawBox(Shader& shader, glm::mat4& model, const bool asleep, const bool raycastHit)
{
    static constexpr float epsilon = 0.00f;
    glm::mat4 lift = glm::translate(glm::mat4(1.0f), glm::vec3(0, epsilon, 0));
    glm::mat4 displacedModel = lift * model;

    shader.use();
    shader.setMat4("model", displacedModel);
    shader.setInt("debug.objectType", 0);
    shader.setBool("debug.useUniformColor", true);
    if (raycastHit) { shader.setVec3("debug.uColor", glm::vec3(0, 1, 0)); }
    else if (!asleep) { shader.setVec3("debug.uColor", glm::vec3(1, 0, 0)); }
    else { shader.setVec3("debug.uColor", glm::vec3(0, 0, 1)); }

    glLineWidth(2.0f);
    glBindVertexArray(VAO_box);
    glDrawArrays(GL_LINES, 0, 24);
}

void OOBBRenderer::drawNormals(Shader& shader, const glm::mat4& model)
{
    shader.use();
    shader.setMat4("model", model);

    shader.setInt("debug.objectType", 0);
    shader.setBool("debug.useUniformColor", false);

    glLineWidth(4.0f);
    glBindVertexArray(VAO_normals);
    glDrawArrays(GL_LINES, 0, 18);
}

void OOBBRenderer::setupWireframeBox()
{
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

    glGenVertexArrays(1, &VAO_box);
    glGenBuffers(1, &VBO_box);

    glBindVertexArray(VAO_box);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_box);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeWire), cubeWire, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
}

void OOBBRenderer::setupNormals()
{
    float L = 1.0f;

    float normals[36] = {
        // pos                // color
        0.0f,  0.0f,  0.0f,   1.0f, 0.0f, 0.0f,
        L,     0.0f,  0.0f,   1.0f, 0.0f, 0.0f,
        0.0f,  0.0f,  0.0f,   0.0f, 1.0f, 0.0f,
        0.0f,  L,     0.0f,   0.0f, 1.0f, 0.0f,
        0.0f,  0.0f,  0.0f,   0.0f, 0.0f, 1.0f,
        0.0f,  0.0f,  L,      0.0f, 0.0f, 1.0f,
    };

    glGenVertexArrays(1, &VAO_normals);
    glGenBuffers(1, &VBO_normals);

    glBindVertexArray(VAO_normals);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_normals);
    glBufferData(GL_ARRAY_BUFFER, sizeof(normals), normals, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}