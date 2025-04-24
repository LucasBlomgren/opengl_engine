#include "world_frame.h"

unsigned int setup_worldFrame()
{
    std::vector<float> vertices =
    {
        0.0f,   0.0f,   0.0f,
        100.0f, 0.0f,   0.0f,
        0.0f,   0.0f,   0.0f,
        0.0f,   100.0f, 0.0f,
        0.0f,   0.0f,   0.0f,
        0.0f,   0.0f,   100.0f,

        500.0f, 0.0f,   0.0f,
        400.0f, 0.0f,   0.0f,
        500.0f, 0.0f,   0.0f,
        500.0f, 100.0f, 0.0f,
        500.0f, 0.0f,   0.0f,
        500.0f, 0.0f,   100.0f,

        500.0f, 0.0f,   500.0f,
        400.0f, 0.0f,   500.0f,
        500.0f, 0.0f,   500.0f,
        500.0f, 100.0f, 500.0f,
        500.0f, 0.0f,   500.0f,
        500.0f, 0.0f,   400.0f,

        0.0f,   0.0f,   500.0f,
        100.0f, 0.0f,   500.0f,
        0.0f,   0.0f,   500.0f,
        0.0f,   100.0f, 500.0f,
        0.0f,   0.0f,   500.0f,
        0.0f,   0.0f,   400.0f,

        0.0f,   500.0f, 0.0f,
        100.0f, 500.0f, 0.0f,
        0.0f,   500.0f, 0.0f,
        0.0f,   400.0f, 0.0f,
        0.0f,   500.0f, 0.0f,
        0.0f,   500.0f, 100.0f,

        500.0f, 500.0f, 0.0f,
        400.0f, 500.0f, 0.0f,
        500.0f, 500.0f, 0.0f,
        500.0f, 400.0f, 0.0f,
        500.0f, 500.0f, 0.0f,
        500.0f, 500.0f, 100.0f,

        500.0f, 500.0f, 500.0f,
        400.0f, 500.0f, 500.0f,
        500.0f, 500.0f, 500.0f,
        500.0f, 400.0f, 500.0f,
        500.0f, 500.0f, 500.0f,
        500.0f, 500.0f, 400.0f,

        0.0f,   500.0f, 500.0f,
        100.0f, 500.0f, 500.0f,
        0.0f,   500.0f, 500.0f,
        0.0f,   400.0f, 500.0f,
        0.0f,   500.0f, 500.0f,
        0.0f,   500.0f, 400.0f,
    };

    // VBO
    unsigned int VAO;
    unsigned int VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // VAO
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    return VAO;
}

void draw_worldFrame(const Shader& shader, const unsigned int& VAO_xyz)
{
    glm::mat4 model = glm::mat4(1.0f);
    shader.setMat4("model", model);
    shader.setBool("useUniformColor", true);
    shader.setVec3("uColor", glm::vec3(0, 0, 0));

    glLineWidth(5.0f);
    glBindVertexArray(VAO_xyz);
    glDrawArrays(GL_LINES, 0, 48);
}