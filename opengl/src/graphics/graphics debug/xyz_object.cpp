#include "xyz_object.h"

unsigned int setup_xyzObject()
{
    std::vector<float> xyzEdges =
    {
        0.0f,   5.0f,   0.0f,    1.0f, 0.0f, 0.0f,
        100.0f, 5.0f,   0.0f,    1.0f, 0.0f, 0.0f,
        0.0f,   5.0f,   0.0f,    0.0f, 1.0f, 0.0f,
        0.0f,   105.0f, 0.0f,    0.0f, 1.0f, 0.0f,
        0.0f,   5.0f,   0.0f,    0.0f, 0.0f, 1.0f,
        0.0f,   5.0f,   100.0f,  0.0f, 0.0f, 1.0f,
    };

    // VBO
    unsigned int VAO_xyz;
    unsigned int VBO_xyz;
    glGenBuffers(1, &VBO_xyz);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_xyz);
    glBufferData(GL_ARRAY_BUFFER, xyzEdges.size() * sizeof(float), xyzEdges.data(), GL_STATIC_DRAW);

    // VAO
    glGenVertexArrays(1, &VAO_xyz);
    glBindVertexArray(VAO_xyz);
    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    return VAO_xyz;
}

void draw_xyzObject(const Shader& shader, const unsigned int& VAO_xyz)
{
    shader.setInt("objectType", 0);
    glm::mat4 model = glm::mat4(1.0f);
    shader.setMat4("model", model);
    shader.setBool("useUniformColor", false);

    glLineWidth(7.0f);
    glBindVertexArray(VAO_xyz);
    glDrawArrays(GL_LINES, 0, 9);
}