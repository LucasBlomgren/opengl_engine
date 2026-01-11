#include "mesh.h"
#include "renderer/renderer.h"

void Mesh::draw() const {
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
}

void Mesh::setup() {
    // set index count for draw call
    indexCount = static_cast<GLsizei>(indices.size());

    // generate buffers/arrays
    glGenVertexArrays(1, &VAO); glcount::incVAO();
    glGenBuffers(1, &VBO); glcount::incVBO();
    glGenBuffers(1, &EBO); glcount::incEBO();
    glGenBuffers(1, &instanceVBO); glcount::incIBO();

    // bind VAO
    glBindVertexArray(VAO);

    // setup and fill EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // setup and fill VBO
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    // layout(location = 0) vec3 position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
    glEnableVertexAttribArray(0);
    // layout(location = 1) vec3 normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);
    // layout(location = 2) vec2 texCoords
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));
    glEnableVertexAttribArray(2);

    // setup instance VBO (empty for now)
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    std::size_t stride = sizeof(Renderer::InstanceData); // size of instance data

    // layout(location = 3..6) mat4 instanceModel
    for (int i = 0; i < 4; ++i) {
        glVertexAttribPointer(3 + i, 4, GL_FLOAT, GL_FALSE, stride, (void*)(i * sizeof(glm::vec4)));
        glEnableVertexAttribArray(3 + i);
        glVertexAttribDivisor(3 + i, 1);
    }

    // layout(location = 7) vec3 instanceColor
    glVertexAttribPointer(7, 3, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(glm::mat4));
    glEnableVertexAttribArray(7);
    glVertexAttribDivisor(7, 1);

    // unbind VAO
    glBindVertexArray(0);
}