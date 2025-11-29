#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>
#include "shaders/shader.h"
#include "vertex.h"

class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    GLuint VAO = 0;
    GLuint VBO = 0;
    GLuint EBO = 0;
    GLsizei indexCount = 0;

    Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices) {
        this->vertices = vertices;
        this->indices = indices;

        setup();
    }

    void draw() const;
    void setup();
};