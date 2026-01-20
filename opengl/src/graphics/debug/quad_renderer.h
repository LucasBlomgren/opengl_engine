// QuadRenderer.h
#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include "camera.h"

class Shader; // forward-deklaration

class QuadRenderer {
public:
    QuadRenderer() {
        // Position (x,y,z) + UV (u,v)
        float vertices[] = {
            // pos               // uv
    // pos                // uv
    -1.f, -1.f, 0.0f,     0.f, 0.f,
     1.f, -1.f, 0.0f,     1.f, 0.f,
     1.f,  1.f, 0.0f,     1.f, 1.f,
    -1.f,  1.f, 0.0f,     0.f, 1.f
        };

        unsigned int indices[] = {
            0, 1, 2,
            2, 3, 0
        };

        glGenVertexArrays(1, &m_vao);
        glGenBuffers(1, &m_vbo);
        glGenBuffers(1, &m_ebo);

        glBindVertexArray(m_vao);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        // layout(location = 0) -> vec3 position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(
            0, 3, GL_FLOAT, GL_FALSE,
            5 * sizeof(float),
            (void*)0
        );

        // layout(location = 1) -> vec2 uv
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(
            1, 2, GL_FLOAT, GL_FALSE,
            5 * sizeof(float),
            (void*)(3 * sizeof(float))
        );

        glBindVertexArray(0);
    }

    ~QuadRenderer() {
        if (m_ebo) glDeleteBuffers(1, &m_ebo);
        if (m_vbo) glDeleteBuffers(1, &m_vbo);
        if (m_vao) glDeleteVertexArrays(1, &m_vao);
    }

    // Rendera quaden
    void draw(Shader* shader, Camera* camera, glm::vec3& dirLightDirection, int targetW, int targetH) {

        glm::mat4 viewMatrix = camera->GetViewMatrix();
        sunDir = glm::normalize(-dirLightDirection);

        shader->use();
        shader->setVec2("iResolution", glm::vec2(targetW, targetH));
        shader->setVec3("SunVec3", sunDir);
        shader->setMat4("ViewMatrix", viewMatrix);

        glDisable(GL_DEPTH_TEST);
        glBindVertexArray(m_vao);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
        glEnable(GL_DEPTH_TEST);
    }

    QuadRenderer(const QuadRenderer&) = delete;
    QuadRenderer& operator=(const QuadRenderer&) = delete;

private:
    unsigned int m_vao = 0;
    unsigned int m_vbo = 0;
    unsigned int m_ebo = 0;

    glm::vec3 sunDir = glm::normalize(glm::vec3(0.45f, 0.0f, -0.9f));
};