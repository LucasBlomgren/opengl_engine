#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <array>
#include <vector>

#include "shader.h"
#include "vertex.h"
#include "draw_line.h"

class OOBB
{
public:
    unsigned int VAO;
    unsigned int VBO;
    std::array<glm::vec3, 8> vertices;
    std::array<glm::vec3, 8> transformedVertices;
    std::array<glm::vec3, 3> normals;
    glm::vec3 color;
    std::array<glm::vec3, 24> bufferData;

    std::array<glm::vec3, 3> localAxes = {
        glm::vec3(1, 0, 0),
        glm::vec3(0, 1, 0),
        glm::vec3(0, 0, 1),
    };

    void Init(std::vector<glm::vec3>& vertices, const glm::mat4& M)
    {
        std::array<float, 6> extremePoints = getExtremePoints(vertices);
        createVertices(extremePoints);

        createNormals();
        setBufferData(this->vertices);
        setupBuffer(bufferData);

        getTransformedVertices(M);
    }

    void update(std::vector<glm::vec3>& verticesPositions, const glm::mat4& M, const bool shouldUpdateBuffer)
    {
        // if only has rotated (behöver optimeras också)
        updateNormals(M);
        getTransformedVertices(M);

        if (shouldUpdateBuffer) {
            setBufferData(transformedVertices);
            updateBuffer();
        }
    }

    void draw(Shader& shader, bool colliding, bool asleep, bool isStatic)
    {
        if (!asleep or isStatic) {
            setBufferData(transformedVertices);
            updateBuffer();
        }

        if (!colliding) { color = glm::vec3(0,0,1); }
        else { color = glm::vec3(1, 0, 0); }

        glm::mat4 model = glm::mat4(1.0f);
        shader.setMat4("model", model);
        shader.setBool("useTexture", false);
        shader.setBool("useUniformColor", true);
        shader.setVec3("uColor", color);

        glLineWidth(1.0f);
        glBindVertexArray(VAO);
        glDrawArrays(GL_LINES, 0, 24);
    }

    void drawNormals(Shader& shader, unsigned int& VAO, glm::vec3& position)
    {
        float lineLength = 10.0f;
        glLineWidth(3.0f);
        glm::vec3 lineEnd = position + normals[0] * lineLength;
        drawLine(shader, VAO, position, lineEnd, glm::vec3(1.0f, 0.0f, 0.0f));
        lineEnd = position + normals[1] * lineLength;
        drawLine(shader, VAO, position, lineEnd, glm::vec3(0.0f, 1.0f, 0.0f));
        lineEnd = position + normals[2] * lineLength;
        drawLine(shader, VAO, position, lineEnd, glm::vec3(0.0f, 0.0f, 1.0f));
    }

private:
    void getTransformedVertices(const glm::mat4& M)
    {
        for (int i = 0; i < vertices.size(); i++) {
            transformedVertices[i] = glm::vec3(M * glm::vec4(vertices[i], 1.0f));
        }
    }

    void updateBuffer()
    {
        // Bind the VBO
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        // Update the data in the VBO
        glBufferSubData(GL_ARRAY_BUFFER, 0, bufferData.size() * sizeof(glm::vec3), bufferData.data());
    }

    void setBufferData(const std::array<glm::vec3, 8>& vertices)
    {
        bufferData =
        {
            vertices[0], vertices[1],
            vertices[0], vertices[2],
            vertices[0], vertices[4],
            vertices[1], vertices[3],
            vertices[1], vertices[5],
            vertices[2], vertices[3],
            vertices[2], vertices[6],
            vertices[3], vertices[7],
            vertices[4], vertices[5],
            vertices[4], vertices[6],
            vertices[5], vertices[7],
            vertices[6], vertices[7]
        };
    }

    void createVertices(const std::array<float,6>& extremePoints)
    {
        float min_X = extremePoints[0];
        float min_Y = extremePoints[1];
        float min_Z = extremePoints[2];
        float max_X = extremePoints[3];
        float max_Y = extremePoints[4];
        float max_Z = extremePoints[5];

        vertices[0] = { glm::vec3(min_X, min_Y, min_Z) };
        vertices[1] = { glm::vec3(max_X, min_Y, min_Z) };
        vertices[2] = { glm::vec3(min_X, max_Y, min_Z) };
        vertices[3] = { glm::vec3(max_X, max_Y, min_Z) };
        vertices[4] = { glm::vec3(min_X, min_Y, max_Z) };
        vertices[5] = { glm::vec3(max_X, min_Y, max_Z) };
        vertices[6] = { glm::vec3(min_X, max_Y, max_Z) };
        vertices[7] = { glm::vec3(max_X, max_Y, max_Z) };
    }

    std::array<float,6> getExtremePoints(const std::vector<glm::vec3>& vertices)
    {
        // Initialize min and max values
        float max_X = std::numeric_limits<float>::lowest();
        float min_X = std::numeric_limits<float>::max();
        float max_Y = std::numeric_limits<float>::lowest();
        float min_Y = std::numeric_limits<float>::max();
        float max_Z = std::numeric_limits<float>::lowest();
        float min_Z = std::numeric_limits<float>::max();

        // Loop through the vertices and update min and max values
        for (size_t i = 0; i < vertices.size(); i++)
        {
            float x = vertices[i].x;
            float y = vertices[i].y;
            float z = vertices[i].z;

            if (x > max_X) max_X = x;
            if (x < min_X) min_X = x;
            if (y > max_Y) max_Y = y;
            if (y < min_Y) min_Y = y;
            if (z > max_Z) max_Z = z;
            if (z < min_Z) min_Z = z;
        }

        return { min_X, min_Y, min_Z, max_X, max_Y, max_Z };
    }

    void createNormals()
    {
        glm::vec3 edgeA;
        glm::vec3 edgeB;
        glm::vec3 normal;

        edgeA = vertices[0] - vertices[2];
        edgeB = vertices[0] - vertices[6];
        normal = glm::normalize(glm::cross(edgeA, edgeB));
        normals[0] = normal;

        edgeA = vertices[7] - vertices[3];
        edgeB = vertices[7] - vertices[2];
        normal = glm::normalize(glm::cross(edgeA, edgeB));
        normals[1] = normal;

        edgeA = vertices[0] - vertices[1];
        edgeB = vertices[0] - vertices[2];
        normal = glm::normalize(glm::cross(edgeA, edgeB));
        normals[2] = normal;
    }

    // måste optimeras
    void updateNormals(const glm::mat4& M) {
        glm::mat3 R = glm::mat3(M);
        normals[0] = glm::normalize(R * localAxes[0]);
        normals[1] = glm::normalize(R * localAxes[1]);
        normals[2] = glm::normalize(R * localAxes[2]);
    }

    void setupBuffer(const std::array<glm::vec3, 24>& bufferData)
    {
        // VAO
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        // VBO
        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, bufferData.size() * sizeof(glm::vec3), bufferData.data(), GL_STATIC_DRAW);
        // position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        glEnableVertexAttribArray(0);
    }
};