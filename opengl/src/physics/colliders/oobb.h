#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <array>
#include <vector>

#include "shader.h"
#include "vertex.h"
#include "draw_line.h"

class OOBB {
public:
    std::array<glm::vec3, 8> vertices;
    std::array<glm::vec3, 8> transformedVertices;
    std::array<glm::vec3, 3> normals;

    void Init(std::vector<glm::vec3>& vertices, const glm::mat4& M) {
        std::array<float, 6> extremePoints = getExtremePoints(vertices);
        createVertices(extremePoints);
        updateNormals(M);
        getTransformedVertices(M);
    }

    void update(std::vector<glm::vec3>& verticesPositions, const glm::mat4& M) {
        updateNormals(M);
        getTransformedVertices(M);
    }

private:
    void getTransformedVertices(const glm::mat4& M) {
        for (int i = 0; i < vertices.size(); i++) {
            transformedVertices[i] = glm::vec3(M * glm::vec4(vertices[i], 1.0f));
        }
    }

    void createVertices(const std::array<float,6>& extremePoints) {
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

    std::array<float,6> getExtremePoints(const std::vector<glm::vec3>& vertices) {
        // Initialize min and max values
        float max_X = std::numeric_limits<float>::lowest();
        float min_X = std::numeric_limits<float>::max();
        float max_Y = std::numeric_limits<float>::lowest();
        float min_Y = std::numeric_limits<float>::max();
        float max_Z = std::numeric_limits<float>::lowest();
        float min_Z = std::numeric_limits<float>::max();

        // Loop through the vertices and update min and max values
        for (size_t i = 0; i < vertices.size(); i++) {
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

    void updateNormals(const glm::mat4& M) {
        glm::mat3 R = glm::mat3(M);
        normals[0] = glm::normalize(R * glm::vec3(1, 0, 0));
        normals[1] = glm::normalize(R * glm::vec3(0, 1, 0));
        normals[2] = glm::normalize(R * glm::vec3(0, 0, 1));
    }
};