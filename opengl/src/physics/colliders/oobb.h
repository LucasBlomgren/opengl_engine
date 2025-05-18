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

    glm::vec3 centroid;
    glm::vec3 halfExtents;
    glm::vec3 lMin;
    glm::vec3 lMax;

    void Init(std::vector<glm::vec3>& vertices, const glm::mat4& M) {
        computeFromVertices(vertices);
        createVertices();
        updateNormals(M);
        getTransformedVertices(M);

        halfExtents = (lMax - lMin) * 0.5f;
    }

    void update(std::vector<glm::vec3>& verticesPositions, const glm::mat4& M) {
        updateNormals(M);
        getTransformedVertices(M);

        centroid = glm::vec3(M[3]);
    }

private:
   void getTransformedVertices(const glm::mat4& M) {
      for (int i = 0; i < vertices.size(); i++) {
         transformedVertices[i] = glm::vec3(M * glm::vec4(vertices[i], 1.0f));
      }
   }

    void createVertices() {
        vertices[0] = { glm::vec3(lMin.x, lMin.y, lMin.z) };
        vertices[1] = { glm::vec3(lMax.x, lMin.y, lMin.z) };
        vertices[2] = { glm::vec3(lMin.x, lMax.y, lMin.z) };
        vertices[3] = { glm::vec3(lMax.x, lMax.y, lMin.z) };
        vertices[4] = { glm::vec3(lMin.x, lMin.y, lMax.z) };
        vertices[5] = { glm::vec3(lMax.x, lMin.y, lMax.z) };
        vertices[6] = { glm::vec3(lMin.x, lMax.y, lMax.z) };
        vertices[7] = { glm::vec3(lMax.x, lMax.y, lMax.z) };
    }

    void computeFromVertices(const std::vector<glm::vec3>& vertices) {
       glm::vec3 mn(std::numeric_limits<float>::max());
       glm::vec3 mx(std::numeric_limits<float>::lowest());

       for (const auto& v : vertices) {
          mn = glm::min(mn, v);
          mx = glm::max(mx, v);
       }

       lMin = mn;
       lMax = mx;
    }

    void updateNormals(const glm::mat4& M) {
        glm::mat3 R = glm::mat3(M);
        normals[0] = glm::normalize(R * glm::vec3(1, 0, 0));
        normals[1] = glm::normalize(R * glm::vec3(0, 1, 0));
        normals[2] = glm::normalize(R * glm::vec3(0, 0, 1));
    }
};