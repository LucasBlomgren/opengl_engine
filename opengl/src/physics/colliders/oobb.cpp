#include "oobb.h"

void OOBB::Init(std::vector<glm::vec3>& vertices, const glm::mat4& M) {
    computeFromVertices(vertices);
    createVertices();
    updateNormals(M);
    getTransformedVertices(M);

    halfExtents = (lMax - lMin) * 0.5f;
}

void OOBB::update(std::vector<glm::vec3>& verticesPositions, const glm::mat4& M) {
    updateNormals(M);
    getTransformedVertices(M);

    centroid = glm::vec3(M[3]);
}

void OOBB::getTransformedVertices(const glm::mat4& M) {
    for (int i = 0; i < vertices.size(); i++) {
        transformedVertices[i] = glm::vec3(M * glm::vec4(vertices[i], 1.0f));
    }
}

void OOBB::createVertices() {
    vertices[0] = { glm::vec3(lMin.x, lMin.y, lMin.z) };
    vertices[1] = { glm::vec3(lMax.x, lMin.y, lMin.z) };
    vertices[2] = { glm::vec3(lMin.x, lMax.y, lMin.z) };
    vertices[3] = { glm::vec3(lMax.x, lMax.y, lMin.z) };
    vertices[4] = { glm::vec3(lMin.x, lMin.y, lMax.z) };
    vertices[5] = { glm::vec3(lMax.x, lMin.y, lMax.z) };
    vertices[6] = { glm::vec3(lMin.x, lMax.y, lMax.z) };
    vertices[7] = { glm::vec3(lMax.x, lMax.y, lMax.z) };
}

void OOBB::computeFromVertices(const std::vector<glm::vec3>& vertices) {
    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());

    for (const auto& v : vertices) {
        mn = glm::min(mn, v);
        mx = glm::max(mx, v);
    }

    lMin = mn;
    lMax = mx;
}

void OOBB::updateNormals(const glm::mat4& M) {
    glm::mat3 R = glm::mat3(M);
    normals[0] = glm::normalize(R * glm::vec3(1, 0, 0));
    normals[1] = glm::normalize(R * glm::vec3(0, 1, 0));
    normals[2] = glm::normalize(R * glm::vec3(0, 0, 1));
}