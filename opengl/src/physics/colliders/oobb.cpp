#include "pch.h"
#include "oobb.h"

void OOBB::update(const glm::mat4& M) {
    // rotate normals
    glm::mat3 R = glm::mat3(M);
    for (int i = 0; i < 3; i++) {
        wAxes[i] = glm::normalize(R * lAxes[i]);
    }

    // transform world corners
    for (int i = 0; i < 8; ++i) {
        wVertices[i] = glm::vec3(M * glm::vec4(lVertices[i], 1.0f));
    }

    // compute centroid
    wCenter = glm::vec3(M * glm::vec4(lCenter, 1.0f));
}

std::array<OOBB::Edge, 4> OOBB::createEdgesAlongAxis(int axisIdx) const {
    int j = (axisIdx + 1) % 3;
    int k = (axisIdx + 2) % 3;

    glm::vec3 N1 = wAxes[j], N2 = wAxes[k];
    float e1 = lHalfExtents[j], e2 = lHalfExtents[k];
    float e3 = lHalfExtents[axisIdx];

    std::array<Edge, 4> edges;
    int idx = 0;
    for (int s1 : { -1, +1 }) {
        for (int s2 : { -1, +1 }) {
            glm::vec3 corner = wCenter + float(s1) * e1 * N1 + float(s2) * e2 * N2;
            edges[idx++] = {
                corner - e3 * wAxes[axisIdx],
                corner + e3 * wAxes[axisIdx]
            };
        }
    }
    return edges;
}

void OOBB::init(std::vector<glm::vec3>& verts, const glm::mat4& M) {
    glm::vec3 lMin(+FLT_MAX), lMax(-FLT_MAX); 
    for (auto const& v : verts) { 
        lMin = glm::min(lMin, v);  
        lMax = glm::max(lMax, v); 
    }

    lHalfExtents = (lMax - lMin) * 0.5f;

    lCenter = (lMin + lMax) * 0.5f;

    lVertices = {
        glm::vec3(lMin.x, lMin.y, lMin.z), 
        glm::vec3(lMax.x, lMin.y, lMin.z),
        glm::vec3(lMax.x, lMax.y, lMin.z), 
        glm::vec3(lMin.x, lMax.y, lMin.z), 

        glm::vec3(lMin.x, lMin.y, lMax.z), 
        glm::vec3(lMax.x, lMin.y, lMax.z), 
        glm::vec3(lMax.x, lMax.y, lMax.z), 
        glm::vec3(lMin.x, lMax.y, lMax.z)  
    };

    wAxes = {
        glm::vec3(1,  0,  0),
        glm::vec3(0,  1,  0),
        glm::vec3(0,  0,  1),
    };

    // transform world corners
    for (int i = 0; i < 8; ++i) {
        wVertices[i] = glm::vec3(M * glm::vec4(lVertices[i], 1.0f));
    }
}