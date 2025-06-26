#include "oobb.h"

void OOBB::update(const glm::mat4& M) {
    centroid = glm::vec3(M[3]);

    // rotate normals
    glm::mat3 R = glm::mat3(M);
    for (int i = 0; i < 3; i++)
        wAxes[i] = glm::normalize(R * lAxes[i]);

    // transform world corners
    for (int i = 0; i < 8; ++i)
        wVertices[i] = glm::vec3(M * glm::vec4(lVertices[i], 1.0f));
}

void OOBB::getFace(int idx) {
    for (int i = 0; i < 4; ++i)
        wFace[i] = wVertices[faceIndices[idx][i]];
}

void OOBB::init(std::vector<glm::vec3>& verts) {
    glm::vec3 lMin(+FLT_MAX), lMax(-FLT_MAX); 
    for (auto const& v : verts) { 
        lMin = glm::min(lMin, v);  
        lMax = glm::max(lMax, v); 
    }

    halfExtents = (lMax - lMin) * 0.5f;

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

    wFace.resize(4);
}