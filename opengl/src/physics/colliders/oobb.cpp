#include "pch.h"
#include "oobb.h"

void OOBB::init(const std::vector<glm::vec3>& verts, const ColliderPose& pose) {
    glm::vec3 lMin(+FLT_MAX), lMax(-FLT_MAX); 
    for (auto const& v : verts) { 
        lMin = glm::min(lMin, v);  
        lMax = glm::max(lMax, v); 
    }

    localHalfExtents = (lMax - lMin) * 0.5f;

    localCenter = (lMin + lMax) * 0.5f;

    localVertices = {
        glm::vec3(lMin.x, lMin.y, lMin.z), 
        glm::vec3(lMax.x, lMin.y, lMin.z),
        glm::vec3(lMax.x, lMax.y, lMin.z), 
        glm::vec3(lMin.x, lMax.y, lMin.z), 

        glm::vec3(lMin.x, lMin.y, lMax.z), 
        glm::vec3(lMax.x, lMin.y, lMax.z), 
        glm::vec3(lMax.x, lMax.y, lMax.z), 
        glm::vec3(lMin.x, lMax.y, lMax.z)  
    };

    worldAxes = {
        glm::vec3(1,  0,  0),
        glm::vec3(0,  1,  0),
        glm::vec3(0,  0,  1),
    };

    // transform world corners
    for (int i = 0; i < 8; ++i) {
        worldVertices[i] = glm::vec3(pose.modelMatrix * glm::vec4(localVertices[i], 1.0f));
    }
}

void OOBB::update(const ColliderPose& pose) {
    //  transform world axes
    for (int i = 0; i < 3; i++) {
        worldAxes[i] = glm::normalize(pose.rotationMatrix * localAxes[i]);
    }

    // transform world corners
    for (int i = 0; i < 8; ++i) {
        worldVertices[i] = glm::vec3(pose.modelMatrix * glm::vec4(localVertices[i], 1.0f));
    }

    worldCenter = glm::vec3(pose.modelMatrix * glm::vec4(localCenter, 1.0f));
    scale = pose.scale;
}

std::array<glm::vec3, 4> OOBB::getLocalFace(FaceId face) const {
    const auto& idx = FACE_INDICES[static_cast<int>(face)];
    return {
        localVertices[idx[0]],
        localVertices[idx[1]],
        localVertices[idx[2]],
        localVertices[idx[3]]
    };
}