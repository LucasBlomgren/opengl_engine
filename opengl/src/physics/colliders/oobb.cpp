#include "pch.h"
#include "oobb.h"
#include "game/game_object.h"

void OOBB::update(const Transform& t) {
    for (int i = 0; i < 3; i++) {
        axesWorld[i] = glm::normalize(t.rotationMatrix * lAxes[i]);
    }

    for (int i = 0; i < 8; ++i) {
        verticesWorld[i] = glm::vec3(t.modelMatrix * glm::vec4(verticesLocal[i], 1.0f));
    }

    centerWorld = glm::vec3(t.modelMatrix * glm::vec4(centerLocal, 1.0f));

    scale = t.scale;
}

void OOBB::init(const std::vector<glm::vec3>& verts, const Transform& t) {
    glm::vec3 lMin(+FLT_MAX), lMax(-FLT_MAX); 
    for (auto const& v : verts) { 
        lMin = glm::min(lMin, v);  
        lMax = glm::max(lMax, v); 
    }

    halfExtentsLocal = (lMax - lMin) * 0.5f;

    centerLocal = (lMin + lMax) * 0.5f;

    verticesLocal = {
        glm::vec3(lMin.x, lMin.y, lMin.z), 
        glm::vec3(lMax.x, lMin.y, lMin.z),
        glm::vec3(lMax.x, lMax.y, lMin.z), 
        glm::vec3(lMin.x, lMax.y, lMin.z), 

        glm::vec3(lMin.x, lMin.y, lMax.z), 
        glm::vec3(lMax.x, lMin.y, lMax.z), 
        glm::vec3(lMax.x, lMax.y, lMax.z), 
        glm::vec3(lMin.x, lMax.y, lMax.z)  
    };

    axesWorld = {
        glm::vec3(1,  0,  0),
        glm::vec3(0,  1,  0),
        glm::vec3(0,  0,  1),
    };

    // transform world corners
    for (int i = 0; i < 8; ++i) {
        verticesWorld[i] = glm::vec3(t.modelMatrix * glm::vec4(verticesLocal[i], 1.0f));
    }
}