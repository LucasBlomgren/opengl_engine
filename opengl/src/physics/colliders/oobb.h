#pragma once

#include <array>
#include <vector>

#include "game/transform.h"
#include "shaders/shader.h"
#include "vertex.h"

class OOBB {
public:
    //constructor
    OOBB() = default;
    OOBB(const std::vector<glm::vec3>& verts, const Transform& t) {
        init(verts, t);
        update(t);
    };

    void update(const Transform& t);
    void init(const std::vector<glm::vec3>& verts, const Transform& t);   // för COM tillfälligt

    std::array<glm::vec3, 8> verticesWorld;
    std::array<glm::vec3, 3> axesWorld;
    glm::vec3 centerWorld;
    glm::vec3 halfExtentsLocal;
    glm::vec3 scale;

private:
    glm::vec3 centerLocal;
    std::array<glm::vec3, 8> verticesLocal;

    static constexpr std::array<glm::vec3, 3>  lAxes = {
      glm::vec3( 1,  0,  0),
      glm::vec3( 0,  1,  0),  
      glm::vec3( 0,  0,  1), 
    };
};