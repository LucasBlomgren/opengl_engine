#pragma once

#include <array>
#include <vector>

#include "shaders/shader.h"
#include "vertex.h"

class OOBB {
public:
    //constructor
    OOBB() = default;
    OOBB(std::vector<glm::vec3>& verts, const glm::mat4& M, const glm::vec3& scale) {
        init(verts, M);
        update(M, scale);
    };

    glm::vec3 lCenter;
    glm::vec3 wCenter;
    glm::vec3 lHalfExtents;
    glm::vec3 scale;
    std::array<glm::vec3, 8> lVertices;
    std::array<glm::vec3, 8> wVertices;

    std::array<std::array<int,4>,6> faceIndices = {{
        { 1, 2, 6, 5 },
        { 0, 4, 7, 3 },
        { 3, 7, 6, 2 },
        { 0, 1, 5, 4 },
        { 4, 5, 6, 7 },
        { 0, 3, 2, 1 },
    }};

    static constexpr std::array<glm::vec3, 3>  lAxes = {
      glm::vec3( 1,  0,  0),
      glm::vec3( 0,  1,  0),  
      glm::vec3( 0,  0,  1), 
    };
    std::array<glm::vec3, 3> wAxes;

    struct Edge { glm::vec3 A, B; };
    std::array<Edge, 4> createEdgesAlongAxis(int axisIdx) const;

    void update(const glm::mat4& M, const glm::vec3& scale);
    void init(std::vector<glm::vec3>& verts, const glm::mat4& M);   // för COM tillfälligt

private:
    //void init(std::vector<glm::vec3>& verts, const glm::mat4& M);
};