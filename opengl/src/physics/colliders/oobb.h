#pragma once

#include <array>
#include <vector>

#include "game/transform_utils.h"
#include "shaders/shader.h"
#include "vertex.h"

enum class FaceId {
    MinX, MaxX,
    MinY, MaxY,
    MinZ, MaxZ
};

static constexpr std::array<std::array<int, 4>, 6> FACE_INDICES = { {
    {{0, 4, 7, 3}}, // MinX
    {{5, 1, 2, 6}}, // MaxX
    {{1, 5, 4, 0}}, // MinY
    {{3, 7, 6, 2}}, // MaxY
    {{0, 3, 2, 1}}, // MinZ
    {{4, 5, 6, 7}}  // MaxZ
} };

class OOBB {
public:
    //constructor
    OOBB() = default;
    OOBB(const std::vector<glm::vec3>& verts, const Transform& t) {
        init(verts, t);
        update(t);
    };

    void init(const std::vector<glm::vec3>& verts, const Transform& t);

    void update(const Transform& t);
    std::array<glm::vec3, 4> getLocalFace(FaceId face) const;

    std::array<glm::vec3, 8> worldVertices;
    std::array<glm::vec3, 3> worldAxes;
    glm::vec3 worldCenter;
    glm::vec3 localHalfExtents;
    glm::vec3 scale;

private:
    glm::vec3 localCenter;
    std::array<glm::vec3, 8> localVertices;

    static constexpr std::array<glm::vec3, 3>  localAxes = {
      glm::vec3( 1,  0,  0),
      glm::vec3( 0,  1,  0),  
      glm::vec3( 0,  0,  1), 
    };
};