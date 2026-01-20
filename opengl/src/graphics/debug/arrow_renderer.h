#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <vector>

#include "mesh/mesh.h"
#include "geometry/vertex.h"

class ArrowRenderer {
public:
    static glm::mat4 getModelMatrix(const glm::vec3& origin, const glm::vec3& dirN, const glm::vec3& scale);
    static glm::mat4 makeBasisFromY(glm::vec3 forward);

    Mesh* mesh = nullptr;
};