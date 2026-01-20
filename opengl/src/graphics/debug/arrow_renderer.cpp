#include "pch.h"
#include "arrow_renderer.h"
#include "mesh/mesh_manager.h"

glm::mat4 ArrowRenderer::getModelMatrix(const glm::vec3& origin, const glm::vec3& dirN, const glm::vec3& scale) {
    glm::mat4 R = makeBasisFromY(dirN);
    glm::mat4 Msh = glm::translate(glm::mat4(1.0f), origin) * R * glm::scale(glm::mat4(1.0f), scale);
    return Msh;
}

glm::mat4 ArrowRenderer::makeBasisFromY(glm::vec3 forward) {
    forward = glm::normalize(forward);

    // välj referens som inte är parallell
    glm::vec3 worldRef =
        (std::abs(forward.y) < 0.999f)
        ? glm::vec3(0, 1, 0)
        : glm::vec3(1, 0, 0);

    // VIKTIGT: rätt cross-ordning för högerhänt system
    glm::vec3 right = glm::normalize(glm::cross(forward, worldRef));
    glm::vec3 up = glm::cross(right, forward);

    glm::mat4 R(1.0f);
    R[0] = glm::vec4(right, 0.0f);   // X
    R[1] = glm::vec4(forward, 0.0f); // Y (arrow direction)
    R[2] = glm::vec4(up, 0.0f);      // Z

    return R;
}