#include "pch.h"
#include "oobb.h"

#include <glm/common.hpp>
#include <glm/gtc/quaternion.hpp>

namespace physics::internal {

OOBB::OOBB(const std::vector<glm::vec3>& vertices) {
    init(vertices);
}

OOBB::OOBB(const glm::vec3& halfExtents, const glm::vec3& center) {
    setLocalBounds(center, halfExtents);
}

void OOBB::init(const std::vector<glm::vec3>& vertices) {
    if (vertices.empty()) {
        setLocalBounds(glm::vec3(0.0f), glm::vec3(0.5f));
        return;
    }

    glm::vec3 minimum = vertices.front();
    glm::vec3 maximum = vertices.front();

    for (const glm::vec3& vertex : vertices) {
        minimum = glm::min(minimum, vertex);
        maximum = glm::max(maximum, vertex);
    }

    glm::vec3 center = (minimum + maximum) * 0.5f;
    glm::vec3 halfExtents = (maximum - minimum) * 0.5f;

    setLocalBounds(center, halfExtents);
}

void OOBB::setLocalBounds(const glm::vec3& center, const glm::vec3& halfExtents) {
    localCenter = center;
    localHalfExtents = glm::abs(halfExtents);
    rebuildLocalVertices();
}

void OOBB::rebuildLocalVertices() {
    const glm::vec3& center = localCenter;
    const glm::vec3& half = localHalfExtents;

    localVertices[0] = center + glm::vec3(-half.x, -half.y, -half.z);
    localVertices[1] = center + glm::vec3(half.x, -half.y, -half.z);
    localVertices[2] = center + glm::vec3(half.x, half.y, -half.z);
    localVertices[3] = center + glm::vec3(-half.x, half.y, -half.z);
    localVertices[4] = center + glm::vec3(-half.x, -half.y, half.z);
    localVertices[5] = center + glm::vec3(half.x, -half.y, half.z);
    localVertices[6] = center + glm::vec3(half.x, half.y, half.z);
    localVertices[7] = center + glm::vec3(-half.x, half.y, half.z);
}

void OOBB::update(const Pose& worldPose, const glm::vec3& worldScale) {
    scale = worldScale;

    glm::mat3 rotation = glm::mat3_cast(worldPose.orientation);

    worldAxes[0] = rotation * LOCAL_AXES[0];
    worldAxes[1] = rotation * LOCAL_AXES[1];
    worldAxes[2] = rotation * LOCAL_AXES[2];

    worldCenter = worldPose.position + rotation * (worldScale * localCenter);

    for (size_t i = 0; i < localVertices.size(); ++i) {
        glm::vec3 scaledLocalVertex = worldScale * localVertices[i];
        worldVertices[i] = worldPose.position + rotation * scaledLocalVertex;
    }
}

std::array<glm::vec3, 4> OOBB::getLocalFace(FaceId face) const {
    const std::array<int, 4>& indices = FACE_INDICES[static_cast<size_t>(face)];

    return {
        localVertices[indices[0]],
        localVertices[indices[1]],
        localVertices[indices[2]],
        localVertices[indices[3]]
    };
}

}
