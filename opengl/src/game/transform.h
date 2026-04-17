#pragma once
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

struct Transform {
    glm::vec3 position{ 0.0f };
    glm::quat orientation{ 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 scale{ 1.0f };
    glm::mat4 modelMatrix{ 1.0f };

    glm::vec3 lastPosition{ 0.0f };

    Transform(
        const glm::vec3& position = glm::vec3{0.0f},
        const glm::quat& orientation = glm::quat{ 1.0f, 0.0f, 0.0f, 0.0f },
        const glm::vec3& scale = glm::vec3{ 1.0f }
    ) {
        this->position = position;
        this->orientation = orientation;
        this->scale = scale;
        updateCache();
    }

    void updateCache() {
        glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
        glm::mat4 R = glm::toMat4(orientation);
        glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
        modelMatrix = T * R * S;

        lastPosition = position;
    }

    glm::vec3 worldToLocalPoint(const glm::vec3& worldPoint) const {
        glm::vec3 p = worldPoint - position;
        p = glm::conjugate(glm::normalize(orientation)) * p;

        return glm::vec3(
            p.x / scale.x,
            p.y / scale.y,
            p.z / scale.z
        );
    }
};