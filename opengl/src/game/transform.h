#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

struct Transform {
    glm::vec3 position{ 0.0f };
    glm::vec3 lastPosition{ 0.0f };

    glm::quat orientation{ 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 scale{ 1.0f };

    glm::mat4 modelMatrix{ 1.0f };
    glm::mat4 invModelMatrix{ 1.0f };
    glm::mat3 rotationMatrix{ 1.0f };
    glm::mat3 invRotationMatrix{ 1.0f };

    void updateCache() {
        rotationMatrix = glm::mat3_cast(orientation);
        invRotationMatrix = glm::transpose(rotationMatrix);

        glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
        glm::mat4 R = glm::toMat4(orientation);
        glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);

        modelMatrix = T * R * S;
        invModelMatrix = glm::inverse(modelMatrix);
    }
};