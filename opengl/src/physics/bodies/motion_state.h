#pragma once

#include <cstdint>
#include <glm/ext/matrix_float3x3.hpp>
#include <glm/ext/vector_float3.hpp>

namespace physics::internal {

class MotionState {
public:
    float mass = 0.0f;
    float invMass = 0.0f;
    float radius = 0.0f;
    float invRadius = 0.0f;
    bool allowGravity = true;

    uint32_t lastBiasCommitFrame = 0;
    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };
    glm::vec3 biasLinearVelocity{ 0.0f };
    glm::vec3 biasAngularVelocity{ 0.0f };
    glm::mat3 invInertiaLocal{ 0.0f };
    glm::mat3 invInertiaWorld{ 0.0f };
};
}