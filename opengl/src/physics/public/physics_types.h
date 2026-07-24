#pragma once

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

enum class BodyType {
    Dynamic,
    Kinematic,
    Static
};

enum class MotionControl {
    Physics,
    External
};

enum class ContactResponseMode {
    Normal,
    Character
};

struct PhysicsPose {
    glm::vec3 position{ 0.0f };
    glm::quat orientation{ 1.0f, 0.0f, 0.0f, 0.0f };
};