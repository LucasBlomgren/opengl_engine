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
    External // editor/player selected (no physics response)
};

enum class ContactResponseMode {
    Normal, // for normal physics response
    Character // for exporting contacts to character controllers
};

struct PhysicsPose {
    glm::vec3 position{ 0.0f };
    glm::quat orientation{ 1.0f, 0.0f, 0.0f, 0.0f };
};