#pragma once

#include <array>
#include <vector>
#include <glm/vec3.hpp>

#include "physics/public/physics_handles.h"
#include "physics/public/physics_types.h"

namespace physics::debug {

struct Data {
    size_t awake = 0;
    size_t asleep = 0;
    size_t staticBodies = 0;
    size_t colliders = 0;
    size_t terrainTris = 0;
    size_t contacts = 0;
    size_t currentSubstepAmount = 0;
};

enum class StepPhase {
    Ready,
    PausedBeforePositionIntegration
};

enum class BvhType {
    Awake,
    Asleep,
    Static
};

struct ContactPoint {
    glm::vec3 worldPosition{ 0.0f };
    bool warmStarted = false;
};

struct Contact {
    glm::vec3 normal{ 0.0f };
    glm::vec3 representativePoint{ 0.0f };
    std::array<ContactPoint, 4> points{};
    size_t pointCount = 0;
};

struct SpeculativeContact {
    BodyHandle bodyA;
    BodyHandle bodyB;
    glm::vec3 worldPosition{ 0.0f };
};

struct BvhNode {
    bool isLeaf = false;
    AABB bounds;
};

struct Bvh {
    std::vector<BvhNode> nodes;
};

}
