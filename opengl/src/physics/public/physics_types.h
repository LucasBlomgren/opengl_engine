#pragma once

#include <algorithm>
#include <limits>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace physics {

enum class BodyType {
    Dynamic,
    Kinematic,
    Static
};

enum class MotionControl {
    Physics,
    External // editor/player selected (no physics response)
};

enum class ResponseMode {
    Normal, // for normal physics response
    Character // for exporting contacts to character controllers
};

enum class ColliderType {
    CUBOID,
    SPHERE
};

struct Pose {
    glm::vec3 position{ 0.0f };
    glm::quat orientation{ 1.0f, 0.0f, 0.0f, 0.0f };
};

struct AABB {
    glm::vec3 worldMin{ 0.0f };
    glm::vec3 worldMax{ 0.0f };
    glm::vec3 worldCenter{ 0.0f };
    glm::vec3 worldHalfExtents{ 0.0f };

    bool intersects(const AABB& other) const {
        constexpr float margin = 1e-4f;

        return
            worldMin.x <= other.worldMax.x + margin &&
            worldMax.x + margin >= other.worldMin.x &&
            worldMin.y <= other.worldMax.y + margin &&
            worldMax.y + margin >= other.worldMin.y &&
            worldMin.z <= other.worldMax.z + margin &&
            worldMax.z + margin >= other.worldMin.z;
    }

    glm::vec3 getOverlapDepth(const AABB& other) const {
        const glm::vec3 centerDelta = other.worldCenter - worldCenter;
        const glm::vec3 combinedHalfExtents =
            worldHalfExtents + other.worldHalfExtents;

        return glm::max(
            combinedHalfExtents - glm::abs(centerDelta),
            glm::vec3(0.0f)
        );
    }

    float getMinOverlapDepth(const AABB& other) const {
        const glm::vec3 depth = getOverlapDepth(other);
        float minimum = std::numeric_limits<float>::max();

        if (depth.x > 0.0f) minimum = (std::min)(minimum, depth.x);
        if (depth.y > 0.0f) minimum = (std::min)(minimum, depth.y);
        if (depth.z > 0.0f) minimum = (std::min)(minimum, depth.z);

        return minimum == std::numeric_limits<float>::max()
            ? 0.0f
            : minimum;
    }

    glm::vec3 getCollisionNormal(const AABB& other) const {
        const glm::vec3 depth = getOverlapDepth(other);

        if (depth.x <= 0.0f || depth.y <= 0.0f || depth.z <= 0.0f) {
            return glm::vec3(0.0f);
        }

        const glm::vec3 centerDelta = other.worldCenter - worldCenter;

        if (depth.x < depth.y && depth.x < depth.z) {
            return glm::vec3(
                centerDelta.x >= 0.0f ? -1.0f : 1.0f,
                0.0f,
                0.0f
            );
        }

        if (depth.y < depth.z) {
            return glm::vec3(
                0.0f,
                centerDelta.y >= 0.0f ? -1.0f : 1.0f,
                0.0f
            );
        }

        return glm::vec3(
            0.0f,
            0.0f,
            centerDelta.z >= 0.0f ? -1.0f : 1.0f
        );
    }
};

struct BoxGeometry {
    glm::vec3 worldCenter{ 0.0f };
    glm::vec3 localHalfExtents{ 0.5f };
    glm::vec3 scale{ 1.0f };
};

struct SphereGeometry {
    glm::vec3 worldCenter{ 0.0f };
    float radius = 0.5f;
};

}
