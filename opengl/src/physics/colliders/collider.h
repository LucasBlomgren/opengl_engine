#pragma once

#include <cstdint>
#include <variant>

#include "aabb.h"
#include "oobb.h"
#include "sphere.h"
#include "collider_transform_cache.h"

#include "physics/public/physics_handles.h"
#include "physics/public/physics_types.h"

namespace physics::internal {

using ColliderShape = std::variant<OOBB, Sphere>;

struct Collider {
    int id = -1;

    ColliderType type = ColliderType::CUBOID;
    ColliderShape shape;

    BodyHandle rigidBodyHandle;

    Pose localPose;
    glm::vec3 localScale{ 1.0f };

    Pose worldPose;
    glm::vec3 worldScale{ 1.0f };

    ColliderTransformCache transformCache;

    AABB aabb;
    bool aabbDirty = true;

    bool enabled = true;
    bool isTrigger = false;

    uint32_t userTag = 0;

    void updateWorldPose(const Pose& bodyPose, const glm::vec3& bodyScale);
    void updateShape();
    void updateAABB();

    AABB& getAABB();

private:
    void rebuildAABB();
};

}
