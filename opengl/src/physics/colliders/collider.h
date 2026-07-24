#pragma once

#include <cstdint>
#include <variant>

#include "aabb.h"
#include "oobb.h"
#include "sphere.h"
#include "collider_transform_cache.h"

#include "physics/public/physics_handles.h"
#include "physics/public/physics_types.h"
#include "game/game_handles.h"

using ColliderShape = std::variant<OOBB, Sphere>;

enum class ColliderType {
    CUBOID,
    SPHERE
};

struct Collider {
    int id = -1;

    ColliderType type = ColliderType::CUBOID;
    ColliderShape shape;

    RigidBodyHandle rigidBodyHandle;

    // Temporary bridge for the current GameObject/Transform based engine.
    // Physics owns the pose data below; a future ECS layer must not depend on this.
    TransformHandle localTransformHandle;

    PhysicsPose localPose;
    glm::vec3 localScale{ 1.0f };

    PhysicsPose worldPose;
    glm::vec3 worldScale{ 1.0f };

    ColliderTransformCache transformCache;

    AABB aabb;
    bool aabbDirty = true;

    bool enabled = true;
    bool isTrigger = false;

    uint32_t userTag = 0;

    void updateWorldPose(const PhysicsPose& bodyPose, const glm::vec3& bodyScale);
    void updateShape();
    void updateAABB();

    AABB& getAABB();

private:
    void rebuildAABB();
};
