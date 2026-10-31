#pragma once

#include <cstdint>
#include <variant>

#include "aabb.h"
#include "shapes/box.h"
#include "shapes/sphere.h"
#include "shapes/mesh.h"
#include "collider_transform_cache.h"

#include "physics/public/handles.h"
#include "physics/public/physics_types.h"

namespace physics::internal {

using ColliderShape = std::variant<Box, Sphere, Mesh>;

class Collider {
public:
   int id = -1;

   ColliderType type = ColliderType::BOX;
   ColliderShape shape;

   BodyHandle bodyHandle;

   Pose localPose;
   Pose worldPose;

   ColliderTransformCache transformCache;

   AABB aabb;
   bool aabbDirty = true;

   bool enabled = true;
   bool isTrigger = false;

   uint32_t userTag = 0;

   void updateWorldPose(const Pose& bodyPose);
   void updateShape();
   void updateAABB();

   AABB& getAABB();

private:
   void rebuildAABB();
};

}
