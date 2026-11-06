#pragma once

#include <cstdint>
#include <variant>

#include "aabb.h"
#include "shapes/box.h"
#include "shapes/sphere.h"
#include "collider_transform_cache.h"
#include "physics/public/handles.h"
#include "physics/public/physics_types.h"

namespace physics::internal {

using ColliderShape = std::variant<
   Box,
   Sphere,
   HullHandle,
   MeshHandle
>;

class Collider {
public:
   int id = -1;

   ShapeType type = ShapeType::BOX;
   ColliderShape shape;

   BodyHandle bodyHandle;

   Pose localPose;
   Pose worldPose;

   // #TODO: should NOT be here later because sphere doesnt need it
   ColliderTransformCache transformCache;

   AABB aabb;

   bool enabled = true;
   bool isTrigger = false;

   uint32_t userTag = 0;

   void updateWorldPose(const Pose& bodyPose);
   void updateShape();
   void rebuildAABB();
   void transformLocalAABB(const AABB& localAABB);

   AABB& getAABB();
   const AABB& getAABB() const;
};
}