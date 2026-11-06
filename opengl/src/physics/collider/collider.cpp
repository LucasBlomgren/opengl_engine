#include "pch.h"
#include "collider.h"

#include "shapes/convex_hull.h"
#include "shapes/mesh_shape.h"

namespace physics::internal {


AABB& Collider::getAABB() {
   return aabb;
}

const AABB& Collider::getAABB() const {
   return aabb;
}

void Collider::updateWorldPose(const Pose& bodyPose) {
   glm::mat3 bodyRotation = glm::mat3_cast(bodyPose.orientation);

   worldPose.position = bodyPose.position + bodyRotation * localPose.position;
   worldPose.orientation = glm::normalize(bodyPose.orientation * localPose.orientation);

   transformCache.markDirty();
}

void Collider::updateShape() {
   std::visit([&](auto& colliderShape) {
      using ShapeType = std::decay_t<decltype(colliderShape)>;

      if constexpr (std::is_same_v<ShapeType, Box>) {
         colliderShape.update(worldPose, transformCache.getRotationMatrix(worldPose));
      }
      else if constexpr (std::is_same_v<ShapeType, Sphere>) {
         colliderShape.update(worldPose);
      }
      }, shape);
}

void Collider::rebuildAABB() {
   if (type == ShapeType::BOX) {
      const Box& box = std::get<Box>(shape);

      glm::vec3 minimum = box.worldVertices[0];
      glm::vec3 maximum = box.worldVertices[0];

      for (size_t i = 1; i < box.worldVertices.size(); ++i) {
         minimum = glm::min(minimum, box.worldVertices[i]);
         maximum = glm::max(maximum, box.worldVertices[i]);
      }

      aabb.min = minimum;
      aabb.max = maximum;
   }
   else if (type == ShapeType::SPHERE) {
      const Sphere& sphere = std::get<Sphere>(shape);

      aabb.min = sphere.centerWorld - glm::vec3(sphere.radius);
      aabb.max = sphere.centerWorld + glm::vec3(sphere.radius);
   }
}

void Collider::transformLocalAABB(const AABB& localAABB)
{
   const glm::mat3 R = glm::mat3_cast(worldPose.orientation);

   const glm::vec3 center =
      worldPose.position + R * localAABB.center();

   const glm::vec3 half = localAABB.halfExtents();

   const glm::vec3 extents =
      glm::abs(R[0]) * half.x +
      glm::abs(R[1]) * half.y +
      glm::abs(R[2]) * half.z;

   aabb.min = center - extents;
   aabb.max = center + extents;
}

}
