#include "pch.h"
#include "collider.h"

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
   if (type == ColliderType::BOX) {
      const Box& box = std::get<Box>(shape);

      glm::vec3 minimum = box.worldVertices[0];
      glm::vec3 maximum = box.worldVertices[0];

      for (size_t i = 1; i < box.worldVertices.size(); ++i) {
         minimum = glm::min(minimum, box.worldVertices[i]);
         maximum = glm::max(maximum, box.worldVertices[i]);
      }

      aabb.worldMin = minimum;
      aabb.worldMax = maximum;
      aabb.worldCenter = (minimum + maximum) * 0.5f;
      aabb.worldHalfExtents = (maximum - minimum) * 0.5f;
   }

   else if (type == ColliderType::SPHERE) {
      const Sphere& sphere = std::get<Sphere>(shape);

      aabb.worldMin = sphere.centerWorld - glm::vec3(sphere.radius);
      aabb.worldMax = sphere.centerWorld + glm::vec3(sphere.radius);
      aabb.worldCenter = sphere.centerWorld;
      aabb.worldHalfExtents = glm::vec3(sphere.radius);
   }
}

void Collider::transformMeshAABB(const AABB& localAABB)
{
   const glm::mat3 R = glm::mat3_cast(worldPose.orientation);

   const glm::vec3 center =
      worldPose.position + R * localAABB.worldCenter;

   const glm::vec3 extents =
      glm::abs(R[0]) * localAABB.worldHalfExtents.x +
      glm::abs(R[1]) * localAABB.worldHalfExtents.y +
      glm::abs(R[2]) * localAABB.worldHalfExtents.z;

   aabb.worldCenter = center;
   aabb.worldHalfExtents = extents;
   aabb.worldMin = center - extents;
   aabb.worldMax = center + extents;
}

}
