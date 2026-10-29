#pragma once

#include "physics/storage/physics_storage.h"

namespace physics::internal {

// Applies function to single or every compound collider
template<typename Func>
void forEachBodyCollider(
   PhysicsStorage& storage,
   BodyColliderRef ref,
   Func&& func)
{
   if (ref.isSingle()) {
      func(ref.single);
   }
   else if (ref.isCompound()) {
      auto& compound = storage.getCompound(ref.compound);

      for (ColliderHandle handle : compound.colliders) {
         func(handle);
      }
   }
}

inline glm::vec3 getColliderSize(Collider& col) {
   glm::vec3 size{ 0.0f };
   if (col.type == ColliderType::SPHERE) {
      const Sphere& sphere = std::get<Sphere>(col.shape);
      size = glm::vec3(sphere.radius * 2.0f);
   }
   else if (col.type == ColliderType::BOX) {
      const Box& box = std::get<Box>(col.shape);
      size = box.localHalfExtents * 2.0f;
   }

   return size;
}


}