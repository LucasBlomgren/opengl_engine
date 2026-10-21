#include "pch.h"

#include "physics/raycast/raycast.h"
#include "physics/collider/aabb.h"

namespace physics::internal {

namespace raycast {
RaycastHit raycastTree(
   const Ray& ray,
   const BVHTree& bvh,
   const SlotMap<Collider, ColliderHandle>& colliderMap,
   BodyHandle ignoredBody)
{
   AABB rayAABB;
   rayAABB.worldMin = glm::min(ray.start, ray.end);
   rayAABB.worldMax = glm::max(ray.start, ray.end);

   ColliderHandle bestColliderHandle;
   float bestT = std::numeric_limits<float>::max();

   std::vector<BVHLeafRef> candidates;
   bvh.singleQuery(rayAABB, candidates);


   for (BVHLeafRef ref : candidates) {
      if (ref.body == ignoredBody) {
         continue;
      }

      const Collider& col = colliderMap.get(ref.collider);

      const glm::vec3 tMin =
         (col.aabb.worldMin - ray.start) / ray.direction;
      const glm::vec3 tMax =
         (col.aabb.worldMax - ray.start) / ray.direction;

      const glm::vec3 t1 = glm::min(tMin, tMax);
      const glm::vec3 t2 = glm::max(tMin, tMax);

      const float tNear = glm::compMax(t1);
      const float tFar = glm::compMin(t2);

      if (tNear <= tFar &&
         tFar >= 0.0f &&
         tNear < bestT)
      {
         bestT = tNear;
         bestColliderHandle = ref.collider;
      }
   }

   if (!bestColliderHandle.isValid()) {
      return {};
   }

   const Collider& bestCollider = colliderMap.get(bestColliderHandle);

   const glm::vec3 tMin =
      (bestCollider.aabb.worldMin - ray.start) / ray.direction;
   const glm::vec3 tMax =
      (bestCollider.aabb.worldMax - ray.start) / ray.direction;
   const glm::vec3 entry = glm::min(tMin, tMax);

   int hitAxis = 2;

   if (entry.x > entry.y && entry.x > entry.z) {
      hitAxis = 0;
   }
   else if (entry.y > entry.z) {
      hitAxis = 1;
   }

   glm::vec3 normal(0.0f);

   if (hitAxis == 0) {
      normal.x = ray.direction.x > 0.0f ? -1.0f : 1.0f;
   }
   else if (hitAxis == 1) {
      normal.y = ray.direction.y > 0.0f ? -1.0f : 1.0f;
   }
   else {
      normal.z = ray.direction.z > 0.0f ? -1.0f : 1.0f;
   }

   RaycastHit hit;
   hit.hit = true;
   hit.bodyHandle = bestCollider.bodyHandle;
   hit.point = ray.start + ray.direction * bestT;
   hit.normal = normal;
   hit.t = bestT;
   return hit;
}
}

}
