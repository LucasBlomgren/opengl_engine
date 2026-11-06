#pragma once

#include <vector>
#include "collider_transform_cache.h"

namespace physics::internal {

class AABB {
public:
   glm::vec3 min{ 0.0f };
   glm::vec3 max{ 0.0f };

   AABB() = default;

   AABB(glm::vec3& min, glm::vec3& max)
      : min(min)
      , max(max)
   {}

   glm::vec3 center() const;
   glm::vec3 halfExtents() const;

   bool intersects(const AABB& b) const;
   void growToInclude(const glm::vec3& p);

   // BVH functions
   void grow(glm::vec3 m);
   bool contains(const AABB& other) const;
   float getMergedSurfaceArea(const AABB& a, const AABB& b);
   float getSurfaceArea() const;

private:
   // Transformations
   //void transform_noRotation(const glm::mat4& M, const glm::vec3& T, const glm::vec3 S);
   //void transform_withRotation(const glm::mat3& M, const glm::vec3& T);
};

}
