#include "pch.h"
#include "aabb.h"

namespace physics::internal {

glm::vec3 AABB::center() const {
   return (min + max) * 0.5f;
}

glm::vec3 AABB::halfExtents() const {
   return (max - min) * 0.5f;
}

bool AABB::intersects(const AABB& b) const {
   constexpr float margin = 1e-4f;

   return
      (min.x <= b.max.x + margin && max.x + margin >= b.min.x) &&
      (min.y <= b.max.y + margin && max.y + margin >= b.min.y) &&
      (min.z <= b.max.z + margin && max.z + margin >= b.min.z);
}

void AABB::growToInclude(const glm::vec3& p) {
   min = glm::min(min, p);
   max = glm::max(max, p);
}

//=================================================
// BVH functions
//=================================================
void AABB::grow(glm::vec3 m) {
   min -= m;
   max += m;
}

bool AABB::contains(const AABB& other) const {
   return
      (min.x <= other.min.x) &&
      (min.y <= other.min.y) &&
      (min.z <= other.min.z) &&
      (max.x >= other.max.x) &&
      (max.y >= other.max.y) &&
      (max.z >= other.max.z);
}

float AABB::getSurfaceArea() const {
   return 2.0f * (
      halfExtents().x * halfExtents().y +
      halfExtents().y * halfExtents().z +
      halfExtents().z * halfExtents().x);
}

float AABB::getMergedSurfaceArea(const AABB& A, const AABB& B) {
   glm::vec3 min = glm::min(A.min, B.min);
   glm::vec3 max = glm::max(A.max, B.max);
   glm::vec3 halfExtents = (max - min) * 0.5f;

   return 2.0f * (
      halfExtents.x * halfExtents.y +
      halfExtents.y * halfExtents.z +
      halfExtents.z * halfExtents.x
      );
}

}


//// only for box meshes
//void AABB::transform_withRotation(const glm::mat3& M, const glm::vec3& T) {
//   float  a, b;
//   float  Amin[3], Amax[3];
//   float  Bmin[3], Bmax[3];
//
//   Amin[0] = localMin.x; Amax[0] = localMax.x;
//   Amin[1] = localMin.y; Amax[1] = localMax.y;
//   Amin[2] = localMin.z; Amax[2] = localMax.z;
//
//   Bmin[0] = Bmax[0] = T.x;
//   Bmin[1] = Bmax[1] = T.y;
//   Bmin[2] = Bmax[2] = T.z;
//
//   for (int i = 0; i < 3; i++) {
//      for (int j = 0; j < 3; j++) {
//         a = (M[j][i] * Amin[j]);
//         b = (M[j][i] * Amax[j]);
//
//         if (a < b) {
//            Bmin[i] += a;
//            Bmax[i] += b;
//         }
//         else {
//            Bmin[i] += b;
//            Bmax[i] += a;
//         }
//      }
//   }
//
//   min.x = Bmin[0]; max.x = Bmax[0];
//   min.y = Bmin[1]; max.y = Bmax[1];
//   min.z = Bmin[2]; max.z = Bmax[2];
//}