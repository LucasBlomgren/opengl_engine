
#include "sat.h"

#include <algorithm>
#include <cmath>

namespace physics::internal {

bool SAT::speculativeBoxBox(
   const Collider& colliderA,
   const Collider& colliderB,
   const RigidBody& bodyA,
   const RigidBody& bodyB,
   float dt,
   SAT::Result& out)
{
   constexpr float eps = 1e-6f;
   constexpr float axisEps2 = 1e-10f;

   if (dt <= 0.0f)
      return false;

   const Box& boxA = std::get<Box>(colliderA.shape);
   const Box& boxB = std::get<Box>(colliderB.shape);

   const glm::vec3& centerA = colliderA.worldPose.position;
   const glm::vec3& centerB = colliderB.worldPose.position;

   const glm::vec3& halfA = boxA.localHalfExtents;
   const glm::vec3& halfB = boxB.localHalfExtents;

   const glm::mat3& rotA =
      colliderA.transformCache.getRotationMatrix(colliderA.worldPose);

   const glm::mat3& rotB =
      colliderB.transformCache.getRotationMatrix(colliderB.worldPose);

   // Rotation axes are unit length if quaternions are normalized.
   const glm::vec3 axesA[3] = {
      rotA[0], rotA[1], rotA[2]
   };

   const glm::vec3 axesB[3] = {
      rotB[0], rotB[1], rotB[2]
   };

   const glm::vec3& velocityA = bodyA.linearVelocity;
   const glm::vec3& velocityB = bodyB.linearVelocity;

   const glm::vec3 centerDelta = centerB - centerA;
   const glm::vec3 relV = velocityB - velocityA;

   // Precompute dot products between the box axes.
   float R[3][3];
   float absR[3][3];

   for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
         R[i][j] = glm::dot(axesA[i], axesB[j]);
         absR[i][j] = std::abs(R[i][j]);
      }
   }

   // Precompute relative position and velocity projections.
   float distA[3];
   float speedA[3];
   float distB[3];
   float speedB[3];

   for (int i = 0; i < 3; ++i) {
      distA[i] = glm::dot(centerDelta, axesA[i]);
      speedA[i] = glm::dot(relV, axesA[i]);

      distB[i] = glm::dot(centerDelta, axesB[i]);
      speedB[i] = glm::dot(relV, axesB[i]);
   }

   float enterTime = 0.0f;
   float exitTime = dt;

   glm::vec3 bestAxis{ 0.0f };
   float bestSeparation = 0.0f;

   SAT::AxisType bestType = SAT::AxisType::None;
   int bestFaceA = -1;
   int bestFaceB = -1;
   int bestEdgeA = -1;
   int bestEdgeB = -1;

   bool separatedAtStart = false;

   // Axis does not need to be normalized.
   auto testAxis = [&](
      const glm::vec3& axis,
      float radius,
      float dist,
      float speed,
      SAT::AxisType axisType,
      int faceA,
      int faceB,
      int edgeA,
      int edgeB) -> bool
      {
         const float len2 = glm::dot(axis, axis);

         if (len2 < axisEps2)
            return true;

         const float separation = std::abs(dist) - radius;

         if (separation > 0.0f)
            separatedAtStart = true;

         // Compare speed relative to axis length.
         if (speed * speed < eps * eps * len2)
            return separation <= 0.0f;

         // Solve:
         // -radius <= dist + speed * t <= radius
         const float invSpeed = 1.0f / speed;

         float t0 = (-radius - dist) * invSpeed;
         float t1 = (radius - dist) * invSpeed;

         if (t0 > t1)
            std::swap(t0, t1);

         if (t0 > enterTime) {
            enterTime = t0;

            // Normal A -> B at first entry.
            bestAxis = (speed < 0.0f) ? axis : -axis;

            // Separation is in unnormalized axis units.
            bestSeparation = std::max(separation, 0.0f);

            bestType = axisType;
            bestFaceA = faceA;
            bestFaceB = faceB;
            bestEdgeA = edgeA;
            bestEdgeB = edgeB;
         }

         exitTime = std::min(exitTime, t1);

         return enterTime <= exitTime;
      };

   // A face axes
   for (int i = 0; i < 3; ++i) {
      const float ra = halfA[i];

      const float rb =
         halfB[0] * absR[i][0] +
         halfB[1] * absR[i][1] +
         halfB[2] * absR[i][2];

      if (!testAxis(
         axesA[i],
         ra + rb,
         distA[i],
         speedA[i],
         SAT::AxisType::FaceA,
         i, -1, -1, -1))
      {
         return false;
      }
   }

   // B face axes
   for (int j = 0; j < 3; ++j) {
      const float ra =
         halfA[0] * absR[0][j] +
         halfA[1] * absR[1][j] +
         halfA[2] * absR[2][j];

      const float rb = halfB[j];

      if (!testAxis(
         axesB[j],
         ra + rb,
         distB[j],
         speedB[j],
         SAT::AxisType::FaceB,
         -1, j, -1, -1))
      {
         return false;
      }
   }

   // Edge-edge cross axes
   constexpr int next[3] = { 1, 2, 0 };
   constexpr int next2[3] = { 2, 0, 1 };

   for (int i = 0; i < 3; ++i) {
      const int i1 = next[i];
      const int i2 = next2[i];

      for (int j = 0; j < 3; ++j) {
         const int j1 = next[j];
         const int j2 = next2[j];

         const glm::vec3 axis = glm::cross(axesA[i], axesB[j]);

         // Projection radii using precomputed R.
         const float ra =
            halfA[i1] * absR[i2][j] +
            halfA[i2] * absR[i1][j];

         const float rb =
            halfB[j1] * absR[i][j2] +
            halfB[j2] * absR[i][j1];

         const float dist = glm::dot(centerDelta, axis);
         const float speed = glm::dot(relV, axis);

         if (!testAxis(
            axis,
            ra + rb,
            dist,
            speed,
            SAT::AxisType::EdgeEdge,
            -1, -1, i, j))
         {
            return false;
         }
      }
   }

   // Normal narrowphase handles initial overlaps.
   if (!separatedAtStart)
      return false;

   if (enterTime < 0.0f || enterTime > dt)
      return false;

   const float bestLen2 = glm::dot(bestAxis, bestAxis);

   if (bestLen2 < axisEps2)
      return false;

   // Normalize only the final collision axis.
   const float invLen = 1.0f / std::sqrt(bestLen2);

   bestAxis *= invLen;
   bestSeparation *= invLen;

   const glm::vec3 centerAAtHit =
      centerA + velocityA * enterTime;

   // Approximate support point on A.
   auto supportPoint = [](
      const glm::vec3& center,
      const glm::vec3 shapeAxes[3],
      const glm::vec3& halfExtents,
      const glm::vec3& dir) -> glm::vec3
      {
         glm::vec3 p = center;

         for (int i = 0; i < 3; ++i) {
            const float sign =
               glm::dot(shapeAxes[i], dir) >= 0.0f
               ? 1.0f : -1.0f;

            p += shapeAxes[i] * (halfExtents[i] * sign);
         }

         return p;
      };

   const glm::vec3 pointA = supportPoint(
      centerAAtHit,
      axesA,
      halfA,
      bestAxis
   );

   out.hitType = SAT::HitType::Speculative;
   out.normal = bestAxis;
   out.separation = bestSeparation;
   out.depth = -bestSeparation;
   out.toi = enterTime;
   out.point = pointA;

   out.feature.type = bestType;
   out.feature.faceIndex = -1;
   out.feature.edgeIndexA = bestEdgeA;
   out.feature.edgeIndexB = bestEdgeB;

   if (bestType == SAT::AxisType::FaceA)
      out.feature.faceIndex = bestFaceA;
   else if (bestType == SAT::AxisType::FaceB)
      out.feature.faceIndex = bestFaceB;

   return true;
}

}
