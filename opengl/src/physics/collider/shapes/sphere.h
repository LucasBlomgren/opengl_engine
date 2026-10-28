#pragma once

#include <algorithm>

#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/public/physics_types.h"

namespace physics::internal {

class Sphere {
public:
   glm::vec3 centerLocal{ 0.0f };
   glm::vec3 centerWorld{ 0.0f };
   float radius = 0.5f;

   Sphere() = default;

   explicit Sphere(float radius, const glm::vec3& centerLocal)
      : radius(radius),
      centerLocal(centerLocal)
   {}

   void update(const Pose& worldPose) {
      glm::mat3 rotation = glm::mat3_cast(worldPose.orientation);
      centerWorld = worldPose.position + rotation * centerLocal;
   }
};

}
