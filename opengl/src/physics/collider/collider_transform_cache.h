#pragma once

#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>

#include "physics/public/physics_types.h"

namespace physics::internal {

struct ColliderTransformCache {
   glm::mat3 rotationMatrix{ 1.0f };
   glm::mat3 invRotationMatrix{ 1.0f };

   glm::mat4 modelMatrix{ 1.0f };
   glm::mat4 invModelMatrix{ 1.0f };

   bool modelDirty = true;
   bool invModelDirty = true;
   bool rotationDirty = true;
   bool invRotationDirty = true;

   void markDirty() {
      modelDirty = true;
      invModelDirty = true;
      rotationDirty = true;
      invRotationDirty = true;
   }

   void ensureModelMatrix(const Pose& pose) {
      if (!modelDirty) {
         return;
      }

      glm::mat4 translation = glm::translate(glm::mat4(1.0f), pose.position);
      glm::mat4 rotation = glm::mat4_cast(pose.orientation);

      modelMatrix = translation * rotation;
      modelDirty = false;
   }

   void ensureInvModelMatrix(const Pose& pose) {
      if (!invModelDirty) {
         return;
      }

      ensureModelMatrix(pose);
      invModelMatrix = glm::inverse(modelMatrix);
      invModelDirty = false;
   }

   void ensureRotationMatrix(const Pose& pose) {
      if (!rotationDirty) {
         return;
      }

      rotationMatrix = glm::mat3_cast(pose.orientation);
      rotationDirty = false;
   }

   void ensureInvRotationMatrix(const Pose& pose) {
      if (!invRotationDirty) {
         return;
      }

      ensureRotationMatrix(pose);
      invRotationMatrix = glm::transpose(rotationMatrix);
      invRotationDirty = false;
   }
};

}
