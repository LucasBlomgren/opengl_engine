#pragma once

#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>

#include "physics/public/physics_types.h"

namespace physics::internal {

class ColliderTransformCache {
private:
   mutable glm::mat3 rotationMatrix{ 1.0f };
   mutable glm::mat3 invRotationMatrix{ 1.0f };

   mutable glm::mat4 modelMatrix{ 1.0f };
   mutable glm::mat4 invModelMatrix{ 1.0f };

   mutable bool modelDirty = true;
   mutable bool invModelDirty = true;
   mutable bool rotationDirty = true;
   mutable bool invRotationDirty = true;

public:
   void markDirty() {
      modelDirty = true;
      invModelDirty = true;
      rotationDirty = true;
      invRotationDirty = true;
   }

   const glm::mat3& getRotationMatrix(const Pose& pose) const {
      if (rotationDirty) {
         rotationMatrix = glm::mat3_cast(pose.orientation);
         rotationDirty = false;
      }

      return rotationMatrix;
   }

   const glm::mat3& getInvRotationMatrix(const Pose& pose) const {
      if (invRotationDirty) {
         invRotationMatrix = glm::transpose(getRotationMatrix(pose));
         invRotationDirty = false;
      }

      return invRotationMatrix;
   }

   const glm::mat4& getModelMatrix(const Pose& pose) const {
      if (modelDirty) {
         glm::mat4 translation =
            glm::translate(glm::mat4(1.0f), pose.position);

         glm::mat4 rotation =
            glm::mat4(getRotationMatrix(pose));

         modelMatrix = translation * rotation;
         modelDirty = false;
      }

      return modelMatrix;
   }

   const glm::mat4& getInvModelMatrix(const Pose& pose) const {
      if (invModelDirty) {
         const glm::mat3& invRot = getInvRotationMatrix(pose);

         invModelMatrix = glm::mat4(invRot);
         invModelMatrix[3] = glm::vec4(
            -(invRot * pose.position), 1.0f
         );

         invModelDirty = false;
      }

      return invModelMatrix;
   }
};

}
