#pragma once

#include "physics/public/handles.h"
#include "physics/public/physics_types.h"
#include "physics/broadphase/rigidbody_types.h"
#include "physics/collider/collider.h"
#include "physics/collider/collider_ref.h"
#include "physics/bodies/motion_state.h"
#include "physics/sleep/sleep_state.h"

namespace physics::internal {

class RigidBody {
public:
   int id = -1;
   BodyType type = BodyType::Dynamic;
   bool reportContacts = false;

   // #TODO: RigidBody: add COM
   // annars fungerar fysiken bara om collidernas 
   // lokala transform är centrerade runt COM

   Pose pose;

   BodyColliderRef colliderRef;
   BroadphaseHandle broadphaseHandle;

   //==============================
   // physics state
   //=============================
   MotionStateHandle motionStateHandle;
   SleepStateHandle sleepStateHandle;

   float mass = 0.0f;
   float invMass = 0.0f;
   float invRadius = 0.0f;
   bool allowGravity = true;
   uint32_t lastBiasCommitFrame = 0;
   glm::vec3 linearVelocity{ 0.0f };
   glm::vec3 angularVelocity{ 0.0f };
   glm::vec3 biasLinearVelocity{ 0.0f };
   glm::vec3 biasAngularVelocity{ 0.0f };
   glm::mat3 invInertiaLocal{ 0.0f };
   glm::mat3 invInertiaWorld{ 0.0f };

   void pushBiasImpulseLinear(const glm::vec3& impulse);
   void pushBiasImpulseAngular(const glm::vec3& impulse);
   void commitBiasImpulses(float dt);

   void applyImpulseLinear(const glm::vec3& impulse);
   void applyImpulseAngular(const glm::vec3& impulse);

   void setReportContacts(bool report) {
      reportContacts = report;
   }
   void integratePose(float dt);

   void updateOrientation(
      glm::quat& orientation,
      const glm::vec3& angularVelocity,
      float dt
   );

   void updateInertiaWorld();

   void applyGravity(float dt);
   void applyVelocityDamping(float dt, SleepState& sleepState);
   void applyRollingFriction(ColliderType colliderType, float dt, SleepState& sleepState);
   void applyAntistuckFriction(float dt, SleepState& sleepState);

   void calculateInverseInertia(
      const ColliderType& type,
      const Collider& collider);

   void inertiaCube(const float sideX);
   void inertiaCuboid(const glm::vec3& scale);
   void inertiaSphere(const glm::vec3& scale);

   bool approxEqual(float a, float b, float epsilon = 0.0001f) {
      return fabs(a - b) < epsilon;
   }
};

}