#include "pch.h"

#include "physics/public/engine.h"
#include "physics/collider/aabb.h"
#include "physics/collider/body_collider_utils.h"

namespace physics {

using namespace internal;

//==============================================================
// Integrate Forces / Velocities
//==============================================================
void Engine::integrateForcesAndVelocities(
   const std::vector<BodyHandle>& awakeBodies,
   float dt)
{
   for (BodyHandle bodyHandle : awakeBodies) {
      RigidBody& body = physicsStorage.getBody(bodyHandle);

      if (!body.colliderRef.isValid())
         continue;

      if (body.type != BodyType::Dynamic)
         continue;

      SleepState& sleepState =
         physicsStorage.getSleepState(body.sleepStateHandle);

      ColliderHandle singleCollider{};

      if (body.colliderRef.isSingle()) {
         singleCollider = body.colliderRef.single;
      }
      else if (body.colliderRef.isCompound()) {
         const Compound& compound =
            physicsStorage.getCompound(body.colliderRef.compound);

         if (compound.colliders.size() == 1)
            singleCollider = compound.colliders.front();
      }

      if (singleCollider.isValid()) {
         const Collider& collider =
            physicsStorage.getCollider(singleCollider);

         body.applyRollingFriction(
            collider.type, dt, sleepState
         );
      }

      body.applyVelocityDamping(dt, sleepState);
      body.applyGravity(dt);
   }
}

//==============================================================
// Integrate Positions and Update Colliders
//==============================================================
void Engine::integratePositionsAndColliders(
   const std::vector<BodyHandle>& awakeBodies,
   float dt)
{
   for (BodyHandle bodyHandle : awakeBodies) {
      RigidBody& body = physicsStorage.getBody(bodyHandle);

      body.integratePose(dt);
      body.updateInertiaWorld();

      updateCollidersAndBodyRadius(body);
   }
}

//==============================================================
// Update Collider Poses and Body Bounding Radius
//==============================================================
void Engine::updateCollidersAndBodyRadius(RigidBody& body)
{
   float maxRadius = 0.0f;

   forEachBodyCollider(
      physicsStorage,
      body.colliderRef,
      [&](ColliderHandle colliderH) {
         Collider& collider = physicsStorage.getCollider(colliderH);

         if (!collider.enabled)
            return;

         collider.updateWorldPose(body.pose, body.scale);
         collider.updateShape();
         collider.updateAABB();

         const internal::AABB& aabb = collider.getAABB();

         const glm::vec3 minDelta =
            glm::abs(aabb.worldMin - body.pose.position);

         const glm::vec3 maxDelta =
            glm::abs(aabb.worldMax - body.pose.position);

         const float radius = glm::length(
            glm::max(minDelta, maxDelta)
         );

         maxRadius = std::max(maxRadius, radius);
      }
   );

   // #TODO: inte beräkna varje frame
   body.invRadius = maxRadius > 0.0f
      ? 1.0f / maxRadius
      : 0.0f;
}

}