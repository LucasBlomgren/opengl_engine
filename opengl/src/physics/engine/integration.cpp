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

      updateColliders(body);
   }
}

//==============================================================
// Update Collider Poses and Body Bounding Radius
//==============================================================
void Engine::updateColliders(RigidBody& body)
{
   Compound* compound = nullptr;

   if (body.colliderRef.isCompound()) {
      compound = &physicsStorage.getCompound(body.colliderRef.compound);
   }

   bool hasBounds = false;

   forEachBodyCollider(
      physicsStorage,
      body.colliderRef,
      [&](ColliderHandle colliderH) {
         Collider& collider = physicsStorage.getCollider(colliderH);

         if (!collider.enabled)
            return;

         collider.updateWorldPose(body.pose);
         collider.updateShape();
         collider.updateAABB();

         // Compound AABB bounds
         if (compound) {
            const internal::AABB& aabb = collider.getAABB();

            if (!hasBounds) {
               compound->aabb = aabb;
               hasBounds = true;
            }
            else {
               compound->aabb.worldMin =
                  glm::min(compound->aabb.worldMin, aabb.worldMin);

               compound->aabb.worldMax =
                  glm::max(compound->aabb.worldMax, aabb.worldMax);
            }
         }
      }
   );

   // Compound AABB center and extents
   if (compound) {
      if (hasBounds) {
         compound->aabb.worldCenter =
            (compound->aabb.worldMin + compound->aabb.worldMax) * 0.5f;

         compound->aabb.worldHalfExtents =
            (compound->aabb.worldMax - compound->aabb.worldMin) * 0.5f;
      }
      else {
         compound->aabb = internal::AABB{};
      }
   }
}

}