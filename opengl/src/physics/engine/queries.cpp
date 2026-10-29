#include "pch.h"

#include "physics/public/engine.h"
#include "physics/raycast/raycast.h"

namespace physics {

using namespace internal;

namespace {
internal::AABB getCombinedBounds(
   const PhysicsStorage& storage,
   const std::vector<ColliderHandle>& handles)
{
   internal::AABB bounds{};
   bool first = true;

   for (ColliderHandle handle : handles) {
      const Collider* collider = storage.tryGetCollider(handle);
      if (!collider || !collider->enabled)
         continue;

      const internal::AABB& box = collider->aabb;

      if (first) {
         bounds = box;
         first = false;
      }
      else {
         bounds.growToInclude(box.worldMin);
         bounds.growToInclude(box.worldMax);
      }
   }

   if (!first) {
      bounds.worldCenter = (bounds.worldMin + bounds.worldMax) * 0.5f;
      bounds.worldHalfExtents = (bounds.worldMax - bounds.worldMin) * 0.5f;
   }

   return bounds;
}

physics::AABB toPublicBounds(const internal::AABB& bounds) {
   physics::AABB result;
   result.worldMin = bounds.worldMin;
   result.worldMax = bounds.worldMax;
   result.worldCenter = bounds.worldCenter;
   result.worldHalfExtents = bounds.worldHalfExtents;
   return result;
}

internal::AABB toInternalBounds(const physics::AABB& bounds) {
   internal::AABB result;
   result.worldMin = bounds.worldMin;
   result.worldMax = bounds.worldMax;
   result.worldCenter = bounds.worldCenter;
   result.worldHalfExtents = bounds.worldHalfExtents;
   return result;
}
}

//=========================================
// Spatial queries
//=========================================
RaycastHit Engine::raycast(
   const Ray& ray,
   BodyHandle ignoredBody)
{
   RaycastHit bestHit;

   const BVHTree* trees[] = {
       &broadphaseManager.getAwakeBVH(),
       &broadphaseManager.getAsleepBVH(),
       &broadphaseManager.getStaticBVH()
   };

   for (const BVHTree* tree : trees) {
      RaycastHit hit = raycast::raycastTree(
         physicsStorage,
         ray,
         *tree,
         ignoredBody
      );

      if (hit.hit && (!bestHit.hit || hit.t < bestHit.t)) {
         bestHit = hit;
      }
   }

   return bestHit;
}

std::vector<BodyHandle> Engine::queryBodies(
   const physics::AABB& bounds,
   BodySet bodySet) const
{
   const BVHTree* tree = nullptr;

   switch (bodySet) {
   case BodySet::Awake:
      tree = &broadphaseManager.getAwakeBVH();
      break;
   case BodySet::Asleep:
      tree = &broadphaseManager.getAsleepBVH();
      break;
   case BodySet::Static:
      tree = &broadphaseManager.getStaticBVH();
      break;
   }

   std::vector<BodyHandle> result;
   if (!tree) return result;

   std::vector<BodyHandle> hits;
   tree->singleQuery(toInternalBounds(bounds), hits);

   std::unordered_set<BodyHandle> seen;
   seen.reserve(hits.size());
   result.reserve(hits.size());

   for (const BodyHandle& hit : hits) {
      if (seen.insert(hit).second) {
         result.push_back(hit);
      }
   }

   return result;
}

//=========================================
// State queries
//=========================================
std::optional<BodyState>
Engine::getRigidBodyState(BodyHandle handle) const 
{
   if (commandBuffer.isBodyPendingDestroy(handle))
      return std::nullopt;

   if (commandBuffer.tryGetPendingBodyCreate(handle))
      return std::nullopt;

   const RigidBody* body = physicsStorage.tryGetBody(handle);

   if (!body || commandBuffer.isBodyPendingDestroy(handle)) {
      return std::nullopt;
   }

   BodyState state;
   state.pose = body->pose;
   state.linearVelocity = body->linearVelocity;
   state.angularVelocity = body->angularVelocity;
   state.type = body->type;
   state.reportContacts = body->reportContacts;
   state.mass = body->mass;
   state.allowGravity = body->allowGravity;
   
   const BodyColliderRef& ref = body->colliderRef;

   if (ref.isCompound()) {
      state.colliders =
         physicsStorage.tryGetCompound(ref.compound)->colliders;
   }
   else if (ref.isSingle()) {
      state.colliders.push_back(ref.single);
   }

   state.bounds =
      toPublicBounds(getCombinedBounds(physicsStorage, state.colliders));


   if (body->type == BodyType::Dynamic) {
      const SleepState& sleepState =
         physicsStorage.getSleepState(body->sleepStateHandle);
      state.asleep = sleepState.isAsleep();
      state.allowSleep = sleepState.canSleep();
   }

   return state;
}

std::optional<ColliderState>
Engine::getColliderState(ColliderHandle handle) const
{
   if (commandBuffer.isColliderPendingDestroy(handle) ||
      commandBuffer.tryGetPendingColliderCreate(handle) ||
      !physicsStorage.colliderStorage().alive(handle))
      return std::nullopt;

   const Collider* collider =
      physicsStorage.tryGetCollider(handle);

   if (!collider ||
      commandBuffer.isBodyPendingDestroy(collider->bodyHandle))
      return std::nullopt;

   ColliderState state;
   state.body = collider->bodyHandle;
   state.localPose = collider->localPose;
   state.worldPose = collider->worldPose;
   state.type = collider->type;
   state.bounds = toPublicBounds(collider->aabb);
   state.enabled = collider->enabled;
   state.isTrigger = collider->isTrigger;
   state.userTag = collider->userTag;

   if (collider->type == ColliderType::BOX) {
      const Box& box = std::get<Box>(collider->shape);
      state.shape = BoxGeometry{
          box.worldCenter,
          box.localHalfExtents,
      };
   }
   else {
      const Sphere& sphere = std::get<Sphere>(collider->shape);
      state.shape = SphereGeometry{
          sphere.centerWorld,
          sphere.radius
      };
   }

   return state;
}

//=========================================
// Simulation output
//=========================================
const std::vector<ExternalMotionContact>&
Engine::getExternalMotionContacts() const {
   return narrowphaseManager.getExternalContacts();
}

}
