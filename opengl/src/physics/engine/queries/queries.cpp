#include "pch.h"

#include "physics/public/engine.h"
#include "physics/raycast/raycast.h"

namespace physics {

using namespace internal;

namespace {
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
   const SlotMap<RigidBody, BodyHandle>& bodyMap =
      physicsStorage.bodyStorage();

   RaycastHit bestHit;

   const BVHTree* trees[] = {
       &broadphaseManager.getAwakeBVH(),
       &broadphaseManager.getAsleepBVH(),
       &broadphaseManager.getStaticBVH()
   };

   for (const BVHTree* tree : trees) {
      RaycastHit hit = raycast::raycastTree(
         ray,
         *tree,
         bodyMap,
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

   std::vector<BVHLeafRef> hits;
   tree->singleQuery(toInternalBounds(bounds), hits);

   std::unordered_set<BodyHandle> seen;
   seen.reserve(hits.size());
   result.reserve(hits.size());

   for (const BVHLeafRef& hit : hits) {
      if (seen.insert(hit.body).second) {
         result.push_back(hit.body);
      }
   }

   return result;
}

//=========================================
// State queries
//=========================================
std::optional<BodyState>
Engine::getRigidBodyState(
   BodyHandle handle) const
{
   // Check if the body is pending creation, 
   // already exists in the physics world or
   // is pending destruction.
   const RigidBody* body = commandBuffer.tryGetPendingBodyCreate(handle);
   if (!body) {
      body = physicsStorage.tryGetBody(handle);
   }
   if (!body || commandBuffer.isBodyPendingDestroy(handle)) {
      return std::nullopt;
   }

   BodyState state;
   state.pose = body->pose;
   state.scale = body->scale;
   state.linearVelocity = body->linearVelocity;
   state.angularVelocity = body->angularVelocity;
   state.type = body->type;
   state.reportContacts = body->reportContacts;
   state.mass = body->mass;
   state.allowGravity = body->allowGravity;
   state.bounds = toPublicBounds(body->aabb);
   
   const BodyColliderRef& ref = body->colliderRef;

   if (ref.isCompound()) {
      state.colliders =
         physicsStorage.tryGetCompound(ref.compound)->colliders;
   }
   else {
      state.colliders.push_back(ref.single);
   }


   if (body->type == BodyType::Dynamic) {
      const SleepState& sleepState =
         physicsStorage.getSleepState(body->sleepStateHandle);
      state.asleep = sleepState.isAsleep();
      state.allowSleep = sleepState.canSleep();
   }

   return state;
}

std::optional<ColliderState>
Engine::getColliderState(
   ColliderHandle handle) const
{
   // Check if the collider is pending creation, 
   // already exists in the physics world or
   // is pending destruction.
   const Collider* collider =
      commandBuffer.tryGetPendingColliderCreate(handle);
   if (!collider) {
      collider = physicsStorage.tryGetCollider(handle);
   }
   if (!collider || commandBuffer.isColliderPendingDestroy(handle)) {
      return std::nullopt;
   }

   if (!collider ||
      commandBuffer.isColliderPendingDestroy(handle) ||
      commandBuffer.isBodyPendingDestroy(collider->bodyHandle)) {
      return std::nullopt;
   }

   ColliderState state;
   state.body = collider->bodyHandle;
   state.localPose = collider->localPose;
   state.localScale = collider->localScale;
   state.worldPose = collider->worldPose;
   state.worldScale = collider->worldScale;
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
          box.scale
      };
   }
   else {
      const Sphere& sphere = std::get<Sphere>(collider->shape);
      state.shape = SphereGeometry{
          sphere.centerWorld,
          sphere.radiusWorld
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
