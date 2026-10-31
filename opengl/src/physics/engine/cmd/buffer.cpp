#include "pch.h"

#include "physics/engine/cmd/buffer.h"
#include "physics/collider/body_collider_utils.h"

#include <algorithm>
#include <utility>

namespace physics::internal::cmd {

//=========================================
// Storage management
//=========================================
void Buffer::reserve(
   size_t bodyCount,
   size_t colliderCount,
   size_t mutationCount) {
   bodyCreates.reserve(bodyCount);
   bodyDestroys.reserve(bodyCount);
   colliderCreates.reserve(colliderCount);
   colliderDestroys.reserve(colliderCount);
   mutations.reserve(mutationCount);
}

void Buffer::clear(PhysicsStorage& physicsStorage) {
   for (const auto& entry : colliderCreates) {
      physicsStorage.releaseColliderReservation(entry.first);
   }

   for (const auto& entry : bodyCreates) {
      physicsStorage.releaseBodyReservation(entry.first);
   }

   bodyCreates.clear();
   colliderCreates.clear();
   bodyDestroys.clear();
   colliderDestroys.clear();
   mutations.clear();
}

Buffer::Batch Buffer::take() {
   Batch batch;

   auto transferNodes = []<class AssociativeContainer>(
      AssociativeContainer & source,
      AssociativeContainer & destination)
   {
      destination.reserve(
         destination.size() + source.size()
      );

      while (!source.empty()) {
         destination.insert(
            source.extract(source.begin())
         );
      }
   };

   transferNodes(meshCreates, batch.meshCreates);
   transferNodes(bodyCreates, batch.bodyCreates);
   transferNodes(colliderCreates, batch.colliderCreates);

   transferNodes(meshDestroys, batch.meshDestroys);
   transferNodes(bodyDestroys, batch.bodyDestroys);
   transferNodes(colliderDestroys, batch.colliderDestroys);

   batch.mutations.reserve(mutations.size());
   for (Mutation& mutation : mutations) {
      batch.mutations.push_back(std::move(mutation));
   }

   mutations.clear();

   return batch;
}

//=========================================
// Lifecycle create
//=========================================
void Buffer::recordMeshCreate(
   MeshHandle mesh,
   Mesh&& meshData)
{
   meshCreates.insert_or_assign(
      mesh,
      std::move(meshData)
   );
}

void Buffer::recordBodyCreate(
   BodyHandle body,
   RigidBody&& rigidBody)
{
   bodyCreates.insert_or_assign(
      body,
      std::move(rigidBody)
   );
}

void Buffer::recordColliderCreate(
   ColliderHandle collider,
   Collider&& colliderData)
{
   colliderCreates.insert_or_assign(
      collider,
      std::move(colliderData)
   );
}

//=========================================
// Lifecycle destroy
//=========================================
bool Buffer::recordMeshDestroy(
   MeshHandle mesh, 
   PhysicsStorage& physicsStorage)
{
   if (meshCreates.contains(mesh)) {
      meshCreates.erase(mesh);
      physicsStorage.releaseMeshReservation(mesh);

      return false;
   }

   return meshDestroys.insert(mesh).second;
}

bool Buffer::recordBodyDestroy(
   BodyHandle body,
   PhysicsStorage& physicsStorage)
{
   if (bodyDestroys.contains(body)) {
      return false;
   }

   std::unordered_set<ColliderHandle> removedColliders;

   // Mark the body and its colliders for destruction
   // (if it exists in the active world)
   auto pendingCreate = bodyCreates.find(body);
   if (pendingCreate == bodyCreates.end()) {
      RigidBody* activeBody = physicsStorage.tryGetBody(body);

      if (!activeBody) {
         return false;
      }

      bodyDestroys.insert(body);

      forEachBodyCollider(
         physicsStorage,
         activeBody->colliderRef,
         [&](ColliderHandle colH) {
            removedColliders.insert(colH);
         }
      );
   }

   // Remove any pending collider creates that are associated with this body
   for (auto collider = colliderCreates.begin();
      collider != colliderCreates.end();) {

      if (collider->second.bodyHandle != body) {
         ++collider;
         continue;
      }

      const ColliderHandle colliderHandle = collider->first;

      removedColliders.insert(colliderHandle);
      physicsStorage.releaseColliderReservation(colliderHandle);

      collider = colliderCreates.erase(collider);
   }

   // Absorb any pending collider destroys that are associated with this body
   for (auto collider = colliderDestroys.begin();
      collider != colliderDestroys.end();) {
      if (collider->second != body) {
         ++collider;
         continue;
      }

      removedColliders.insert(collider->first);
      collider = colliderDestroys.erase(collider);
   }

   // Remove any pending mutations that target this body or its colliders
   removeMutationsTargeting(
      body,
      removedColliders
   );

   // If the body was pending creation, remove it from the pending creates
   // and release its motion state and body reservation
   if (pendingCreate != bodyCreates.end()) {
      MotionStateHandle motion = pendingCreate->second.motionStateHandle;
      SleepStateHandle sleep = pendingCreate->second.sleepStateHandle;

      bodyCreates.erase(pendingCreate);

      physicsStorage.destroyMotionState(motion);
      physicsStorage.destroySleepState(sleep);
      physicsStorage.releaseBodyReservation(body);
   }

   return true;
}

bool Buffer::recordColliderDestroy(
   ColliderHandle collider,
   PhysicsStorage& physicsStorage)
{
   if (colliderDestroys.contains(collider)) {
      return false;
   }

   auto create = colliderCreates.find(collider);

   // If the collider is pending creation, remove it from the 
   // pending creates and release its reservation
   if (create != colliderCreates.end()) {
      const BodyHandle parent = create->second.bodyHandle;

      if (RigidBody* pendingBody = tryGetPendingBodyCreate(parent))
      {
         BodyColliderRef& ref = pendingBody->colliderRef;

         if (ref.isSingle()) {
            if (ref.single == collider) {
               ref = BodyColliderRef{};
            }
         }
         else if (ref.isCompound()) {
            Compound& compound =
               physicsStorage.getCompound(ref.compound);

            std::erase(compound.colliders, collider);

            if (compound.colliders.empty()) {
               // Frigör compound-objektet enligt din poolhantering.
               ref = BodyColliderRef{};
            }
         }
      }

      colliderCreates.erase(create);
      removeMutationsTargeting(collider);
      physicsStorage.releaseColliderReservation(collider);
      return true;
   }

   // If the collider is not active or its parent body is pending 
   // destruction, don't destroy it (it will be destroyed when the parent body is destroyed)
   const Collider* activeCollider =
      physicsStorage.tryGetCollider(collider);

   if (!activeCollider ||
      bodyDestroys.contains(activeCollider->bodyHandle)) {
      return false;
   }

   // If the collider is active, mark it for destruction and 
   // remove any pending mutations that target it
   auto [it, inserted] = colliderDestroys.emplace(
      collider,
      activeCollider->bodyHandle
   );

   if (inserted) {
      removeMutationsTargeting(collider);
   }

   return inserted;
}

void Buffer::removeMutationsTargeting(
   BodyHandle body,
   const std::unordered_set<ColliderHandle>& colliders)
{
   auto targetsBody = [](
      const Buffer::Mutation& mutation,
      BodyHandle body)
      {
         return std::visit([body](const auto& command) {
            if constexpr (requires { command.body; }) {
               return command.body == body;
            }
            else {
               return false;
            }
            }, mutation);
      };

   auto targetsCollider = [](
      const Buffer::Mutation& mutation,
      const std::unordered_set<ColliderHandle>& colliders)
      {
         return std::visit([&colliders](const auto& command) {
            if constexpr (requires { command.collider; }) {
               return colliders.contains(command.collider);
            }
            else {
               return false;
            }
            }, mutation);
      };

   // Remove any pending mutations that target this body or its colliders
   std::erase_if(
      mutations,
      [&](const Mutation& mutation) {
         return targetsBody(mutation, body) ||
            targetsCollider(mutation, colliders);
      }
   );
}

void Buffer::removeMutationsTargeting(
   ColliderHandle collider)
{
   auto targetsCollider = [](
      const Buffer::Mutation& mutation,
      ColliderHandle collider)
      {
         return std::visit([collider](const auto& command) {
            if constexpr (requires { command.collider; }) {
               return command.collider == collider;
            }
            else {
               return false;
            }
            }, mutation);
      };

   // Remove any pending mutations that target this collider
   std::erase_if(
      mutations,
      [collider, &targetsCollider](const Mutation& mutation) {
         return targetsCollider(mutation, collider);
      }
   );
}

//=========================================
// Pending create/destroy queries
//=========================================
RigidBody* Buffer::tryGetPendingBodyCreate(BodyHandle body) {
   auto found = bodyCreates.find(body);
   return found != bodyCreates.end() ? &found->second : nullptr;
}
const RigidBody* Buffer::tryGetPendingBodyCreate(BodyHandle body) const {
   auto found = bodyCreates.find(body);
   return found != bodyCreates.end() ? &found->second : nullptr;
}

Collider* Buffer::tryGetPendingColliderCreate(ColliderHandle collider) {
   auto found = colliderCreates.find(collider);
   return found != colliderCreates.end() ? &found->second : nullptr;
}
const Collider* Buffer::tryGetPendingColliderCreate(ColliderHandle collider) const {
   auto found = colliderCreates.find(collider);
   return found != colliderCreates.end() ? &found->second : nullptr;
}

bool Buffer::isBodyPendingDestroy(
   BodyHandle body) const {
   return bodyDestroys.contains(body);
}
bool Buffer::isColliderPendingDestroy(
   ColliderHandle collider) const {
   return colliderDestroys.contains(collider);
}

//=========================================
// Rigid body command recording
//=========================================
void Buffer::recordApplyLinearImpulse(
   BodyHandle body,
   const glm::vec3& impulse) {
   mutations.emplace_back(ApplyLinearImpulse{ body, impulse });
}

void Buffer::recordSetLinearVelocity(
   BodyHandle body,
   const glm::vec3& velocity) {
   mutations.emplace_back(SetLinearVelocity{ body, velocity });
}

void Buffer::recordSetAngularVelocity(
   BodyHandle body,
   const glm::vec3& velocity) {
   mutations.emplace_back(SetAngularVelocity{ body, velocity });
}

void Buffer::recordSetKinematicTarget(
   BodyHandle body,
   const Pose& target) {
   mutations.emplace_back(SetKinematicTarget{ body, target });
}

void Buffer::recordSetBodyTransform(
   BodyHandle body,
   const Pose& pose) {
   mutations.emplace_back(SetBodyTransform{ body, pose });
}

void Buffer::recordSetBodySleepState(
   BodyHandle body,
   bool asleep) {
   mutations.emplace_back(SetBodySleepState{ body, asleep });
}

void Buffer::recordSetBodyType(
   BodyHandle body,
   BodyType type) {
   mutations.emplace_back(SetBodyType{ body, type });
}

void Buffer::recordSetBodyReportContacts(
   BodyHandle body,
   bool reportContacts) {
   mutations.emplace_back(SetBodyReportContacts{ body, reportContacts });
}

void Buffer::recordSetBodyMass(
   BodyHandle body,
   float mass) {
   mutations.emplace_back(SetBodyMass{ body, mass });
}

void Buffer::recordSetBodyAllowGravity(
   BodyHandle body,
   bool allowGravity) {
   mutations.emplace_back(SetBodyAllowGravity{ body, allowGravity });
}

void Buffer::recordSetBodyAllowSleep(
   BodyHandle body,
   bool allowSleep) {
   mutations.emplace_back(SetBodyAllowSleep{ body, allowSleep });
}

//=========================================
// Collider command recording
//=========================================
void Buffer::recordSetColliderLocalPose(
   ColliderHandle collider,
   const Pose& localPose) {
   mutations.emplace_back(SetColliderLocalPose{ collider, localPose });
}

void Buffer::recordSetColliderLocalTransform(
   ColliderHandle collider,
   const Pose& localPose,
   const glm::vec3& localScale) {
   mutations.emplace_back(SetColliderLocalTransform{
       collider,
       localPose,
       localScale
      });
}

void Buffer::recordSetColliderShape(
   ColliderHandle collider,
   const ColliderShapeDesc& shape) {
   mutations.emplace_back(SetColliderShape{ collider, shape });
}

void Buffer::recordSetColliderEnabled(
   ColliderHandle collider,
   bool enabled) {
   mutations.emplace_back(SetColliderEnabled{ collider, enabled });
}

void Buffer::recordSetColliderTrigger(
   ColliderHandle collider,
   bool isTrigger) {
   mutations.emplace_back(SetColliderTrigger{ collider, isTrigger });
}

//=========================================
// Scene-wide command recording
//=========================================
void Buffer::recordSleepAllObjects() {
   mutations.emplace_back(SleepAllObjects{});
}

void Buffer::recordAwakenAllObjects() {
   mutations.emplace_back(AwakenAllObjects{});
}
}