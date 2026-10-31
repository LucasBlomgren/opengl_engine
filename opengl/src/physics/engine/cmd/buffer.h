#pragma once

#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>
#include <glm/vec3.hpp>

#include "physics/public/collider_desc.h"
#include "physics/public/handles.h"
#include "physics/public/physics_types.h"

#include "physics/storage/physics_storage.h"

#include "physics/bodies/rigidbody.h"
#include "physics/collider/collider.h"

namespace physics::internal::cmd {

class Buffer {
public:
   //=========================================
   // Rigid body mutation commands
   //=========================================
   struct ApplyLinearImpulse {
      BodyHandle body;
      glm::vec3 impulse;
   };

   struct SetLinearVelocity {
      BodyHandle body;
      glm::vec3 velocity;
   };

   struct SetAngularVelocity {
      BodyHandle body;
      glm::vec3 velocity;
   };

   struct SetKinematicTarget {
      BodyHandle body;
      Pose target;
   };

   struct SetBodyTransform {
      BodyHandle body;
      Pose pose;
   };

   struct SetBodySleepState {
      BodyHandle body;
      bool asleep;
   };

   struct SetBodyType {
      BodyHandle body;
      BodyType type;
   };

   struct SetBodyReportContacts {
      BodyHandle body;
      bool reportContacts;
   };

   struct SetBodyMass {
      BodyHandle body;
      float mass;
   };

   struct SetBodyAllowGravity {
      BodyHandle body;
      bool allowGravity;
   };

   struct SetBodyAllowSleep {
      BodyHandle body;
      bool allowSleep;
   };

   //=========================================
   // Collider mutation commands
   //=========================================
   struct SetColliderLocalPose {
      ColliderHandle collider;
      Pose localPose;
   };

   struct SetColliderLocalTransform {
      ColliderHandle collider;
      Pose localPose;
      glm::vec3 localScale;
   };

   struct SetColliderShape {
      ColliderHandle collider;
      ColliderShapeDesc shape;
   };

   struct SetColliderEnabled {
      ColliderHandle collider;
      bool enabled;
   };

   struct SetColliderTrigger {
      ColliderHandle collider;
      bool isTrigger;
   };

   //=========================================
   // Scene-wide mutation commands
   //=========================================
   struct SleepAllObjects {};
   struct AwakenAllObjects {};

   using Mutation = std::variant<
      ApplyLinearImpulse,
      SetLinearVelocity,
      SetAngularVelocity,
      SetKinematicTarget,
      SetBodyTransform,
      SetBodySleepState,
      SetBodyType,
      SetBodyReportContacts,
      SetBodyMass,
      SetBodyAllowGravity,
      SetBodyAllowSleep,
      SetColliderLocalPose,
      SetColliderLocalTransform,
      SetColliderShape,
      SetColliderEnabled,
      SetColliderTrigger,
      SleepAllObjects,
      AwakenAllObjects
   >;

   //=========================================
   // Complete command batch
   //=========================================
   struct Batch {
      std::unordered_map<MeshHandle, Mesh> meshCreates;
      std::unordered_map<BodyHandle, RigidBody> bodyCreates;
      std::unordered_map<ColliderHandle, Collider> colliderCreates;
      std::unordered_set<MeshHandle> meshDestroys;
      std::unordered_set<BodyHandle> bodyDestroys;
      std::unordered_map<ColliderHandle, BodyHandle> colliderDestroys;
      std::vector<Mutation> mutations;

      bool empty() const noexcept {
         return 
            meshCreates.empty() &&
            bodyCreates.empty() &&
            colliderCreates.empty() &&
            meshDestroys.empty() &&
            bodyDestroys.empty() &&
            colliderDestroys.empty() &&
            mutations.empty();
      }
   };

   //=========================================
   // Storage management
   //=========================================
   void reserve(size_t bodyCount, size_t colliderCount, size_t mutationCount);
   void clear(PhysicsStorage& physicsStorage);
   [[nodiscard]] Batch take();

   //=========================================
   // Lifecycle recording
   //=========================================
   void recordMeshCreate(MeshHandle mesh, Mesh&& meshData);
   void recordBodyCreate(BodyHandle body, RigidBody&& bodyData);
   void recordColliderCreate(ColliderHandle collider, Collider&& colliderData);

   bool recordMeshDestroy(MeshHandle mesh, PhysicsStorage& physicsStorage);
   bool recordBodyDestroy(BodyHandle body, PhysicsStorage& physicsStorage);
   bool recordColliderDestroy(ColliderHandle collider, PhysicsStorage& physicsStorage);

   RigidBody* tryGetPendingBodyCreate(BodyHandle body);
   const RigidBody* tryGetPendingBodyCreate(BodyHandle body) const;

   Collider* tryGetPendingColliderCreate(ColliderHandle collider);
   const Collider* tryGetPendingColliderCreate(ColliderHandle collider) const;

   [[nodiscard]] bool isBodyPendingDestroy(BodyHandle body) const;
   [[nodiscard]] bool isColliderPendingDestroy(ColliderHandle collider) const;

   //=========================================
   // Rigid body command recording
   //=========================================
   void recordApplyLinearImpulse(BodyHandle body, const glm::vec3& impulse);
   void recordSetLinearVelocity(BodyHandle body, const glm::vec3& velocity);
   void recordSetAngularVelocity(BodyHandle body, const glm::vec3& velocity);
   void recordSetKinematicTarget(BodyHandle body, const Pose& target);

   void recordSetBodyTransform(
      BodyHandle body,
      const Pose& pose);

   void recordSetBodySleepState(BodyHandle body, bool asleep);
   void recordSetBodyType(BodyHandle body, BodyType type);
   void recordSetBodyReportContacts(BodyHandle body, bool reportContacts);

   void recordSetBodyMass(BodyHandle body, float mass);
   void recordSetBodyAllowGravity(BodyHandle body, bool allowGravity);
   void recordSetBodyAllowSleep(BodyHandle body, bool allowSleep);

   //=========================================
   // Collider command recording
   //=========================================
   void recordSetColliderLocalPose(ColliderHandle collider, const Pose& localPose);

   void recordSetColliderLocalTransform(
      ColliderHandle collider,
      const Pose& localPose,
      const glm::vec3& localScale);

   void recordSetColliderShape(
      ColliderHandle collider,
      const ColliderShapeDesc& shape);

   void recordSetColliderEnabled(ColliderHandle collider, bool enabled);
   void recordSetColliderTrigger(ColliderHandle collider, bool isTrigger);

   //=========================================
   // Scene-wide command recording
   //=========================================
   void recordSleepAllObjects();
   void recordAwakenAllObjects();

private:
   void absorbColliderDestroysForBody(
      BodyHandle body,
      std::unordered_set<ColliderHandle>& removedColliders);

   void removeMutationsTargeting(
      BodyHandle body,
      const std::unordered_set<ColliderHandle>& colliders);

   void removeMutationsTargeting(ColliderHandle collider);

   std::vector<Mutation> mutations;
   std::unordered_map<MeshHandle, Mesh> meshCreates;
   std::unordered_set<MeshHandle> meshDestroys;

   std::unordered_map<BodyHandle, RigidBody> bodyCreates;
   std::unordered_set<BodyHandle> bodyDestroys;

   std::unordered_map<ColliderHandle, Collider> colliderCreates;
   std::unordered_map<ColliderHandle, BodyHandle> colliderDestroys;
};

}
