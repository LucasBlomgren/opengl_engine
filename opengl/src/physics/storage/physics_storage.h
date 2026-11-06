#pragma once

#include <source_location>
#include <unordered_map>

#include "slot_map.h"

#include "physics/public/handles.h"
#include "physics/bodies/rigidbody.h"
#include "physics/bodies/motion_state.h"
#include "physics/sleep/sleep_state.h"
#include "physics/collider/collider.h"
#include "physics/collider/compound_collider.h"
#include "physics/collider/shapes/mesh_shape.h"
#include "physics/collider/shapes/convex_hull.h"
#include "physics/collider/collider_ref.h"

namespace physics::internal {

using SourceLoc = std::source_location;

class PhysicsStorage {
public:
   void clear();

   //===========================================
   // Handle alive check
   //===========================================
   template<typename Tag>
   bool isAlive(HandleT<Tag> handle) const {
      if constexpr (std::is_same_v<Tag, BodyTag>)
         return bodies.alive(handle);
      else if constexpr (std::is_same_v<Tag, ColliderTag>)
         return colliders.alive(handle);
      else if constexpr (std::is_same_v<Tag, CompoundTag>)
         return compounds.alive(handle);
      else if constexpr (std::is_same_v<Tag, MotionStateTag>)
         return motionStates.alive(handle);
      else if constexpr (std::is_same_v<Tag, SleepStateTag>)
         return sleepStates.alive(handle);
      else if constexpr (std::is_same_v<Tag, MeshTag>)
         return meshes.alive(handle);
      else if constexpr (std::is_same_v<Tag, HullTag>)
         return hulls.alive(handle);
      else
         return false;
   }

   bool isAlive(const BodyColliderRef& ref) const {
      switch (ref.type) {
      case BodyColliderRef::Type::SINGLE:
         return isAlive(ref.single);
      case BodyColliderRef::Type::COMPOUND:
         return isAlive(ref.compound);
      default:
         return false;
      }
   }

   //===========================================
   // Reservation
   //===========================================
   BodyHandle reserveBodyHandle();
   ColliderHandle reserveColliderHandle();
   CompoundHandle reserveCompoundHandle();
   MeshHandle reserveMeshHandle();
   HullHandle reserveHullHandle();

   void releaseBodyReservation(BodyHandle);
   void releaseColliderReservation(ColliderHandle);
   void releaseCompoundReservation(CompoundHandle);
   void releaseMeshReservation(MeshHandle);
   void releaseHullReservation(HullHandle);

   //===========================================
   // Creation
   //===========================================
   MeshShape* commitMesh(MeshHandle, MeshShape&&);
   Hull* commitHull(HullHandle, Hull&&);
   RigidBody* commitBody(BodyHandle, RigidBody&&);
   Collider* commitCollider(ColliderHandle, Collider&&);
   Compound* commitCompound(CompoundHandle, Compound&&);
   MotionStateHandle commitMotionState(MotionState&&);
   SleepStateHandle commitSleepState(SleepState&&);

   //===========================================
   // Deletion
   //===========================================
   void destroyMesh(MeshHandle);
   void destroyHull(HullHandle);
   void destroyBody(BodyHandle);
   void destroyMotionState(MotionStateHandle);
   void destroySleepState(SleepStateHandle);
   void destroyCollider(ColliderHandle);
   void destroyCompound(CompoundHandle);

   //===========================================
   // Get data by handle
   //===========================================
   MeshShape& getMesh(MeshHandle);
   MeshShape* tryGetMesh(MeshHandle, SourceLoc c = SourceLoc::current());
   const MeshShape* tryGetMesh(MeshHandle, SourceLoc c = SourceLoc::current()) const;

   Hull& getHull(HullHandle);
   Hull* tryGetHull(HullHandle, SourceLoc c = SourceLoc::current());
   const Hull* tryGetHull(HullHandle, SourceLoc c = SourceLoc::current()) const;

   RigidBody& getBody(BodyHandle);
   RigidBody* tryGetBody(BodyHandle, SourceLoc c = SourceLoc::current());
   const RigidBody* tryGetBody(BodyHandle, SourceLoc c = SourceLoc::current()) const;

   MotionState& getMotionState(MotionStateHandle);
   const MotionState& getMotionState(MotionStateHandle) const;

   SleepState& getSleepState(SleepStateHandle);
   const SleepState& getSleepState(SleepStateHandle) const;
   SleepState* tryGetSleepState(SleepStateHandle, SourceLoc c = SourceLoc::current());

   Collider& getCollider(ColliderHandle);
   Collider* tryGetCollider(ColliderHandle, SourceLoc c = SourceLoc::current());
   const Collider* tryGetCollider(ColliderHandle, SourceLoc c = SourceLoc::current()) const;

   Compound& getCompound(CompoundHandle);
   Compound* tryGetCompound(CompoundHandle, SourceLoc c = SourceLoc::current());
   const Compound* tryGetCompound(CompoundHandle, SourceLoc c = SourceLoc::current()) const;

   //===========================================
   // Get storage
   //===========================================
   SlotMap<Hull, HullHandle>& hullStorage();
   const SlotMap<Hull, HullHandle>& hullStorage() const;

   SlotMap<MeshShape, MeshHandle>& meshStorage();
   const SlotMap<MeshShape, MeshHandle>& meshStorage() const;

   SlotMap<RigidBody, BodyHandle>& bodyStorage();
   const SlotMap<RigidBody, BodyHandle>& bodyStorage() const;

   SlotMap<MotionState, MotionStateHandle>& motionStatesStorage();
   const SlotMap<MotionState, MotionStateHandle>& motionStatesStorage() const;

   SlotMap<SleepState, SleepStateHandle>& sleepStatesStorage();
   const SlotMap<SleepState, SleepStateHandle>& sleepStatesStorage() const;

   SlotMap<Collider, ColliderHandle>& colliderStorage();
   const SlotMap<Collider, ColliderHandle>& colliderStorage() const;

   SlotMap<Compound, CompoundHandle>& compoundStorage();
   const SlotMap<Compound, CompoundHandle>& compoundStorage() const;

   //===========================================
   // Get Body AABB
   //===========================================
   AABB& getAABB(const BodyHandle h);

   //===========================================
   // Get registries
   //===========================================
   std::unordered_map<std::string, MeshHandle>& getMeshRegistry() { return meshRegistry; }
   const std::unordered_map<std::string, MeshHandle>& getMeshRegistry() const { return meshRegistry; }

   std::unordered_map<std::string, HullHandle>& getHullRegistry() { return hullRegistry; }
   const std::unordered_map<std::string, HullHandle>& getHullRegistry() const { return hullRegistry; }

private:
   int colliderId = 0;
   int rigidBodyId = 0;
   SlotMap<RigidBody, BodyHandle> bodies{ "RigidBody" };
   SlotMap<MotionState, MotionStateHandle> motionStates{ "MotionState" };
   SlotMap<SleepState, SleepStateHandle> sleepStates{ "SleepState" };
   SlotMap<Collider, ColliderHandle> colliders{ "Collider" };
   SlotMap<Compound, CompoundHandle> compounds{ "Compound" };
   SlotMap<Hull, HullHandle> hulls{ "Hulls" };
   SlotMap<MeshShape, MeshHandle> meshes{ "Mesh" };

   std::unordered_map<std::string, HullHandle> hullRegistry;
   std::unordered_map<std::string, MeshHandle> meshRegistry;
};
}