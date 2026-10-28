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
#include "physics/collider/collider_ref.h"

namespace physics::internal {

using SourceLoc = std::source_location;

class PhysicsStorage {
public:
   void clear();

   //===========================================
   // Reservation
   //===========================================
   BodyHandle reserveBodyHandle();
   ColliderHandle reserveColliderHandle();
   CompoundHandle reserveCompoundHandle();

   void releaseBodyReservation(BodyHandle);
   void releaseColliderReservation(ColliderHandle);
   void releaseCompoundReservation(CompoundHandle);

   //===========================================
   // Creation
   //===========================================
   RigidBody* commitBody(BodyHandle, RigidBody&&);
   Collider* commitCollider(ColliderHandle, Collider&&);
   Compound* commitCompound(CompoundHandle, Compound&&);
   MotionStateHandle commitMotionState(MotionState&&);
   SleepStateHandle commitSleepState(SleepState&&);

   //===========================================
   // Deletion
   //===========================================
   void destroyBody(BodyHandle);
   void destroyMotionState(MotionStateHandle);
   void destroySleepState(SleepStateHandle);
   void destroyCollider(ColliderHandle);
   void destroyCompound(CompoundHandle);

   //===========================================
   // Get data by handle
   //===========================================
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

private:
   int colliderId = 0;
   int rigidBodyId = 0;
   SlotMap<RigidBody, BodyHandle> bodies{ "RigidBody" };
   SlotMap<MotionState, MotionStateHandle> motionStates{ "MotionState" };
   SlotMap<SleepState, SleepStateHandle> sleepStates{ "SleepState" };
   SlotMap<Collider, ColliderHandle> colliders{ "Collider" };
   SlotMap<Compound, CompoundHandle> compounds{ "Compound" };

   SlotMap<Sphere, ShapeHandle> spheres{ "Sphere" };
   SlotMap<BoxData, ShapeHandle> boxes{ "Box" };

};
}