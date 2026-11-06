#include "pch.h"

#include "physics_storage.h"

namespace physics::internal {

using BodyH = BodyHandle;
using ColH = ColliderHandle;
using CompH = CompoundHandle;
using MotionH = MotionStateHandle;
using SleepH = SleepStateHandle;
using MeshH = MeshHandle;
using HullH = HullHandle;

using MeshMap = SlotMap<MeshShape, MeshHandle>;
using HullMap = SlotMap<Hull, HullHandle>;
using BodyMap = SlotMap<RigidBody, BodyHandle>;
using MotionMap = SlotMap<MotionState, MotionStateHandle>;
using SleepMap = SlotMap<SleepState, SleepStateHandle>;
using ColliderMap = SlotMap<Collider, ColliderHandle>;
using CompoundMap = SlotMap<Compound, CompoundHandle>;

using PS = PhysicsStorage;


template bool PhysicsStorage::isAlive<MotionStateTag>(
   MotionStateHandle) const;

template bool PhysicsStorage::isAlive<SleepStateTag>(
   SleepStateHandle) const;

//===========================================
// Clear all data
//===========================================
void PS::clear() {
   hulls.clear();
   meshes.clear();
   colliders.clear();
   bodies.clear();
   motionStates.clear();
   sleepStates.clear();
   colliderId = 0;
   rigidBodyId = 0;
}

//===========================================
// Get data by handle
//===========================================
Hull& PS::getHull(HullH h) { return hulls.get(h); }
Hull* PS::tryGetHull(HullH h, SourceLoc c) { return hulls.try_get(h, c); }
const Hull* PS::tryGetHull(HullH h, SourceLoc c) const { return hulls.try_get(h, c); }

MeshShape& PS::getMesh(MeshH h) { return meshes.get(h); }
MeshShape* PS::tryGetMesh(MeshH h, SourceLoc c) { return meshes.try_get(h, c); }
const MeshShape* PS::tryGetMesh(MeshH h, SourceLoc c) const { return meshes.try_get(h, c); }

RigidBody& PS::getBody(BodyH h) { return bodies.get(h); }
RigidBody* PS::tryGetBody(BodyH h, SourceLoc c) { return bodies.try_get(h, c); }
const RigidBody* PS::tryGetBody(BodyH h, SourceLoc c) const { return bodies.try_get(h, c); }

MotionState& PS::getMotionState(MotionH h) { return motionStates.get(h); }
const MotionState& PS::getMotionState(MotionH h) const { return motionStates.get(h); }

SleepState& PS::getSleepState(SleepH h) { return sleepStates.get(h); }
const SleepState& PS::getSleepState(SleepH h) const { return sleepStates.get(h); }
SleepState* PS::tryGetSleepState(SleepH h, SourceLoc c) { return sleepStates.try_get(h, c); }

Collider& PS::getCollider(ColH h) { return colliders.get(h); }
Collider* PS::tryGetCollider(ColH h, SourceLoc c) { return colliders.try_get(h, c); }
const Collider* PS::tryGetCollider(ColH h, SourceLoc c) const { return colliders.try_get(h, c); }

Compound& PS::getCompound(CompH h) { return compounds.get(h); }
Compound* PS::tryGetCompound(CompH h, SourceLoc c) { return compounds.try_get(h, c); }
const Compound* PS::tryGetCompound(CompH h, SourceLoc c) const { return compounds.try_get(h, c); }

//===========================================
// Get storage
//===========================================
HullMap& PS::hullStorage() { return hulls; }
const HullMap& PS::hullStorage() const { return hulls; }

MeshMap& PS::meshStorage() { return meshes; }
const MeshMap& PS::meshStorage() const { return meshes; }

BodyMap& PS::bodyStorage() { return bodies; }
const BodyMap& PS::bodyStorage() const { return bodies; }

MotionMap& PS::motionStatesStorage() { return motionStates; }
const MotionMap& PS::motionStatesStorage() const { return motionStates; }

SleepMap& PS::sleepStatesStorage() { return sleepStates; }
const SleepMap& PS::sleepStatesStorage() const { return sleepStates; }

ColliderMap& PS::colliderStorage() { return colliders; }
const ColliderMap& PS::colliderStorage() const { return colliders; }

CompoundMap& PS::compoundStorage() { return compounds; }
const CompoundMap& PS::compoundStorage() const { return compounds; }

//===========================================
// Get Body AABB
//===========================================
AABB& PS::getAABB(const BodyHandle h) 
{
   RigidBody& body = getBody(h);
   BodyColliderRef& ref = body.colliderRef;

   if (ref.isSingle()) {
      return getCollider(ref.single).getAABB();
   }
   else if (ref.isCompound()) {
      return getCompound(ref.compound).aabb;
   }
}

//===========================================
// Reservation
//===========================================
BodyHandle PS::reserveBodyHandle() { return bodies.reserve(); }
ColliderHandle PS::reserveColliderHandle() { return colliders.reserve(); }
CompoundHandle PS::reserveCompoundHandle() { return compounds.reserve(); }
MeshHandle PS::reserveMeshHandle() { return meshes.reserve(); }
HullHandle PS::reserveHullHandle() { return hulls.reserve(); }

void PS::releaseBodyReservation(BodyH h) { bodies.release_reserved(h); }
void PS::releaseColliderReservation(ColH h) { colliders.release_reserved(h); }
void PS::releaseCompoundReservation(CompH h) { compounds.release_reserved(h); }
void PS::releaseMeshReservation (MeshH h) { meshes.release_reserved(h); }
void PS::releaseHullReservation(HullH h) { hulls.release_reserved(h); }

//===========================================
// Creation
//===========================================
Hull* PS::commitHull(HullHandle h, Hull&& m) {
   return hulls.create_reserved(h, std::move(m));
}
MeshShape* PS::commitMesh(MeshHandle h, MeshShape&& m) {
   return meshes.create_reserved(h, std::move(m));
}
RigidBody* PS::commitBody(BodyH h, RigidBody&& b) {
   b.id = rigidBodyId++;
   return bodies.create_reserved(h, std::move(b));
}
Collider* PS::commitCollider(ColH h, Collider&& c) {
   c.id = colliderId++;
   return colliders.create_reserved(h, std::move(c));
}
Compound* PS::commitCompound(CompH h, Compound&& c) {
   return compounds.create_reserved(h, std::move(c));
}
MotionH PS::commitMotionState(MotionState&& m) {
   return motionStates.create(std::move(m));
}
SleepStateHandle PS::commitSleepState(SleepState&& s) {
   return sleepStates.create(std::move(s));
}

//===========================================
// Deletion
//===========================================
void PS::destroyHull(HullH h) { hulls.destroy(h); }
void PS::destroyMesh(MeshH h) { meshes.destroy(h); }
void PS::destroyBody(BodyH h) { bodies.destroy(h); }
void PS::destroyCollider(ColH h) { colliders.destroy(h); }
void PS::destroyCompound(CompH h) { compounds.destroy(h); }
void PS::destroyMotionState(MotionH h) { motionStates.destroy(h); }
void PS::destroySleepState(SleepH h) { sleepStates.destroy(h); }
}