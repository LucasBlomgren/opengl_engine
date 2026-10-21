#include "pch.h"
#include "narrowphase_manager.h"

namespace physics::internal {

namespace {

std::optional<ShapePairKind> classifyShapePair(
   const Collider& colliderA,
   const Collider& colliderB)
{
   const bool boxBox =
      colliderA.type == ColliderType::BOX &&
      colliderB.type == ColliderType::BOX;

   if (boxBox) {
      return ShapePairKind::BoxBox;
   }

   const bool boxSphere =
      (colliderA.type == ColliderType::BOX &&
         colliderB.type == ColliderType::SPHERE) ||
      (colliderA.type == ColliderType::SPHERE &&
         colliderB.type == ColliderType::BOX);

   if (boxSphere) {
      return ShapePairKind::BoxSphere;
   }

   const bool sphereSphere =
      colliderA.type == ColliderType::SPHERE &&
      colliderB.type == ColliderType::SPHERE;

   if (sphereSphere) {
      return ShapePairKind::SphereSphere;
   }

   return std::nullopt;
}

}


//=======================================================
//              Initialization
//=======================================================
void NarrowphaseManager::init(
   PhysicsStorage* storage,
   CollisionManifold* collisionManifold,
   std::vector<DebugSpeculativeContact>* debugSpeculativeContacts,
   std::unordered_map<size_t, Contact>* contactCache)
{
   this->physicsStorage = storage;
   this->collisionManifold = collisionManifold;
   this->debugSpeculativeContacts = debugSpeculativeContacts;
   this->contactCache = contactCache;
}

void NarrowphaseManager::clear() {
   externalContacts.clear();
   terrainTriCandidates.clear();
   SAT_resultsList.clear();
}

//=======================================================
//    Narrow phase
//=======================================================
void NarrowphaseManager::narrowPhase(
   const PairBatch& pairs,
   ContactBatch& batch,
   float dt)
{
   externalContacts.clear();
   normalHitPairs.clear();
   pendingSweepHits.clear();

   // Terrain
   for (const TerrainPair& pair : pairs.terrainPairs) {
      processTerrainPair(pair, batch);
   }

   // Dynamic
   double start = glfwGetTime();
   for (const DynamicPair& pair : pairs.dynamicPairs) {
      processDynamicPair(pair, batch);
   }

   // Speculative terrain
   for (const SpeculativeTerrainPair& pair : pairs.speculativeTerrainPairs) {
      processSpeculativeTerrainPair(pair, dt);
   }

   // Speculative dynamic
   start = glfwGetTime();
   for (const SpeculativeDynamicPair& pair : pairs.speculativeDynamicPairs) {
      processSpeculativePair(pair, dt);
   }

   flushPendingSweepHits(batch);
}

//=======================================================
//    Dynamic Pair     
//=======================================================
void NarrowphaseManager::processDynamicPair(
   const DynamicPair& pair,
   ContactBatch& batch)
{
   RigidBody& bodyA = physicsStorage->getBody(pair.bodyA);
   RigidBody& bodyB = physicsStorage->getBody(pair.bodyB);

   Collider& colliderA = physicsStorage->getCollider(pair.colliderA);
   Collider& colliderB = physicsStorage->getCollider(pair.colliderB);

   const auto shapePair = classifyShapePair(colliderA, colliderB);
   if (!shapePair) return;

   ResolvedColliderPair resolved{
      ColliderEndpointRef{
         pair.bodyA, pair.colliderA, &bodyA, &colliderA
      },
      ColliderEndpointRef{
         pair.bodyB, pair.colliderB, &bodyB, &colliderB
      },
      *shapePair
   };

   std::optional<OverlapHit> hit;

   switch (*shapePair) {
   case ShapePairKind::BoxBox:
      hit = tryBoxBox(std::move(resolved));
      break;

   case ShapePairKind::BoxSphere:
      hit = tryBoxSphere(std::move(resolved));
      break;

   case ShapePairKind::SphereSphere:
      hit = trySphereSphere(std::move(resolved));
      break;
   }

   if (!hit) return;

   PairKey key = makeColliderPairKey(
      hit->pair.a.colliderHandle,
      hit->pair.b.colliderHandle
   );

   normalHitPairs.insert(key);
   emitRigidContact(batch, *hit);
}

//=======================================================
//    Speculative pair
//=======================================================
void NarrowphaseManager::processSpeculativePair(
   const SpeculativeDynamicPair& pair,
   float dt)
{
   RigidBody& bodyA = physicsStorage->getBody(pair.bodyA);
   RigidBody& bodyB = physicsStorage->getBody(pair.bodyB);

   if (bodyA.type == BodyType::Kinematic &&
      bodyB.type == BodyType::Kinematic)
      return;

   Collider& colliderA = physicsStorage->getCollider(pair.colliderA);
   Collider& colliderB = physicsStorage->getCollider(pair.colliderB);

   const auto shapePair = classifyShapePair(colliderA, colliderB);
   if (!shapePair) return;

   PairKey key = makeColliderPairKey(
      pair.colliderA,
      pair.colliderB
   );

   if (normalHitPairs.contains(key))
      return;

   ResolvedColliderPair resolved{
      ColliderEndpointRef{
         pair.bodyA, pair.colliderA, &bodyA, &colliderA
      },
      ColliderEndpointRef{
         pair.bodyB, pair.colliderB, &bodyB, &colliderB
      },
      *shapePair
   };

   std::optional<SweepHit> hit;

   switch (*shapePair) {
   case ShapePairKind::BoxBox:
      hit = trySpeculativeBoxBox(
         std::move(resolved), dt, pair.sweepOwner
      );
      break;

   case ShapePairKind::BoxSphere:
      hit = trySpeculativeBoxSphere(
         std::move(resolved), dt, pair.sweepOwner
      );
      break;

   case ShapePairKind::SphereSphere:
      hit = trySpeculativeSphereSphere(
         std::move(resolved), dt, pair.sweepOwner
      );
      break;
   }

   if (!hit) return;

   pendingSweepHits.emplace_back(std::move(*hit));
}
//=======================================================
//     Normal Terrain pair processing
//=======================================================
void NarrowphaseManager::processTerrainPair(
   const TerrainPair& pair,
   ContactBatch& batch)
{
   RigidBody& body = physicsStorage->getBody(pair.body);

   if (body.type == BodyType::Kinematic && !body.reportContacts)
      return;

   if (body.type == BodyType::Dynamic) {
      SleepState& sleepState =
         physicsStorage->getSleepState(body.sleepStateHandle);

      if (sleepState.isAsleep())
         return;
   }

   Collider& collider =
      physicsStorage->getCollider(pair.collider);

   SAT_resultsList.clear();
   SAT_resultsList.reserve(pair.tris.size());

   switch (collider.type) {
   case ColliderType::BOX:
      processTerrainTriBox(
         batch, pair.body, &collider, &body, pair.tris
      );
      break;

   case ColliderType::SPHERE:
      processTerrainTriSphere(
         batch, pair.body, &collider, &body, pair.tris
      );
      break;
   }
}

//=======================================================
//     Speculative Terrain pair processing
//=======================================================
void NarrowphaseManager::processSpeculativeTerrainPair(
   const SpeculativeTerrainPair& pair,
   float dt)
{
   RigidBody& body = physicsStorage->getBody(pair.body);

   if (body.type == BodyType::Kinematic)
      return;

   Collider& collider =
      physicsStorage->getCollider(pair.collider);

   ColliderEndpointRef colliderRef{
      pair.body,
      pair.collider,
      &body,
      &collider
   };

   for (Tri* tri : pair.tris) {
      std::optional<TerrainSweepHit> hit;

      switch (collider.type) {
      case ColliderType::BOX:
         hit = trySpeculativeBoxTriangle(
            colliderRef, tri, dt, pair.body
         );
         break;

      case ColliderType::SPHERE:
         hit = trySpeculativeSphereTriangle(
            colliderRef, tri, dt, pair.body
         );
         break;
      }

      if (!hit)
         continue;

      pendingSweepHits.emplace_back(std::move(*hit));
   }
}

//=======================================================
//     Collider and body handle packing for pair keys
//=======================================================
uint64_t NarrowphaseManager::packColliderHandle(ColliderHandle h) {
   return (uint64_t(h.slot) << 32) | uint64_t(h.gen);
}
uint64_t NarrowphaseManager::packBodyHandle(BodyHandle h) {
   return (uint64_t(h.slot) << 32) | uint64_t(h.gen);
}
PairKey NarrowphaseManager::makeColliderPairKey(ColliderHandle a, ColliderHandle b) {
   uint64_t pa = packColliderHandle(a);
   uint64_t pb = packColliderHandle(b);

   if (pa > pb) {
      std::swap(pa, pb);
   }

   return { pa, pb };
}

//=======================================================
//     Runtime data generation
//=======================================================
ContactRuntime NarrowphaseManager::makeRuntimeData(
   RigidBody* bodyA, RigidBody* bodyB,
   Collider* colliderA, Collider* colliderB) const
{
   ContactRuntime rt;
   rt.bodyA = bodyA;
   rt.bodyB = bodyB;
   rt.colliderA = colliderA;
   rt.colliderB = colliderB;
   return rt;
}

ContactRuntime NarrowphaseManager::makeRuntimeData(
   RigidBody* bodyA,
   Collider* colliderA) const
{
   ContactRuntime rt;
   rt.bodyA = bodyA;
   rt.colliderA = colliderA;
   return rt;
}

//=======================================================
//     Terrain tri candidate collection
//=======================================================
void NarrowphaseManager::collectTerrainTriCandidates(
   Collider& collider,
   const std::vector<Tri*>& inputTris,
   std::vector<Tri*>& outCandidates)
{
   outCandidates.clear();

   const AABB& colAABB = collider.getAABB();
   for (Tri* tri : inputTris) {
      if (tri->getAABB().intersects(colAABB)) {
         outCandidates.push_back(tri);
      }
   }
}


}
