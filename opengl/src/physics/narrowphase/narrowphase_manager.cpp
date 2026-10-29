#include "pch.h"
#include "narrowphase_manager.h"

#include "physics/broadphase/broadphase_manager.h"

#include "physics/collider/body_collider_utils.h"

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
   const SweepFrame& sweepFrame,
   float dt)
{
   externalContacts.clear();
   normalHitPairs.clear();
   pendingSweepHits.clear();

   // Terrain
   for (const TerrainPair& pair : pairs.terrainPairs) {
      processTerrainPair(pair, batch);
   }

   // Speculative terrain
   for (const SpeculativeTerrainPair& pair : pairs.speculativeTerrainPairs) {
      processSpeculativeTerrainPair(pair, sweepFrame, dt);
   }

   // Dynamic
   double start = glfwGetTime();
   for (const DynamicPair& pair : pairs.dynamicPairs) {
      processDynamicPair(pair, batch);
   }

   // Speculative dynamic
   start = glfwGetTime();
   for (const SpeculativeDynamicPair& pair : pairs.speculativeDynamicPairs) {
      processSpeculativePair(pair, sweepFrame, dt);
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

   forEachBodyCollider(
      *physicsStorage,
      bodyA.colliderRef,
      [&](ColliderHandle colA)
      {
         Collider& colliderA = physicsStorage->getCollider(colA);

         if (!colliderA.enabled)
            return;

         forEachBodyCollider(
            *physicsStorage,
            bodyB.colliderRef,
            [&](ColliderHandle colB)
            {
               Collider& colliderB = physicsStorage->getCollider(colB);

               if (!colliderB.enabled)
                  return;

               // Broadphase filtering per collider
               if (!colliderA.getAABB().intersects(colliderB.getAABB()))
                  return;

               const auto shapePair =
                  classifyShapePair(colliderA, colliderB);

               if (!shapePair)
                  return;

               ResolvedColliderPair resolved{
                  ColliderEndpointRef{
                     pair.bodyA, colA, &bodyA, &colliderA
                  },
                  ColliderEndpointRef{
                     pair.bodyB, colB, &bodyB, &colliderB
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

               if (!hit)
                  return;

               PairKey key = makeColliderPairKey(
                  hit->pair.a.colliderHandle,
                  hit->pair.b.colliderHandle
               );

               normalHitPairs.insert(key);
               emitRigidContact(batch, *hit);
            }
         );
      }
   );
}

//=======================================================
//    Speculative pair
//=======================================================
void NarrowphaseManager::processSpeculativePair(
   const SpeculativeDynamicPair& pair,
   const SweepFrame& sweepFrame,
   float dt)
{
   RigidBody& bodyA = physicsStorage->getBody(pair.bodyA);
   RigidBody& bodyB = physicsStorage->getBody(pair.bodyB);

   if (bodyA.type == BodyType::Kinematic &&
      bodyB.type == BodyType::Kinematic)
      return;

   // Get speculative metadata
   auto getSweepInfo = [&](BodyHandle handle) -> const BodySweepInfo* {
      const BodySweepInfo& info = sweepFrame.infoByBodySlot[handle.slot];

      return info.bodyAABBIdx != -1 ? &info : nullptr;
      };

   const BodySweepInfo* infoA = getSweepInfo(pair.bodyA);
   const BodySweepInfo* infoB = getSweepInfo(pair.bodyB);

   // Step forward to next child even if AABB-test failed
   int childIdxA = 0;

   forEachBodyCollider(
      *physicsStorage,
      bodyA.colliderRef,
      [&](ColliderHandle colA)
      {
         Collider& colliderA = physicsStorage->getCollider(colA);

         if (!colliderA.enabled)
            return;

         // Swept AABB if speculative, otherwise tight AABB
         const AABB& aabbA = infoA
            ? sweepFrame.childSweptAABBs[infoA->childStart + childIdxA]
            : colliderA.getAABB();

         ++childIdxA;

         // Step forward to next child even if AABB-test failed
         int childIdxB = 0;

         forEachBodyCollider(
            *physicsStorage,
            bodyB.colliderRef,
            [&](ColliderHandle colB)
            {
               Collider& colliderB = physicsStorage->getCollider(colB);

               if (!colliderB.enabled)
                  return;

               const AABB& aabbB = infoB
                  ? sweepFrame.childSweptAABBs[infoB->childStart + childIdxB]
                  : colliderB.getAABB();

               ++childIdxB;

               // Midphase: reject before expensive SAT
               if (!aabbA.intersects(aabbB))
                  return;

               PairKey key = makeColliderPairKey(colA, colB);

               if (normalHitPairs.contains(key))
                  return;

               const auto shapePair =
                  classifyShapePair(colliderA, colliderB);

               if (!shapePair)
                  return;

               ResolvedColliderPair resolved{
                  ColliderEndpointRef{
                     pair.bodyA, colA, &bodyA, &colliderA
                  },
                  ColliderEndpointRef{
                     pair.bodyB, colB, &bodyB, &colliderB
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

               if (hit)
                  pendingSweepHits.emplace_back(std::move(*hit));
            }
         );
      }
   );
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

   const bool isCompound = body.colliderRef.isCompound();

   forEachBodyCollider(
      *physicsStorage,
      body.colliderRef,
      [&](ColliderHandle colliderH)
      {
         Collider& collider =
            physicsStorage->getCollider(colliderH);

         if (!collider.enabled)
            return;

         const std::vector<Tri*>* candidates = &pair.tris;

         // Midphase filtering for compounds
         if (isCompound) {
            terrainTriCandidates.clear();

            collectTerrainTriCandidates(
               collider,
               pair.tris,
               terrainTriCandidates
            );

            if (terrainTriCandidates.empty())
               return;

            candidates = &terrainTriCandidates;
         }

         SAT_resultsList.clear();
         SAT_resultsList.reserve(candidates->size());

         switch (collider.type) {
         case ColliderType::BOX:
            processTerrainTriBox(
               batch, pair.body, &collider, &body, *candidates
            );
            break;

         case ColliderType::SPHERE:
            processTerrainTriSphere(
               batch, pair.body, &collider, &body, *candidates
            );
            break;
         }
      }
   );
}

//=======================================================
//     Speculative Terrain pair processing
//=======================================================
void NarrowphaseManager::processSpeculativeTerrainPair(
   const SpeculativeTerrainPair& pair,
   const SweepFrame& sweepFrame,
   float dt)
{
   RigidBody& body = physicsStorage->getBody(pair.body);

   if (body.type == BodyType::Kinematic)
      return;

   const BodySweepInfo& info = sweepFrame.infoByBodySlot[pair.body.slot];

   // Step forward to next child even if AABB-test failed
   int childIdx = 0;

   forEachBodyCollider(
      *physicsStorage,
      body.colliderRef,
      [&](ColliderHandle colliderH)
      {
         Collider& collider =
            physicsStorage->getCollider(colliderH);

         if (!collider.enabled)
            return;

         const AABB& sweptAABB =
            sweepFrame.childSweptAABBs[info.childStart + childIdx];

         ++childIdx;

         ColliderEndpointRef colliderRef{
            pair.body,
            colliderH,
            &body,
            &collider
         };

         for (Tri* tri : pair.tris) 
         {
            // Midphase filtering
            if (!sweptAABB.intersects(tri->getAABB()))
               continue;

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

            if (hit)
               pendingSweepHits.emplace_back(std::move(*hit));
         }
      }
   );

   assert(childIdx == info.childCount);
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
