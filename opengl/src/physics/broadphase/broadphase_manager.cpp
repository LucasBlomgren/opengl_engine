#include <unordered_set>

#include "pch.h"
#include "broadphase_manager.h"

#include "physics/bodies/rigidbody.h"
#include "physics/collider/body_collider_utils.h"
#include "physics/collider/geometry/tri.h"
#include "physics/bvh/query_othertree.h"
#include "physics/bvh/query_sametree.h"

namespace physics::internal {

//=======================================
//    Init & Clear
//=======================================
void BroadphaseManager::init(
   PhysicsStorage* physicsStorage,
   std::vector<Tri>* terrainTris)
{
   this->storage = physicsStorage;
   this->terrainTriangles = terrainTris;

   SlotMap<RigidBody, BodyHandle>& bMap = physicsStorage->bodyStorage();
   size_t slotCap = bMap.dense().capacity();

   awakeBvh.init(physicsStorage, slotCap, true);
   asleepBvh.init(physicsStorage, slotCap, true);
   staticBvh.init(physicsStorage, slotCap, true);

   // false = speculative pairs don't need to write leaf indices
   speculativeBvh.init(physicsStorage, slotCap, false);

   awakeHandles.reserve(slotCap * 2);
   asleepHandles.reserve(slotCap * 2);
   staticHandles.reserve(slotCap * 2);


   constructBvh(awakeBvh, awakeHandles);
   constructBvh(asleepBvh, asleepHandles);
   constructBvh(staticBvh, staticHandles);
   terrainBvh.build(*terrainTriangles);
}

void BroadphaseManager::constructBvh(
   BVHTree& tree,
   std::vector<BodyHandle>& handles)
{
   std::vector<ColliderHandle> colliderHandles;

   for (BodyHandle bodyH : handles) {
      RigidBody& body = storage->getBody(bodyH);

      if (body.colliderRef.isCompound()) {
         Compound& compound =
            storage->getCompound(body.colliderRef.compound);

         for (ColliderHandle colH : compound.colliders)
            colliderHandles.push_back(colH);
      }
      else {
         colliderHandles.push_back(body.colliderRef.single);
      }
   }

   tree.buildFromColliders(colliderHandles);
}

void BroadphaseManager::clear() {
   awakeHandles.clear();
   asleepHandles.clear();
   staticHandles.clear();

   awakeBvh.clear();
   asleepBvh.clear();
   staticBvh.clear();
   speculativeBvh.clear();

   pairsBufDynamic.clear();
   pairsBufTerrain.clear();
}

//=======================================
//    Set BVH render data flag
//=======================================
void BroadphaseManager::updateBVHRenderData(const physics::debug::BvhType& type, bool update) {
   switch (type) {
   case physics::debug::BvhType::Awake:
      awakeBvh.shouldUpdateRenderData = update;
      break;
   case physics::debug::BvhType::Asleep:
      asleepBvh.shouldUpdateRenderData = update;
      break;
   case physics::debug::BvhType::Static:
      staticBvh.shouldUpdateRenderData = update;
      break;
   }
}

//==================================================
//      Update BVHs
//==================================================
void BroadphaseManager::updateBVHs() {
   awakeBvh.update();

   if (asleepBvh.dirty) asleepBvh.update();
   if (staticBvh.dirty) constructBvh(staticBvh, staticHandles);
}

//==================================================
//     Global pair building for narrowphase
//==================================================
void BroadphaseManager::buildPairs(PairBatch& batch)
{
   batch.dynamicPairs.reserve(BVHTree::MaxCollisionBuf);
   batch.terrainPairs.reserve(BVHTree::MaxCollisionBuf);

   // Dynamic vs terrain
   if (terrainBvh.rootIdx != -1) {
      pairsBufTerrain.clear();
      pairsBufTerrain.reserve(BVHTree::MaxCollisionBuf);

      treeVsOtherTreeQuery(awakeBvh, terrainBvh, pairsBufTerrain);

      // Group terrain triangles by collider
      std::unordered_map<ColliderHandle, std::vector<Tri*>> temp;
      temp.reserve(pairsBufTerrain.size());

      for (const auto& [leafRef, tri] : pairsBufTerrain) {
         temp[leafRef.collider].push_back(tri);
      }

      // Finalize terrain pairs
      batch.terrainPairs.clear();
      batch.terrainPairs.reserve(temp.size());

      for (auto& [colH, trisVec] : temp) {
         const Collider& collider = storage->getCollider(colH);

         batch.terrainPairs.push_back(TerrainPair{
            collider.bodyHandle,
            colH,
            std::move(trisVec)
            });
      }
   }

   batch.dynamicPairs.clear();

   // dynamic vs dynamic
   if (awakeBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsSameTreeQuery(awakeBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.dynamicPairs.resize(cap);
      int sp = 0;

      for (auto& hp : pairsBufDynamic) {
         batch.dynamicPairs[sp++] =
            DynamicPair{
               hp.first.body,
               hp.second.body,
               hp.first.collider,
               hp.second.collider
         };
      }
      batch.dynamicPairs.resize(sp);
   }

   // dynamic vs asleep
   if (asleepBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsOtherTreeQuery(awakeBvh, asleepBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.dynamicPairs.reserve(batch.dynamicPairs.size() + cap);
      int sp = 0;

      for (auto& hp : pairsBufDynamic) {
         batch.dynamicPairs.emplace_back(
            DynamicPair{
               hp.first.body,
               hp.second.body,
               hp.first.collider,
               hp.second.collider
            }
         );
      }
   }

   // dynamic vs static
   if (staticBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsOtherTreeQuery(awakeBvh, staticBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.dynamicPairs.reserve(batch.dynamicPairs.size() + cap);
      int sp = 0;

      for (auto& hp : pairsBufDynamic) {
         batch.dynamicPairs.emplace_back(
            DynamicPair{
               hp.first.body,
               hp.second.body,
               hp.first.collider,
               hp.second.collider
            }
         );
      }
   }
}

//==================================================
//     Speculative pair building for narrowphase
//==================================================
void BroadphaseManager::buildSpeculativePairs(
   float dt,
   PairBatch& batch,
   std::vector<AABB>& debugSweeps)
{
   batch.speculativeDynamicPairs.clear();
   batch.speculativeTerrainPairs.clear();
   batch.speculativeDynamicPairs.reserve(BVHTree::MaxCollisionBuf);
   batch.speculativeTerrainPairs.reserve(BVHTree::MaxCollisionBuf);

   static std::vector<ColliderHandle> colliders;
   static std::vector<AABB> AABBs;

   colliders.reserve(awakeHandles.size());
   colliders.clear();

   AABBs.reserve(awakeHandles.size());
   AABBs.clear();

   determineSpeculativeBodies(dt, colliders, AABBs);

   speculativeBvh.clear();
   speculativeBvh.buildFromExternalAABBs(colliders, AABBs);

   struct SpecKey {
      uint64_t owner;
      uint64_t a;
      uint64_t b;

      bool operator==(const SpecKey& other) const {
         return owner == other.owner &&
            a == other.a &&
            b == other.b;
      }
   };

   struct SpecKeyHash {
      size_t operator()(const SpecKey& k) const {
         size_t h = std::hash<uint64_t>{}(k.owner);
         h ^= std::hash<uint64_t>{}(k.a + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2));
         h ^= std::hash<uint64_t>{}(k.b + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2));
         return h;
      }
   };

   static std::unordered_set<SpecKey, SpecKeyHash> seenSpecPairs;
   seenSpecPairs.clear();

   auto packHandle = [](auto h) -> uint64_t {
      return (uint64_t(h.slot) << 32) | uint64_t(h.gen);
      };

   auto addSpecPair = [&](
      BodyHandle bodyA,
      BodyHandle bodyB,
      ColliderHandle colA,
      ColliderHandle colB,
      BodyHandle owner)
      {
         if (bodyA == bodyB) {
            return;
         }

         uint64_t ak = packHandle(colA);
         uint64_t bk = packHandle(colB);
         uint64_t ok = packHandle(owner);

         if (bk < ak) {
            std::swap(ak, bk);
         }

         SpecKey key{};
         key.owner = ok;
         key.a = ak;
         key.b = bk;

         if (!seenSpecPairs.insert(key).second) {
            return;
         }

         batch.speculativeDynamicPairs.emplace_back(
            SpeculativeDynamicPair{ bodyA, bodyB, colA, colB, owner }
         );
      };

   // ----- Dynamic vs terrain -----
   if (terrainBvh.rootIdx != -1) {
      pairsBufTerrain.clear();
      pairsBufTerrain.reserve(BVHTree::MaxCollisionBuf);

      treeVsOtherTreeQuery(speculativeBvh, terrainBvh, pairsBufTerrain);

      // Group terrain triangles by collider
      std::unordered_map<ColliderHandle, std::vector<Tri*>> temp;
      temp.reserve(pairsBufTerrain.size());

      for (const auto& [leafRef, tri] : pairsBufTerrain) {
         temp[leafRef.collider].push_back(tri);
      }

      // Finalize terrain pairs
      batch.speculativeTerrainPairs.clear();
      batch.speculativeTerrainPairs.reserve(temp.size());

      for (auto& [colH, trisVec] : temp) {
         const Collider& collider = storage->getCollider(colH);

         batch.speculativeTerrainPairs.push_back(
            SpeculativeTerrainPair{
               collider.bodyHandle,
               colH,
               std::move(trisVec)
            }
         );
      }
   }

   // ----- speculative vs speculative -----
   if (speculativeBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsSameTreeQuery(speculativeBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

      for (auto& hp : pairsBufDynamic) {
         BodyHandle& bodyA = hp.first.body;
         BodyHandle& bodyB = hp.second.body;
         ColliderHandle& colA = hp.first.collider;
         ColliderHandle& colB = hp.second.collider;

         addSpecPair(bodyA, bodyB, colA, colB, bodyA); // A's owner
         addSpecPair(bodyA, bodyB, colA, colB, bodyB); // B's owner
      }
   }

   // ----- speculative vs awake -----
   if (awakeBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsOtherTreeQuery(speculativeBvh, awakeBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

      for (auto& hp : pairsBufDynamic) {
         BodyHandle& bodyA = hp.first.body;
         BodyHandle& bodyB = hp.second.body;
         ColliderHandle& colA = hp.first.collider;
         ColliderHandle& colB = hp.second.collider;

         addSpecPair(bodyA, bodyB, colA, colB, bodyA); // A's owner
      }
   }

   // ----- speculative vs asleep -----
   if (asleepBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsOtherTreeQuery(speculativeBvh, asleepBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

      for (auto& hp : pairsBufDynamic) {
         batch.speculativeDynamicPairs.emplace_back(
            SpeculativeDynamicPair{ 
               hp.first.body, 
               hp.second.body, 
               hp.first.collider,
               hp.second.collider,
               hp.first.body 
            }
         );
      }
   }

   // ----- speculative vs static -----
   if (staticBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsOtherTreeQuery(speculativeBvh, staticBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

      for (auto& hp : pairsBufDynamic) {
         batch.speculativeDynamicPairs.emplace_back(
            SpeculativeDynamicPair{
               hp.first.body,
               hp.second.body,
               hp.first.collider,
               hp.second.collider,
               hp.first.body
            }
         );
      }
   }
}

void BroadphaseManager::determineSpeculativeBodies(
   float dt,
   std::vector<ColliderHandle>& outColliders,
   std::vector<AABB>& outAABBs)
{
   for (const BodyHandle& handle : awakeHandles) {
      RigidBody& body = storage->getBody(handle);

      if (body.type != BodyType::Dynamic) {
         continue;
      }

      if (!body.colliderRef.isValid()) {
         continue;
      }

      constexpr float safeFraction = 0.50f;
      constexpr float minSafeDistance = 0.02f;

      const glm::vec3& scale = body.scale;

      float minExtent =
         std::min(scale.x, std::min(scale.y, scale.z));

      float safeDistance =
         std::max(minExtent * safeFraction, minSafeDistance);

      float scale2 = glm::dot(scale, scale);
      float dt2 = dt * dt;

      float linearMotion2 =
         glm::dot(body.linearVelocity, body.linearVelocity) * dt2;

      float angularMotion2 =
         glm::dot(body.angularVelocity, body.angularVelocity) *
         (0.25f * scale2) *
         dt2;

      float safeDistance2 = safeDistance * safeDistance;

      if (linearMotion2 + angularMotion2 <= 0.5f * safeDistance2) {
         continue;
      }

      forEachBodyCollider(*storage, body.colliderRef, [&](ColliderHandle colH) {
         Collider& collider = storage->getCollider(colH);

         if (!collider.enabled)
            return;

         outColliders.push_back(colH);
         outAABBs.emplace_back(computeSweptAABB(body, collider, dt));
      });
   }
}

AABB BroadphaseManager::computeSweptAABB(
   const RigidBody& body,
   Collider& collider,
   float dt)
{
   AABB swept = collider.getAABB();

   // Linear motion
   glm::vec3 delta = body.linearVelocity * dt;

   swept.worldMin += glm::min(delta, glm::vec3(0.0f));
   swept.worldMax += glm::max(delta, glm::vec3(0.0f));

   // Angular motion
   const AABB& aabb = collider.getAABB();

   glm::vec3 center =
      (aabb.worldMin + aabb.worldMax) * 0.5f;

   glm::vec3 halfExtents =
      (aabb.worldMax - aabb.worldMin) * 0.5f;

   float rotationRadius =
      glm::length(center - body.pose.position) +
      glm::length(halfExtents);

   float angularExpansion =
      glm::length(body.angularVelocity) * dt * rotationRadius;

   constexpr float sweptSkin = 0.01f;

   float expansion = angularExpansion + sweptSkin;

   swept.worldMin -= glm::vec3(expansion);
   swept.worldMax += glm::vec3(expansion);

   swept.worldCenter =
      (swept.worldMin + swept.worldMax) * 0.5f;

   swept.worldHalfExtents =
      (swept.worldMax - swept.worldMin) * 0.5f;

   return swept;
}

//==================================================
//    List management
//==================================================
void BroadphaseManager::add(const BodyHandle& handle, BroadphaseBucket dst) {
   RigidBody& body = storage->getBody(handle);
   if (dst == BroadphaseBucket::None) {
      return;
   }

   auto& h = body.broadphaseHandle;
   if (h.bucket != BroadphaseBucket::None) return;

   auto& list = listFor(dst);
   list.push_back(handle);
   h.listIdx = (int)list.size() - 1;

   auto& bvh = bvhFor(dst);

   forEachBodyCollider(*storage, body.colliderRef, [&](ColliderHandle colH) {
      Collider& col = storage->getCollider(colH);

      if (col.enabled)
         bvh.insertLeaf(handle, colH);
   });

   h.bucket = dst;

   bvh.dirty = true;
}

// Remove from current list
void BroadphaseManager::remove(const BodyHandle& handle) {
   RigidBody& body = storage->getBody(handle);

   auto& h = body.broadphaseHandle;
   if (h.bucket == BroadphaseBucket::None) return;

   // remove from list
   swapAndPop(handle, listFor(h.bucket));

   // remove from BVH
   auto& bvh = bvhFor(h.bucket);
   forEachBodyCollider(*storage, body.colliderRef, [&](ColliderHandle colH) {
      Collider& col = storage->getCollider(colH);

      if (col.enabled) {
         bvh.removeLeaf(col.BVHleafIdx);
         col.BVHleafIdx = -1;
      }

   });

   h.bucket = BroadphaseBucket::None;

   bvh.dirty = true;
}

// Move to awake
void BroadphaseManager::moveToAwake(const BodyHandle& handle) {
   RigidBody& body = storage->getBody(handle);

   if (body.broadphaseHandle.bucket == BroadphaseBucket::Awake) {
      return;
   }

   remove(handle);
   add(handle, BroadphaseBucket::Awake);
   awakeBvh.dirty = true;
}
// Move to asleep
void BroadphaseManager::moveToAsleep(const BodyHandle& handle) {
   RigidBody& body = storage->getBody(handle);

   if (body.broadphaseHandle.bucket == BroadphaseBucket::Asleep) {
      return;
   }

   remove(handle);
   add(handle, BroadphaseBucket::Asleep);
   asleepBvh.dirty = true;
}
// Move to static
void BroadphaseManager::moveToStatic(const BodyHandle& handle) {
   RigidBody& body = storage->getBody(handle);
   if (body.broadphaseHandle.bucket == BroadphaseBucket::Static) {
      return;
   }

   remove(handle);
   add(handle, BroadphaseBucket::Static);
   staticBvh.dirty = true;
}

void BroadphaseManager::setBVHDirty(const BodyHandle& handle) {
   RigidBody& rigidBody = storage->getBody(handle);
   auto& h = rigidBody.broadphaseHandle;
   if (h.bucket == BroadphaseBucket::None) return;
   bvhFor(h.bucket).dirty = true;
}

//==================================================
//     Helpers
//==================================================
void BroadphaseManager::swapAndPop(
   const BodyHandle& handle,
   std::vector<BodyHandle>& list)
{
   RigidBody& rigidBody = storage->getBody(handle);

   int i = rigidBody.broadphaseHandle.listIdx;
   if (i == -1) return;

   if (i < 0 ||
      i >= static_cast<int>(list.size()) ||
      !(list[static_cast<size_t>(i)] == handle)) {
      auto position = std::find(list.begin(), list.end(), handle);
      if (position == list.end()) {
         rigidBody.broadphaseHandle.listIdx = -1;
         return;
      }

      i = static_cast<int>(std::distance(list.begin(), position));
      rigidBody.broadphaseHandle.listIdx = i;
   }

   int lastPos = (int)list.size() - 1;
   if (i != lastPos) {
      BodyHandle movedHandle = list[lastPos];
      list[i] = movedHandle;

      RigidBody& moved = storage->getBody(movedHandle);
      moved.broadphaseHandle.listIdx = i;
   }

   list.pop_back();
   rigidBody.broadphaseHandle.listIdx = -1;
}

// Get BVH for bucket
BVHTree& BroadphaseManager::bvhFor(BroadphaseBucket b) {
   if (b == BroadphaseBucket::Awake)  return awakeBvh;
   if (b == BroadphaseBucket::Asleep) return asleepBvh;
   return staticBvh;
}

// Get list for bucket
std::vector<BodyHandle>& BroadphaseManager::listFor(BroadphaseBucket b) {
   if (b == BroadphaseBucket::Awake)  return awakeHandles;
   if (b == BroadphaseBucket::Asleep) return asleepHandles;
   return staticHandles;
}

}
