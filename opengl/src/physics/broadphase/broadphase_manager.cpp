#include <unordered_set>

#include "pch.h"
#include "broadphase_manager.h"

#include "physics/bodies/rigidbody.h"
#include "physics/collider/collider.h"
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

   awakeBvh.buildFromBodies(awakeHandles);
   asleepBvh.buildFromBodies(asleepHandles);
   staticBvh.buildFromBodies(staticHandles);
   terrainBvh.build(*terrainTriangles);
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
   if (staticBvh.dirty) staticBvh.update();
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

      // Group terrain triangles by body
      std::unordered_map<BodyHandle, std::vector<Tri*>> temp;
      temp.reserve(pairsBufTerrain.size());

      for (const auto& [body, tri] : pairsBufTerrain) {
         temp[body].emplace_back(tri);
      }

      // Finalize terrain pairs
      batch.terrainPairs.clear();
      batch.terrainPairs.reserve(temp.size());

      for (auto& [bodyH, trisVec] : temp) {
         batch.terrainPairs.emplace_back(TerrainPair{ bodyH,std::move(trisVec) });
      }
   }

   batch.dynamicPairs.clear();

   // dynamic vs dynamic
   if (awakeBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsSameTreeQuery(awakeBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.dynamicPairs.reserve(cap);

      for (auto& hp : pairsBufDynamic) {
         batch.dynamicPairs.emplace_back(DynamicPair{ hp.first, hp.second });
      }
   }

   // dynamic vs asleep
   if (asleepBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsOtherTreeQuery(awakeBvh, asleepBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.dynamicPairs.reserve(batch.dynamicPairs.size() + cap);

      for (auto& hp : pairsBufDynamic) {
         batch.dynamicPairs.emplace_back(DynamicPair{ hp.first, hp.second });
      }
   }

   // dynamic vs static
   if (staticBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsOtherTreeQuery(awakeBvh, staticBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.dynamicPairs.reserve(batch.dynamicPairs.size() + cap);

      for (auto& hp : pairsBufDynamic) {
         batch.dynamicPairs.emplace_back(DynamicPair{ hp.first, hp.second });
      }
   }
}

//==================================================
//     Speculative pair building for narrowphase
//==================================================
void BroadphaseManager::buildSpeculativePairs(float dt, PairBatch& batch)
{
   batch.speculativeDynamicPairs.clear();
   batch.speculativeTerrainPairs.clear();
   batch.speculativeDynamicPairs.reserve(BVHTree::MaxCollisionBuf);
   batch.speculativeTerrainPairs.reserve(BVHTree::MaxCollisionBuf);

   // Reset previous speculative bodies
   for (BodyHandle& h : speculativeBodies) {
      sweepFrame.infoByBodySlot[h.slot] = BodySweepInfo{};
   }
   // Clear previous step
   speculativeBodies.clear();
   bodySweptAABBs.clear();
   sweepFrame.childSweptAABBs.clear();

   // Resize only when new slots have been allocated
   sweepFrame.infoByBodySlot.resize(
      storage->bodyStorage().slot_capacity()
   );

   determineSpeculativeBodies(dt);

   speculativeBvh.clear();
   speculativeBvh.buildFromExternalAABBs(speculativeBodies, bodySweptAABBs);

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

   auto packHandle = [](BodyHandle h) -> uint64_t {
      return (uint64_t(h.slot) << 32) | uint64_t(h.gen);
      };

   auto addSpecPair = [&](
      BodyHandle a,
      BodyHandle b,
      BodyHandle owner)
      {
         if (a == b) {
            return;
         }

         uint64_t ak = packHandle(a);
         uint64_t bk = packHandle(b);
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
            SpeculativeDynamicPair{ a, b, owner }
         );
      };

   // ----- dynamic vs terrain -----
   if (terrainBvh.rootIdx != -1) {
      pairsBufTerrain.reserve(BVHTree::MaxCollisionBuf);
      pairsBufTerrain.clear();
      treeVsOtherTreeQuery(speculativeBvh, terrainBvh, pairsBufTerrain);

      // Sort terrain pairs by body to avoid duplicates 
      std::unordered_map<BodyHandle, std::vector<Tri*>> temp;
      temp.reserve(pairsBufTerrain.size());
      if (temp.bucket_count() < pairsBufTerrain.size())
         temp.reserve(pairsBufTerrain.size());

      for (int i = 0; i < pairsBufTerrain.size(); i++) {
         auto [body, tri] = pairsBufTerrain[i];
         temp[body].push_back(tri);
      }

      // Finalize terrain pairs
      batch.speculativeTerrainPairs.clear();
      int cap = static_cast<int>(temp.size());
      batch.speculativeTerrainPairs.resize(cap);
      int sp = 0;

      for (auto& [bodyHandle, trisVec] : temp) {
         batch.speculativeTerrainPairs[sp++] =
            SpeculativeTerrainPair{ bodyHandle, std::move(trisVec) };
      }
      batch.speculativeTerrainPairs.resize(sp);
   }

   // ----- speculative vs speculative -----
   if (speculativeBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsSameTreeQuery(speculativeBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

      for (auto& hp : pairsBufDynamic) {
         BodyHandle& a = hp.first;
         BodyHandle& b = hp.second;

         addSpecPair(a, b, a); // A's owner
         addSpecPair(a, b, b); // B's owner
      }
   }

   // ----- speculative vs awake -----
   if (awakeBvh.rootIdx != -1) {
      pairsBufDynamic.clear();
      treeVsOtherTreeQuery(speculativeBvh, awakeBvh, pairsBufDynamic);

      int cap = static_cast<int>(pairsBufDynamic.size());
      batch.speculativeDynamicPairs.reserve(batch.speculativeDynamicPairs.size() + cap);

      for (auto& hp : pairsBufDynamic) {
         addSpecPair(hp.first, hp.second, hp.first);
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
            SpeculativeDynamicPair{ hp.first, hp.second, hp.first }
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
            SpeculativeDynamicPair{ hp.first, hp.second, hp.first }
         );
      }
   }
}

void BroadphaseManager::determineSpeculativeBodies(float dt)
{
   // #TODO: Broadphase: better Sweep Fraction
   constexpr float safeFraction = 5.0f;
   constexpr float minSafeDistance = 0.02f;

   const float dt2 = dt * dt;

   for (BodyHandle& handle : awakeHandles) {
      RigidBody& body = storage->getBody(handle);

      if (body.type != BodyType::Dynamic ||
         !body.colliderRef.isValid())
         continue;

      float minExtent = std::numeric_limits<float>::max();
      float maxRadius = 0.0f;
      bool hasCollider = false;

      // Determine if body needs speculative detection
      forEachBodyCollider(
         *storage,
         body.colliderRef,
         [&](ColliderHandle colliderH) {
            Collider& collider = storage->getCollider(colliderH);

            if (!collider.enabled)
               return;

            hasCollider = true;

            float extent = 0.0f;
            float radius = 0.0f;

            if (collider.type == ColliderType::BOX) {
               const Box& box = std::get<Box>(collider.shape);

               const glm::vec3& h = box.localHalfExtents;

               extent = 2.0f * std::min(
                  h.x, std::min(h.y, h.z)
               );

               const AABB& aabb = collider.getAABB();

               radius =
                  glm::length(aabb.worldCenter - body.pose.position) +
                  glm::length(aabb.worldHalfExtents);
            }
            else if (collider.type == ColliderType::SPHERE) {
               const Sphere& sphere = std::get<Sphere>(collider.shape);

               extent = 2.0f * sphere.radius;

               // Only the sphere center moves due to body rotation.
               radius = glm::length(
                  sphere.centerWorld - body.pose.position
               );
            }

            minExtent = std::min(minExtent, extent);
            maxRadius = std::max(maxRadius, radius);
         }
      );

      if (!hasCollider)
         continue;

      const float safeDistance = std::max(
         minExtent * safeFraction,
         minSafeDistance
      );

      const float linearMotion2 =
         glm::dot(body.linearVelocity, body.linearVelocity) * dt2;

      const float angularMotion2 =
         glm::dot(body.angularVelocity, body.angularVelocity) *
         maxRadius * maxRadius * dt2;

      if (linearMotion2 + angularMotion2 <=
         0.5f * safeDistance * safeDistance)
         continue;

      // Build swept AABB from all enabled colliders
      AABB sweptAABB{};
      bool first = true;

      const int childStart =
         static_cast<int>(sweepFrame.childSweptAABBs.size());

      forEachBodyCollider(
         *storage,
         body.colliderRef,
         [&](ColliderHandle colliderH) {
            Collider& collider = storage->getCollider(colliderH);

            if (!collider.enabled)
               return;

            const AABB childSwept =
               computeSweptAABB(body, collider, dt);

            sweepFrame.childSweptAABBs.emplace_back(childSwept);

            if (first) {
               sweptAABB = childSwept;
               first = false;
            }
            else {
               sweptAABB.worldMin = glm::min(
                  sweptAABB.worldMin, childSwept.worldMin
               );

               sweptAABB.worldMax = glm::max(
                  sweptAABB.worldMax, childSwept.worldMax
               );
            }
         }
      );

      sweptAABB.worldCenter =
         (sweptAABB.worldMin + sweptAABB.worldMax) * 0.5f;

      sweptAABB.worldHalfExtents =
         (sweptAABB.worldMax - sweptAABB.worldMin) * 0.5f;

      const int childCount = 
         static_cast<int>(sweepFrame.childSweptAABBs.size()) - childStart;

      const int bodyAABBIdx = static_cast<int>(bodySweptAABBs.size());

      speculativeBodies.push_back(handle);
      bodySweptAABBs.push_back(sweptAABB);

      sweepFrame.infoByBodySlot[handle.slot] = {
         bodyAABBIdx,
         childStart,
         childCount
      };
   }
}

AABB BroadphaseManager::computeSweptAABB(
   const RigidBody& body,
   Collider& collider,
   float dt)
{
   const AABB& aabb = collider.getAABB();
   AABB swept = aabb;

   // Linear motion
   glm::vec3 delta = body.linearVelocity * dt;

   swept.worldMin += glm::min(delta, glm::vec3(0.0f));
   swept.worldMax += glm::max(delta, glm::vec3(0.0f));

   // Angular motion
   float rotationRadius = 0.0f;

   if (collider.type == ColliderType::SPHERE) {
      const Sphere& sphere = std::get<Sphere>(collider.shape);

      // Only the sphere center moves due to rotation
      rotationRadius = glm::length(
         sphere.centerWorld - body.pose.position
      );
   }
   else {
      glm::vec3 center =
         (aabb.worldMin + aabb.worldMax) * 0.5f;

      glm::vec3 halfExtents =
         (aabb.worldMax - aabb.worldMin) * 0.5f;

      rotationRadius =
         glm::length(center - body.pose.position) +
         glm::length(halfExtents);
   }

   // Conservative per-axis angular expansion
   const glm::vec3& omega = body.angularVelocity;
   float omega2 = glm::dot(omega, omega);

   glm::vec3 angularExpansion = glm::sqrt(glm::max(
      glm::vec3(omega2) - omega * omega,
      glm::vec3(0.0f)
   )) * dt * rotationRadius;

   constexpr float sweptSkin = 0.01f;

   glm::vec3 expansion =
      angularExpansion + glm::vec3(sweptSkin);

   swept.worldMin -= expansion;
   swept.worldMax += expansion;

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
   if (h.bucket != BroadphaseBucket::None) {
      return;
   }

   auto& list = listFor(dst);
   list.push_back(handle);
   h.listIdx = (int)list.size() - 1;

   auto& bvh = bvhFor(dst);
   h.leafIdx = bvh.insertLeaf(handle);
   bvh.dirty = true;

   h.bucket = dst;
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
   bvh.removeLeaf(h.leafIdx);
   bvh.dirty = true;

   h.leafIdx = -1;
   h.bucket = BroadphaseBucket::None;
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
