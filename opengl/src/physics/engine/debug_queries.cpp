#include "pch.h"

#include "physics/public/engine.h"

#include "graphics/debug/draw_line.h"

namespace physics {

using namespace internal;

namespace {
physics::AABB toPublicBounds(const internal::AABB& bounds) {
   physics::AABB result;
   result.min = bounds.min;
   result.max = bounds.max;
   return result;
}

glm::vec3 getColliderCenter(const Collider* collider) {
   if (!collider) {
      return glm::vec3(0.0f);
   }

   if (collider->type == ShapeType::BOX) {
      return std::get<Box>(collider->shape).worldCenter;
   }

   return std::get<Sphere>(collider->shape).centerWorld;
}

physics::debug::Bvh copyBvh(const BVHTree& tree) {
   physics::debug::Bvh result;
   result.nodes.reserve(tree.nodes.size());

   for (const BVHTree::Node& node : tree.nodes) {
      if (!node.alive) {
         continue;
      }

      result.nodes.push_back({
          node.isLeaf,
          toPublicBounds(node.isLeaf ? node.tightBox : node.fatBox)
         });
   }

   return result;
}
}

void Engine::renderDebugImmediate()
{
   for (const RigidBody& body : physicsStorage.bodyStorage().dense()) {
      if (!body.colliderRef.isSingle())
         continue;

      const Collider& collider =
         physicsStorage.getCollider(body.colliderRef.single);

      const MeshHandle* meshH = std::get_if<MeshHandle>(&collider.shape);
      if (!meshH || !physicsStorage.isAlive(*meshH))
         continue;

      const MeshShape& mesh = physicsStorage.getMesh(*meshH);

      std::vector<internal::AABB> boxes;
      boxes.reserve(mesh.faces.size());

      for (const MeshFace& face : mesh.faces) {
         glm::vec3 minimum(FLT_MAX);
         glm::vec3 maximum(-FLT_MAX);

         for (size_t i = 0; i < face.count; ++i) {
            glm::vec3 vertex = mesh.vertices[mesh.faceIndices[face.start + i]];

            glm::vec3 worldVertex = 
               collider.worldPose.position + (collider.worldPose.orientation * vertex);

            minimum = glm::min(minimum, worldVertex);
            maximum = glm::max(maximum, worldVertex);
         }

         internal::AABB aabb;
         aabb.min = minimum;
         aabb.max = maximum;

         boxes.push_back(aabb);
      }

      for (uint32_t i = 0; i < boxes.size(); ++i) {
         const auto& aabb = boxes[i];
         glm::vec3 p[8];

         for (int j = 0; j < 8; ++j) {
            p[j] = {
               (j & 1) ? aabb.max.x : aabb.min.x,
               (j & 2) ? aabb.max.y : aabb.min.y,
               (j & 4) ? aabb.max.z : aabb.min.z
            };
         }

         for (int j = 0; j < 8; ++j) {
            for (int axis = 0; axis < 3; ++axis) {
               if (!(j & (1 << axis))) 
                  drawLine(p[j], p[j | (1 << axis)], i);
            }
         }
      }
   }
}

physics::debug::Data Engine::getDebugData() const {
   physics::debug::Data debugData;
   debugData.awake = broadphaseManager.getAwakeList().size();
   debugData.asleep = broadphaseManager.getAsleepList().size();
   debugData.staticBodies = broadphaseManager.getStaticList().size();
   debugData.colliders = physicsStorage.colliderStorage().dense().size();
   debugData.terrainTris = terrainTriangles.size();
   debugData.contacts = contactsThisFrame;
   debugData.speculativeContacts = speculativeContactsThisFrame;
   return debugData;
}

physics::debug::StepPhase
Engine::getDebugPhase() const {
   return debugPhase;
}

void Engine::updateBVHRenderData(
   const physics::debug::BvhType& type,
   bool update) {
   broadphaseManager.updateBVHRenderData(type, update);
}

std::vector<physics::AABB>
Engine::getDebugSweeps() const 
{
   const std::vector<internal::AABB>& aabbs = broadphaseManager.getBodySweptAABBs();

   static std::vector<physics::AABB> result;
   result.clear();
   result.reserve(static_cast<int>(aabbs.size()));

   for (const internal::AABB& bounds : broadphaseManager.getBodySweptAABBs()) {
      result.push_back(toPublicBounds(bounds));
   }

   return result;
}

std::vector<physics::debug::SpeculativeContact>
Engine::getDebugSpeculativeContacts() const {
   std::vector<physics::debug::SpeculativeContact> result;
   result.reserve(debugSpeculativeContacts.size());

   for (const DebugSpeculativeContact& contact :
      debugSpeculativeContacts) {
      result.push_back({
          contact.bodyA,
          contact.bodyB,
          contact.worldPos
         });
   }

   return result;
}

std::vector<physics::debug::Contact>
Engine::getDebugContacts() const
{
   std::vector<physics::debug::Contact> result;
   result.reserve(contactCache.size());

   for (const auto& [key, contact] : contactCache) {
      (void)key;

      if (!contact.wasUsedThisFrame) {
         continue;
      }

      physics::debug::Contact debugContact;
      debugContact.normal = contact.normal;

      // Compute the representative point and contact points
      for (size_t index = 0; index < contact.numPoints; ++index) {
         const ContactPoint& point = contact.points[index];

         if (!point.wasUsedThisFrame) {
            continue;
         }

         // Add the contact point to the debug contact
         debugContact.points[debugContact.pointCount] = {
             point.worldPos,
             point.wasWarmStarted
         };

         // Update the representative point by averaging the contact points
         debugContact.representativePoint += point.worldPos;
         ++debugContact.pointCount;
      }

      // If there are no contact points, use the average of the 
      // contact points as the representative point
      if (debugContact.pointCount > 0) {
         debugContact.representativePoint /=
            static_cast<float>(debugContact.pointCount);
      }
      // If there are contact points, use the center of one 
      // of the colliders as the representative point
      else if (contact.runtimeData.colliderA) {
         debugContact.representativePoint =
            getColliderCenter(contact.runtimeData.colliderA);
      }
      else {
         debugContact.representativePoint =
            getColliderCenter(contact.runtimeData.colliderB);
      }

      result.push_back(debugContact);
   }

   return result;
}

const std::vector<BodyHandle>&
Engine::getAwakeList() const {
   return broadphaseManager.getAwakeList();
}

physics::debug::Bvh Engine::getDebugBvh(
   physics::debug::BvhType type) const
{
   switch (type) {
   case physics::debug::BvhType::Awake:
      return copyBvh(broadphaseManager.getAwakeBVH());
   case physics::debug::BvhType::Asleep:
      return copyBvh(broadphaseManager.getAsleepBVH());
   case physics::debug::BvhType::Static:
      return copyBvh(broadphaseManager.getStaticBVH());
   }

   return {};
}

physics::debug::Bvh
Engine::getTerrainDebugBvh() const
{
   physics::debug::Bvh result;
   const TerrainBVH& tree = broadphaseManager.getTerrainBVH();
   result.nodes.reserve(tree.nodes.size());

   for (const TerrainBVH::Node& node : tree.nodes) {
      if (!node.alive || !node.isLeaf) {
         continue;
      }

      result.nodes.push_back({
          true,
          toPublicBounds(node.fatBox)
         });
   }

   return result;
}

}
