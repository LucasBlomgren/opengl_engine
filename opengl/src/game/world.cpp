#include "pch.h"

#include <algorithm>
#include <source_location>

#include "world.h"

#include "graphics/mesh/mesh_manager.h"
#include "graphics/renderer/renderer.h"
#include "graphics/shaders/shader_manager.h"
#include "graphics/textures/texture_manager.h"
#include "physics/public/engine.h"

namespace {
constexpr float kSyncEpsilon = 1e-6f;

bool vectorsDiffer(
   const glm::vec3& left,
   const glm::vec3& right)
{
   return glm::length2(left - right) >
      kSyncEpsilon * kSyncEpsilon;
}

bool orientationsDiffer(
   const glm::quat& left,
   const glm::quat& right)
{
   const glm::quat normalizedLeft = glm::normalize(left);
   const glm::quat normalizedRight = glm::normalize(right);
   return 1.0f - glm::abs(glm::dot(
      normalizedLeft,
      normalizedRight
   )) > kSyncEpsilon;
}

bool posesDiffer(
   const physics::Pose& physicsPose,
   const Transform& transform)
{
   return
      vectorsDiffer(physicsPose.position, transform.position) ||
      orientationsDiffer(
         physicsPose.orientation,
         transform.orientation
      );
}

physics::Pose makePhysicsPose(const Transform& transform) {
   physics::Pose pose;
   pose.position = transform.position;
   pose.orientation = transform.orientation;
   return pose;
}

// Match the root * part matrix hierarchy, while keeping collider poses unscaled.
physics::Pose makeColliderLocalPose(
   const Transform& root,
   const Transform& part)
{
   physics::Pose pose = makePhysicsPose(part);
   pose.position = root.scale * part.position;
   return pose;
}

bool posesDiffer(
   const physics::Pose& left,
   const physics::Pose& right)
{
   return vectorsDiffer(left.position, right.position) ||
      orientationsDiffer(left.orientation, right.orientation);
}

// Express S_root * R_part * S_part in the collider's R_part frame.
// This captures correct centers and conservative OBB half-extents even
// when nonuniform root scaling and part rotation cause shear.
glm::mat3 getBakedShapeTransform(
   const Transform& root,
   const Transform& part)
{
   const glm::mat3 R = glm::mat3_cast(glm::normalize(part.orientation));
   glm::mat3 rootScale(1.0f);
   rootScale[0][0] = root.scale.x;
   rootScale[1][1] = root.scale.y;
   rootScale[2][2] = root.scale.z;

   glm::mat3 partScale(1.0f);
   partScale[0][0] = part.scale.x;
   partScale[1][1] = part.scale.y;
   partScale[2][2] = part.scale.z;

   return glm::transpose(R) * rootScale * R * partScale;
}

// Axis-aligned bounds of the transformed box in collider-local coordinates.
glm::vec3 getTransformedHalfExtents(
   const glm::mat3& transform,
   const glm::vec3& halfExtents)
{
   return glm::abs(transform[0]) * halfExtents.x +
      glm::abs(transform[1]) * halfExtents.y +
      glm::abs(transform[2]) * halfExtents.z;
}
}

void World::clear() {
   bodyToGameObject.clear();
   gameObjects.clear();
   transforms.clear();
   objectId = 0;
}

GameObject* World::getGameObject(
   const GameObjectHandle& handle) {
   return gameObjects.try_get(handle);
}

Transform* World::getTransform(
   const TransformHandle& handle) {
   return transforms.try_get(handle);
}

GameObjectHandle World::getGameObjectHandle(
   physics::BodyHandle body) const
{
   const auto iterator = bodyToGameObject.find(body);

   if (iterator == bodyToGameObject.end()) {
      return {};
   }

   return iterator->second;
}

TransformHandle World::createTransform(
   const glm::vec3& position,
   const glm::quat& orientation,
   const glm::vec3& scale) {
   return transforms.create(position, orientation, scale);
}

GameObjectHandle World::createGameObject(
   GameObjectDesc& objectDesc)
{
   Transform* rootTransform =
      transforms.try_get(objectDesc.rootTransformHandle);

   if (!rootTransform) {
      std::cerr
         << "[World] Cannot create GameObject '"
         << objectDesc.name
         << "': invalid root transform.\n";
      return {};
   }

   GameObjectHandle gameObjectHandle =
      gameObjects.create(objectId, objectDesc.rootTransformHandle);
   GameObject* gameObject =
      gameObjects.try_get(gameObjectHandle);

   if (!gameObject) {
      return {};
   }

   gameObject->name = objectDesc.name;

   physics::BodyDesc bodyDesc;
   bodyDesc.pose = makePhysicsPose(*rootTransform);
   bodyDesc.type = objectDesc.bodyType;
   bodyDesc.reportContacts = objectDesc.reportContacts;
   bodyDesc.mass = objectDesc.mass;
   bodyDesc.allowSleep = objectDesc.allowSleep;
   bodyDesc.allowGravity = objectDesc.allowGravity;
   bodyDesc.startAsleep = objectDesc.asleep;

   physics::BodyHandle bodyHandle =
      physicsEngine.createRigidBody(bodyDesc);

   if (!bodyHandle.isValid()) {
      std::cerr
         << "[World] Cannot create physics body for GameObject '"
         << objectDesc.name
         << "'.\n";
      gameObjects.destroy(gameObjectHandle);
      return {};
   }

   gameObject->rigidBodyHandle = bodyHandle;

   // Create SubParts and their colliders
   size_t createdColliderCount = 0;
   for (const SubPartDesc& partDesc : objectDesc.parts) {
      Transform* partTransform =
         transforms.try_get(partDesc.localTransformHandle);

      if (!partTransform) {
         std::cerr
            << "[World] Skipping part '"
            << partDesc.name
            << "' with invalid local transform.\n";
         continue;
      }

      // Create the SubPart and set its properties
      SubPart part;
      part.name = partDesc.name;
      part.localTransformHandle = partDesc.localTransformHandle;
      part.shader = shaderManager.getShader(partDesc.shaderName);
      part.mesh = meshManager.getMesh(partDesc.meshName);

      // If the specified mesh is not found, use a default cube mesh
      if (!part.mesh) {
         std::cerr
            << "[World] Warning: Mesh not found for part '"
            << partDesc.name
            << "' of GameObject '"
            << objectDesc.name
            << "'. Using default cube mesh.\n";
         part.mesh = meshManager.getMesh("cube");
      }

      if (!part.mesh) {
         std::cerr
            << "[World] Skipping part '"
            << partDesc.name
            << "': fallback cube mesh was not found.\n";
         continue;
      }

      // Set the texture ID based on the texture name
      part.textureId =
         partDesc.textureName == "plain"
         ? 999
         : textureManager.getTexture(partDesc.textureName);
      part.color = partDesc.color;
      part.seeThrough = partDesc.seeThrough;

      // Calculate the local center and half extents for the collider
      glm::vec3 localCenter(0.0f);
      glm::vec3 localHalfExtents(0.5f);
      if (!part.mesh->vertices.empty()) {
         glm::vec3 minimum =
            part.mesh->vertices.front().position;
         glm::vec3 maximum = minimum;

         for (const Vertex& vertex : part.mesh->vertices) {
            minimum = glm::min(minimum, vertex.position);
            maximum = glm::max(maximum, vertex.position);
         }

         localCenter = (minimum + maximum) * 0.5f;
         localHalfExtents = (maximum - minimum) * 0.5f;
      }

      // Build unscaled collider pose + geometry baked to match the render mesh.
      physics::ColliderDesc colliderDesc;
      colliderDesc.localPose = makeColliderLocalPose(
         *rootTransform, *partTransform
      );

      const glm::mat3 bakedTransform = getBakedShapeTransform(
         *rootTransform, *partTransform
      );
      const glm::vec3 bakedCenter = bakedTransform * localCenter;

      if (partDesc.colliderType == physics::ColliderType::BOX) {
         const glm::vec3 bakedHalfExtents =
            getTransformedHalfExtents(bakedTransform, localHalfExtents);

         colliderDesc.shape = physics::BoxShapeDesc{
            bakedCenter,
            bakedHalfExtents
         };
      }
      else if (partDesc.colliderType == physics::ColliderType::SPHERE) {
         float radius = 0.5f;

         if (!part.mesh->vertices.empty()) {
            radius = 0.0f;
            for (const Vertex& vertex : part.mesh->vertices) {
               radius = (std::max)(
                  radius,
                  glm::length(vertex.position - localCenter)
                  );
            }
         }

         // Conservative sphere enclosing an anisotropically scaled mesh.
         const float maxRootScale =
            glm::compMax(glm::abs(rootTransform->scale));
         const float maxPartScale =
            glm::compMax(glm::abs(partTransform->scale));

         colliderDesc.shape = physics::SphereShapeDesc{
            bakedCenter,
            radius * maxRootScale * maxPartScale
         };
      }
      else {
         std::cerr
            << "[World] Skipping part '"
            << partDesc.name
            << "': unknown collider type.\n";
         continue;
      }

      physics::ColliderHandle colliderHandle =
         physicsEngine.createCollider(bodyHandle, colliderDesc);

      if (!colliderHandle.isValid()) {
         std::cerr
            << "[World] Failed to create collider for part '"
            << partDesc.name
            << "'.\n";
         continue;
      }

      part.colliderHandle = colliderHandle;
      gameObject->parts.push_back(part);
      ++createdColliderCount;
   }

   //// If no colliders were created, clean up and return an invalid handle
   //if (createdColliderCount == 0) {
   //    std::cerr
   //        << "[World] Cannot create GameObject '"
   //        << objectDesc.name
   //        << "': no colliders were created.\n";

   //    physicsEngine.destroyRigidBody(bodyHandle);
   //    gameObjects.destroy(gameObjectHandle);
   //    return {};
   //}

   // Map the rigid body handle to the game object handle
   bodyToGameObject[bodyHandle] = gameObjectHandle;

   // Add the game object to the renderer's batch for rendering
   renderer.addObjectToBatch(gameObjectHandle);

   ++objectId;
   return gameObjectHandle;
}

void World::deleteGameObject(
   GameObjectHandle handle)
{
   GameObject* object = gameObjects.try_get(handle);

   if (!object) {
      std::cerr
         << "[World] Warning: Tried to delete non-existing "
         << "GameObject with handle (slot: "
         << handle.slot
         << ", gen: "
         << handle.gen
         << ")\n";
      return;
   }

   const physics::BodyHandle bodyHandle =
      object->rigidBodyHandle;

   physicsEngine.destroyRigidBody(bodyHandle);
   renderer.removeObjectFromBatch(handle);
   bodyToGameObject.erase(bodyHandle);

   transforms.destroy(object->rootTransformHandle);
   gameObjects.destroy(handle);
}

void World::deleteSubPart(
   GameObjectHandle handle,
   int partIndex)
{
   GameObject* object = gameObjects.try_get(handle);
   if (!object) {
      std::cerr
         << "[World] Warning: Tried to delete sub-part from "
         << "non-existing GameObject with handle (slot: "
         << handle.slot
         << ", gen: "
         << handle.gen
         << ")\n";
      return;
   }
   if (partIndex < 0 || partIndex >= static_cast<int>(object->parts.size())) {
      std::cerr
         << "[World] Warning: Tried to delete sub-part with invalid index "
         << partIndex
         << " from GameObject '"
         << object->name
         << "'\n";
      return;
   }
   SubPart& part = object->parts[partIndex];
   physicsEngine.destroyCollider(part.colliderHandle);

   renderer.removeObjectFromBatch(handle);
   transforms.destroy(part.localTransformHandle);

   object->parts.erase(object->parts.begin() + partIndex);

   if (!object->parts.empty()) {
      renderer.addObjectToBatch(handle);
   }
}

void World::syncGameObjectTransformToPhysics(
   GameObjectHandle handle)
{
   GameObject* object = gameObjects.try_get(handle);

   if (!object) {
      return;
   }

   Transform* rootTransform =
      transforms.try_get(object->rootTransformHandle);
   const std::optional<physics::BodyState> bodyState =
      physicsEngine.getRigidBodyState(object->rigidBodyHandle);

   if (!rootTransform || !bodyState) {
      return;
   }

   if (posesDiffer(bodyState->pose, *rootTransform))
   {
      physicsEngine.setRigidBodyTransform(
         object->rigidBodyHandle,
         makePhysicsPose(*rootTransform)
      );
   }

   for (const SubPart& part : object->parts) {
      Transform* localTransform =
         transforms.try_get(part.localTransformHandle);
      const std::optional<physics::ColliderState> colliderState =
         physicsEngine.getColliderState(part.colliderHandle);

      if (!localTransform || !colliderState) {
         continue;
      }

      const physics::Pose expectedPose = makeColliderLocalPose(
         *rootTransform, *localTransform
      );

      if (posesDiffer(colliderState->localPose, expectedPose)) {
         physicsEngine.setColliderLocalPose(
            part.colliderHandle,
            expectedPose
         );
      }
   }
}

void World::syncTransformsToPhysics() {
   double start = glfwGetTime();
   const std::vector<GameObject>& objects = gameObjects.dense();

   for (uint32_t index = 0;
      index < static_cast<uint32_t>(objects.size());
      ++index)
   {
      const GameObject& object = objects[index];
      const std::optional<physics::BodyState> bodyState =
         physicsEngine.getRigidBodyState(object.rigidBodyHandle);

      if (!bodyState) {
         continue;
      }

      if (bodyState->type == physics::BodyType::Kinematic) {
         syncGameObjectTransformToPhysics(
            gameObjects.handle_from_dense_index(index)
         );
         continue;
      }

      Transform* rootTransform =
         transforms.try_get(object.rootTransformHandle);

      if (!rootTransform) {
         continue;
      }

      for (const SubPart& part : object.parts)
      {
         Transform* localTransform =
            transforms.try_get(part.localTransformHandle);

         const std::optional<physics::ColliderState> colliderState =
            physicsEngine.getColliderState(part.colliderHandle);

         if (!localTransform || !colliderState) {
            continue;
         }

         const physics::Pose expectedPose = makeColliderLocalPose(
            *rootTransform, *localTransform
         );

         if (posesDiffer(colliderState->localPose, expectedPose)) {
            physicsEngine.setColliderLocalPose(
               part.colliderHandle,
               expectedPose
            );
         }
      }
   }

   double end = glfwGetTime();
   //std::cout << "syncTransformsToPhysics: " << (end - start) * 1000 << " ms\n";
}

void World::syncPhysicsToTransforms()
{
   double start = glfwGetTime();
   std::vector<GameObject>& objects = gameObjects.dense();

   for (GameObject& object : objects) {
      const std::optional<physics::BodyState> bodyState =
         physicsEngine.getRigidBodyState(object.rigidBodyHandle);

      if (!bodyState) {
         continue;
      }

      const bool physicsDrivesPose =
         bodyState->type == physics::BodyType::Dynamic ||
         bodyState->type == physics::BodyType::Kinematic;

      if (!physicsDrivesPose) {
         continue;
      }

      Transform* transform =
         transforms.try_get(object.rootTransformHandle);

      if (!transform) {
         continue;
      }

      if (!posesDiffer(bodyState->pose, *transform)) {
         continue;
      }

      transform->lastPosition = transform->position;
      transform->position = bodyState->pose.position;
      transform->orientation = bodyState->pose.orientation;
      transform->updateCache();
   }

   double end = glfwGetTime();
   //std::cout << "syncPhysicsToTransforms: " << (end - start) * 1000 << " ms \n";
}
