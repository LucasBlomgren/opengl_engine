#include "pch.h"

#include <type_traits>
#include <utility>
#include <variant>

#include "physics/public/engine.h"

namespace physics {

using namespace internal;

namespace {

const RigidBody* resolveBody(
   const cmd::Buffer& commandBuffer,
   const PhysicsStorage& physicsStorage,
   BodyHandle handle)
{
   if (commandBuffer.isBodyPendingDestroy(handle)) {
      return nullptr;
   }

   if (const RigidBody* body =
      commandBuffer.tryGetPendingBodyCreate(handle)) {
      return body;
   }

   return physicsStorage.tryGetBody(handle);
}

const Collider* resolveCollider(
   const cmd::Buffer& commandBuffer,
   const PhysicsStorage& physicsStorage,
   ColliderHandle handle)
{
   if (commandBuffer.isColliderPendingDestroy(handle)) {
      return nullptr;
   }

   const Collider* collider =
      commandBuffer.tryGetPendingColliderCreate(handle);

   if (!collider) {
      collider = physicsStorage.tryGetCollider(handle);
   }

   if (!collider ||
      commandBuffer.isBodyPendingDestroy(
         collider->bodyHandle)) {
      return nullptr;
   }

   return collider;
}
} // namespace 

//=========================================
// Mesh upload and destruction
//=========================================
MeshHandle Engine::uploadMesh(
   const std::string& id, 
   const MeshGeometryDesc& desc)
{
   std::unordered_map<std::string, MeshHandle>& registry = 
      physicsStorage.getMeshRegistry();

   if (auto search = registry.find(id); search != registry.end()) {
      std::cout << "[Physics] Mesh already uploaded." << std::endl;
      return {};
   }

   if (desc.faces.size() <= 0 ||
      desc.vertices.size() <= 0 ||
      desc.faceIndices.size() <= 0)
   {
      std::cout << "[Physics] Invalid Mesh." << std::endl;
      return {};
   }

   for (const FaceDesc& faceDesc : desc.faces) {
      if (faceDesc.firstIndex < 0 || faceDesc.vertexCount <= 0)
      {
         std::cout << "[Physics] Invalid Mesh Face." << std::endl;
         return {};
      }
   }

   MeshHandle handle = physicsStorage.reserveMeshHandle();
   registry.emplace(id, handle);

   std::vector<MeshFace> faces;
   faces.reserve(desc.faces.size());
   for (const FaceDesc& faceDesc : desc.faces) {
      MeshFace face;
      face.start = faceDesc.firstIndex;
      face.count = faceDesc.vertexCount;
      faces.push_back(face);
      // #TODO: MeshGeometry: also upload face normals
   }

   MeshShape mesh(
      desc.vertices,
      faces,
      desc.faceIndices
   );

   // #TODO: MeshGeometry: cook inertia matrix
   // #TODO: Mesh Collider: cook BVH

   commandBuffer.recordMeshCreate(
      handle,
      std::move(mesh)
   );

   return handle;
}

bool Engine::destroyMesh(MeshHandle meshHandle) 
{
   if (!physicsStorage.tryGetMesh(meshHandle)) {
      std::cout << "Mesh handle is not alive" << std::endl;
      return false;
   }

   return commandBuffer.recordMeshDestroy(
      meshHandle, 
      physicsStorage);
}

//=========================================
// ConvexHull upload and destruction
//=========================================
HullHandle Engine::uploadConvexHull(
   const std::string& id,
   const MeshGeometryDesc& desc)
{
   std::unordered_map<std::string, HullHandle>& registry =
      physicsStorage.getHullRegistry();

   if (auto search = registry.find(id); search != registry.end()) {
      std::cout << "[Physics] ConvexHull already uploaded." << std::endl;
      return {};
   }

   if (desc.faces.size() <= 0 ||
      desc.vertices.size() <= 0 ||
      desc.faceIndices.size() <= 0)
   {
      std::cout << "[Physics] Invalid ConvexHull." << std::endl;
      return {};
   }

   for (const FaceDesc& faceDesc : desc.faces) {
      if (faceDesc.firstIndex < 0 || faceDesc.vertexCount <= 0)
      {
         std::cout << "[Physics] Invalid MeshGeometry Face." << std::endl;
         return {};
      }
   }

   HullHandle handle = physicsStorage.reserveHullHandle();
   registry.emplace(id, handle);

   std::vector<MeshFace> faces;
   faces.reserve(desc.faces.size());
   for (const FaceDesc& faceDesc : desc.faces) {
      MeshFace face;
      face.start = faceDesc.firstIndex;
      face.count = faceDesc.vertexCount;
      faces.push_back(face);
      // #TODO: MeshGeometry: also upload face normals
   }

   Hull hull(
      desc.vertices,
      faces,
      desc.faceIndices
   );

   // #TODO: MeshGeometry: cook inertia matrix

   commandBuffer.recordHullCreate(
      handle,
      std::move(hull)
   );

   return handle;
}

bool Engine::destroyConvexHull(HullHandle hull)
{
   if (!physicsStorage.tryGetHull(hull)) {
      std::cout << "Mesh handle is not alive" << std::endl;
      return false;
   }

   return commandBuffer.recordHullDestroy(
      hull,
      physicsStorage);
}

//=========================================
// Collider creation and destruction
//=========================================
ColliderHandle Engine::createCollider(
   BodyHandle bodyHandle,
   const ColliderDesc& desc)
{
   const RigidBody* body = resolveBody(
      commandBuffer,
      physicsStorage,
      bodyHandle
   );

   if (!body) {
      return {};
   }

   ColliderHandle colliderHandle =
      physicsStorage.reserveColliderHandle();

   Collider collider;
   collider.bodyHandle = bodyHandle;
   collider.localPose = desc.localPose;
   collider.enabled = desc.enabled;
   collider.isTrigger = desc.isTrigger;
   collider.userTag = desc.userTag;

   bool validShape = true;

   std::visit([&](const auto& shapeDesc) {
      using ShapeDescType = std::decay_t<decltype(shapeDesc)>;

      if constexpr (std::is_same_v<ShapeDescType, BoxShapeDesc>) {
         collider.type = ShapeType::BOX;
         collider.shape = Box(shapeDesc.halfExtents, shapeDesc.center);
      }
      else if constexpr (std::is_same_v<ShapeDescType, SphereShapeDesc>) {
         collider.type = ShapeType::SPHERE;
         collider.shape = Sphere(shapeDesc.radius, shapeDesc.center);
      }
      else if constexpr (std::is_same_v<ShapeDescType, MeshHandle>) {
         collider.type = ShapeType::MESH;
         collider.shape = shapeDesc;
      }
      else if constexpr (std::is_same_v<ShapeDescType, HullHandle>) {
         collider.type = ShapeType::CONVEX_HULL;
         collider.shape = shapeDesc;
      }
      else {
         validShape = false;
      }
      }, desc.shape);

   if (!validShape) {
      physicsStorage.releaseColliderReservation(colliderHandle);
      return {};
   }

   collider.updateWorldPose(body->pose);
   collider.updateShape();
   collider.rebuildAABB();

   commandBuffer.recordColliderCreate(
      colliderHandle,
      std::move(collider)
   );

   return colliderHandle;
}

bool Engine::destroyCollider(ColliderHandle colliderHandle) 
{
   if (!resolveCollider(
      commandBuffer,
      physicsStorage,
      colliderHandle)) {
      return false;
   }

   return commandBuffer.recordColliderDestroy(
      colliderHandle,
      physicsStorage
   );
}

//=========================================
// Collider commands
//=========================================
bool Engine::setColliderLocalPose(
   ColliderHandle handle,
   const Pose& localPose)
{
   const Collider* collider = resolveCollider(
      commandBuffer,
      physicsStorage,
      handle
   );

   if (!collider) {
      return false;
   }

   commandBuffer.recordSetColliderLocalPose(handle, localPose);
   return true;
}

bool Engine::setColliderLocalTransform(
   ColliderHandle handle,
   const Pose& localPose,
   const glm::vec3& localScale)
{
   const Collider* collider = resolveCollider(
      commandBuffer,
      physicsStorage,
      handle
   );

   if (!collider) {
      return false;
   }

   commandBuffer.recordSetColliderLocalTransform(
      handle,
      localPose,
      localScale
   );
   return true;
}

bool Engine::setColliderShape(
   ColliderHandle handle,
   const ColliderShapeDesc& shape)
{
   const Collider* collider = resolveCollider(
      commandBuffer,
      physicsStorage,
      handle
   );

   if (!collider) {
      return false;
   }

   commandBuffer.recordSetColliderShape(handle, shape);
   return true;
}

bool Engine::setColliderEnabled(
   ColliderHandle handle,
   bool enabled)
{
   const Collider* collider = resolveCollider(
      commandBuffer,
      physicsStorage,
      handle
   );

   if (!collider) {
      return false;
   }

   commandBuffer.recordSetColliderEnabled(handle, enabled);
   return true;
}

bool Engine::setColliderTrigger(
   ColliderHandle handle,
   bool isTrigger)
{
   const Collider* collider = resolveCollider(
      commandBuffer,
      physicsStorage,
      handle
   );

   if (!collider) {
      return false;
   }

   commandBuffer.recordSetColliderTrigger(handle, isTrigger);
   return true;
}

}
