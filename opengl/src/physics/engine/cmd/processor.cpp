#include "pch.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>

#include "physics/engine/cmd/processor.h"

#include "physics/broadphase/broadphase_manager.h"
#include "physics/storage/physics_storage.h"
#include "physics/sleep/sleep_manager.h"
#include "physics/collider/body_collider_utils.h"

namespace physics::internal::cmd {

namespace {

void attachColliderToBody(
   PhysicsStorage& storage,
   RigidBody& body,
   ColliderHandle colliderH)
{
   BodyColliderRef& ref = body.colliderRef;

   if (!ref.isValid()) {
      ref = BodyColliderRef{ colliderH };
      return;
   }

   if (ref.isSingle()) {
      Compound compound;
      compound.colliders.reserve(2);
      compound.colliders.push_back(ref.single);
      compound.colliders.push_back(colliderH);

      CompoundHandle compoundH = storage.reserveCompoundHandle();
      storage.commitCompound(compoundH, std::move(compound));

      ref = BodyColliderRef{ compoundH };
      return;
   }

   storage.getCompound(ref.compound).colliders.push_back(colliderH);
}

void detachColliderFromBody(
   PhysicsStorage& storage,
   RigidBody& body,
   ColliderHandle colliderH)
{
   BodyColliderRef& ref = body.colliderRef;

   if (ref.isSingle()) {
      ref = BodyColliderRef{};
      return;
   }

   if (ref.isCompound()) {
      CompoundHandle compoundH = ref.compound;
      Compound& compound = storage.getCompound(compoundH);

      size_t removed = std::erase(compound.colliders, colliderH);

      if (compound.colliders.empty()) {
         storage.destroyCompound(compoundH);
         ref = BodyColliderRef{};
      }
   }
}

BroadphaseBucket getBodyBucket(
   PhysicsStorage& physicsStorage,
   const RigidBody& body)
{
   if (body.type == BodyType::Static) {
      return BroadphaseBucket::Static;
   }

   if (body.type == BodyType::Dynamic) {
      SleepState& sleepState = physicsStorage.getSleepState(body.sleepStateHandle);

      if (sleepState.isAsleep() && sleepState.canSleep()) {
         return BroadphaseBucket::Asleep;
      }
   }

   return BroadphaseBucket::Awake;
}

Collider* findFirstEnabledCollider(
   PhysicsStorage& storage,
   const RigidBody& body)
{
   Collider* result = nullptr;

   forEachBodyCollider(
      storage,
      body.colliderRef,
      [&](ColliderHandle colH) {
         if (result) return;

         Collider* col = storage.tryGetCollider(colH);

         if (col && col->enabled)
            result = col;
      }
   );

   return result;
}

bool applyShape(
   Collider& collider,
   const ColliderShapeDesc& shape)
{
   bool validShape = true;

   std::visit([&](const auto& shapeDesc) {
      using ShapeDescType = std::decay_t<decltype(shapeDesc)>;

      if constexpr (std::is_same_v<ShapeDescType, BoxShapeDesc>) {
         collider.type = ColliderType::BOX;
         collider.shape = Box(
            shapeDesc.halfExtents,
            shapeDesc.center
         );
      }
      else if constexpr (std::is_same_v<ShapeDescType, SphereShapeDesc>) {
         collider.type = ColliderType::SPHERE;
         collider.shape = Sphere(
            shapeDesc.radius,
            shapeDesc.center
         );
      }
      else {
         validShape = false;
      }
      }, shape);

   return validShape;
}

glm::vec3 calculateAngularVelocity(
   const glm::quat& currentRotation,
   const glm::quat& targetRotation,
   float dt)
{
   if (dt <= 0.0f) {
      return glm::vec3(0.0f);
   }

   const glm::quat current = glm::normalize(currentRotation);
   const glm::quat target = glm::normalize(targetRotation);

   glm::quat delta = target * glm::conjugate(current);

   if (delta.w < 0.0f) {
      delta = -delta;
   }

   const glm::vec3 imaginary(delta.x, delta.y, delta.z);
   const float sinHalfAngle = glm::length(imaginary);

   if (sinHalfAngle < 1e-6f) {
      return glm::vec3(0.0f);
   }

   const glm::vec3 axis = imaginary / sinHalfAngle;
   const float angle = 2.0f * std::atan2(sinHalfAngle, delta.w);

   return axis * (angle / dt);
}

} // namespace

Processor::Processor(
   PhysicsStorage& physicsStorage,
   SleepManager& sleepManager,
   BroadphaseManager& broadphaseManager)
   : physicsStorage(physicsStorage),
   sleepManager(sleepManager),
   broadphaseManager(broadphaseManager)
{}

//================================================
// Batch processing
//================================================
void Processor::process(
   Buffer::Batch& batch,
   float dt)
{
   processLifecycleCommands(batch);
   applyMutationCommands(batch.mutations, dt);
}

//================================================
// Lifecycle processing
//================================================
void Processor::processLifecycleCommands(Buffer::Batch& batch)
{
   // Destroy Meshes
   for (MeshHandle h : batch.meshDestroys) {
      physicsStorage.destroyMesh(h);
   }
   // Commit Meshes
   for (auto& [meshH, mesh] : batch.meshCreates) {
      physicsStorage.commitMesh(meshH, std::move(mesh));
   }

   std::unordered_set<BodyHandle> bodiesToRefresh;

   // Collect affected bodies
   for (const auto& [bodyH, body] : batch.bodyCreates)
      bodiesToRefresh.insert(bodyH);

   for (const auto& [colH, col] : batch.colliderCreates)
      bodiesToRefresh.insert(col.bodyHandle);

   for (const auto& [colH, bodyH] : batch.colliderDestroys)
      bodiesToRefresh.insert(bodyH);

   // Remove affected bodies from broadphase before changing storage
   for (BodyHandle bodyH : batch.bodyDestroys)
      broadphaseManager.remove(bodyH);

   for (BodyHandle bodyH : bodiesToRefresh) {
      if (batch.bodyCreates.contains(bodyH) ||
         batch.bodyDestroys.contains(bodyH)) {
         continue;
      }

      broadphaseManager.remove(bodyH);
   }

   // Destroy individual colliders
   for (const auto& [colH, bodyH] : batch.colliderDestroys) {
      RigidBody& body = physicsStorage.getBody(bodyH);

      detachColliderFromBody(physicsStorage, body, colH);
      physicsStorage.destroyCollider(colH);
   }

   // Destroy bodies and their colliders
   for (BodyHandle bodyH : batch.bodyDestroys) {
      RigidBody& body = physicsStorage.getBody(bodyH);
      BodyColliderRef ref = body.colliderRef;

      forEachBodyCollider(
         physicsStorage, ref,
         [&](ColliderHandle colH) {
            physicsStorage.destroyCollider(colH);
         }
      );

      if (ref.isCompound())
         physicsStorage.destroyCompound(ref.compound);

      if (body.motionStateHandle.isValid())
         physicsStorage.destroyMotionState(body.motionStateHandle);

      if (body.sleepStateHandle.isValid())
         physicsStorage.destroySleepState(body.sleepStateHandle);

      physicsStorage.destroyBody(bodyH);
   }

   // Commit bodies
   for (auto& [bodyH, body] : batch.bodyCreates) {
      physicsStorage.commitBody(bodyH, std::move(body));
   }

   // Commit and attach colliders
   for (auto& [colH, col] : batch.colliderCreates) {
      BodyHandle bodyH = col.bodyHandle;

      physicsStorage.commitCollider(colH, std::move(col));

      RigidBody& body = physicsStorage.getBody(bodyH);
      attachColliderToBody(physicsStorage, body, colH);
   }

   // Rebuild affected bodies
   for (BodyHandle bodyH : bodiesToRefresh) {
      if (!batch.bodyDestroys.contains(bodyH))
         refreshBodySpatialState(bodyH);
   }
}

//================================================
// Mutation processing
//================================================
void Processor::applyMutationCommands(
   const std::vector<Buffer::Mutation>& mutations,
   float dt)
{
   for (const Buffer::Mutation& mutation : mutations) {
      std::visit([this, dt](const auto& command) {
         applyCommand(command, dt);
         }, mutation);
   }
}

//================================================
// Body commands
//================================================
void Processor::applyCommand(
   const Buffer::ApplyLinearImpulse& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (!body || body->type != BodyType::Dynamic) {
      return;
   }

   SleepState& sleepState = physicsStorage.getSleepState(body->sleepStateHandle);

   if (sleepState.isAsleep() && sleepState.canSleep()) {
      broadphaseManager.moveToAwake(command.body);
   }

   body->applyImpulseLinear(command.impulse);
}

void Processor::applyCommand(
   const Buffer::SetLinearVelocity& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (!body || body->type == BodyType::Static) {
      return;
   }

   body->linearVelocity = command.velocity;
}

void Processor::applyCommand(
   const Buffer::SetAngularVelocity& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (!body || body->type == BodyType::Static) {
      return;
   }

   body->angularVelocity = command.velocity;
}

void Processor::applyCommand(
   const Buffer::SetKinematicTarget& command,
   float dt)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (!body ||
      body->type != BodyType::Kinematic ||
      dt <= 0.0f) {
      return;
   }

   body->linearVelocity =
      (command.target.position - body->pose.position) / dt;

   body->angularVelocity = calculateAngularVelocity(
      body->pose.orientation,
      command.target.orientation,
      dt
   );
}

void Processor::applyCommand(
   const Buffer::SetBodyTransform& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (!body) {
      return;
   }

   body->pose = command.pose;

   refreshBodySpatialState(command.body);

   broadphaseManager.setBVHDirty(command.body);
}

void Processor::applyCommand(
   const Buffer::SetBodySleepState& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (!body ||
      body->type != BodyType::Dynamic) {
      return;
   }

   SleepState& sleepState = physicsStorage.getSleepState(body->sleepStateHandle);
   if (!sleepState.canSleep()) {
      return;
   }

   if (command.asleep) {
      sleepManager.sleepBody(command.body, sleepState);
   }
   else {
      RigidBody* body = physicsStorage.tryGetBody(command.body);
      sleepManager.wakeBody(command.body, *body, sleepState);
   }
}

void Processor::applyCommand(
   const Buffer::SetBodyType& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (!body) {
      return;
   }

   body->type = command.type;

   switch (command.type) {
   case BodyType::Dynamic: {
      if (body->mass <= 0.0f) {
         body->mass = 1.0f;
      }

      body->invMass = 1.0f / body->mass;

      SleepState& sleepState =
         sleepManager.ensureSleepState(command.body);

      refreshBodyInertia(*body);
      sleepManager.wakeBody(command.body, *body, sleepState);

      break;
   }

   case BodyType::Kinematic:
      body->invMass = 0.0f;
      body->invInertiaLocal = glm::mat3(0.0f);
      body->invInertiaWorld = glm::mat3(0.0f);
      body->linearVelocity = glm::vec3(0.0f);
      body->angularVelocity = glm::vec3(0.0f);
      broadphaseManager.moveToAwake(command.body);
      break;

   case BodyType::Static:
      body->mass = 0.0f;
      body->invMass = 0.0f;
      body->invInertiaLocal = glm::mat3(0.0f);
      body->invInertiaWorld = glm::mat3(0.0f);
      body->linearVelocity = glm::vec3(0.0f);
      body->angularVelocity = glm::vec3(0.0f);
      broadphaseManager.moveToStatic(command.body);
      break;
   }
}

void Processor::applyCommand(
   const Buffer::SetBodyReportContacts& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);
   if (body) {
      body->reportContacts = command.reportContacts;
   }
}

void Processor::applyCommand(
   const Buffer::SetBodyMass& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (!body ||
      body->type != BodyType::Dynamic ||
      command.mass <= 0.0f) {
      return;
   }

   body->mass = command.mass;
   body->invMass = 1.0f / command.mass;
   refreshBodyInertia(*body);
}

void Processor::applyCommand(
   const Buffer::SetBodyAllowGravity& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (body) {
      body->allowGravity = command.allowGravity;
   }
}

void Processor::applyCommand(
   const Buffer::SetBodyAllowSleep& command,
   float)
{
   RigidBody* body = physicsStorage.tryGetBody(command.body);

   if (!body || body->type != BodyType::Dynamic) {
      return;
   }

   sleepManager.setBodyAllowSleep(command.body, command.allowSleep);
}

//================================================
// Collider commands
//================================================
void Processor::applyCommand(
   const Buffer::SetColliderLocalPose& command,
   float)
{
   Collider* collider = physicsStorage.tryGetCollider(command.collider);

   if (!collider) {
      return;
   }

   collider->localPose = command.localPose;
   refreshBodySpatialState(collider->bodyHandle);
}

void Processor::applyCommand(
   const Buffer::SetColliderLocalTransform& command,
   float)
{
   Collider* collider = physicsStorage.tryGetCollider(command.collider);

   if (!collider) {
      return;
   }

   collider->localPose = command.localPose;
   refreshBodySpatialState(collider->bodyHandle);
}

void Processor::applyCommand(
   const Buffer::SetColliderShape& command,
   float)
{
   Collider* collider = physicsStorage.tryGetCollider(command.collider);

   if (!collider || !applyShape(*collider, command.shape)) {
      return;
   }

   refreshBodySpatialState(collider->bodyHandle);
}

void Processor::applyCommand(
   const Buffer::SetColliderEnabled& command,
   float)
{
   Collider* collider = physicsStorage.tryGetCollider(command.collider);

   if (!collider) {
      return;
   }

   collider->enabled = command.enabled;
   refreshBodySpatialState(collider->bodyHandle);
}

void Processor::applyCommand(
   const Buffer::SetColliderTrigger& command,
   float)
{
   Collider* collider = physicsStorage.tryGetCollider(command.collider);

   if (collider) {
      collider->isTrigger = command.isTrigger;
   }
}

//================================================
// Scene-wide commands
//================================================
void Processor::applyCommand(
   const Buffer::SleepAllObjects&,
   float)
{
   const std::vector<BodyHandle>& awakeHandles =
      broadphaseManager.getAwakeList();

   for (const BodyHandle& bodyHandle : awakeHandles) {
      RigidBody& body = physicsStorage.getBody(bodyHandle);

      if (body.type == BodyType::Kinematic ||
         !body.colliderRef.isValid()) {
         continue;
      }

      SleepState& sleepState =
         physicsStorage.getSleepState(body.sleepStateHandle);

      sleepManager.sleepBody(
         bodyHandle,
         sleepState
      );
   }
}

void Processor::applyCommand(
   const Buffer::AwakenAllObjects&,
   float)
{
   const std::vector<BodyHandle>& asleepHandles =
      broadphaseManager.getAsleepList();

   for (const BodyHandle& bodyHandle : asleepHandles) {
      RigidBody& body = physicsStorage.getBody(bodyHandle);

      if (!body.colliderRef.isValid()) {
         continue;
      }

      SleepState& sleepState =
         physicsStorage.getSleepState(body.sleepStateHandle);

      sleepManager.wakeBody(
         bodyHandle,
         body,
         sleepState
      );
   }
}

//================================================
// Private helpers
//================================================
void Processor::refreshBodyInertia(RigidBody& body)
{
   // #TODO: Processor: Combine inertia for compound bodies. 
   // For now, just use the first collider.

   if (body.type != BodyType::Dynamic) {
      body.invInertiaLocal = glm::mat3(0.0f);
      body.invInertiaWorld = glm::mat3(0.0f);
      return;
   }

   Collider* collider = findFirstEnabledCollider(
      physicsStorage,
      body
   );

   if (!collider) {
      body.invInertiaLocal = glm::mat3(0.0f);
      body.invInertiaWorld = glm::mat3(0.0f);
      return;
   }

   glm::vec3 inertiaScale{ 0.0f };

   if (collider->type == ColliderType::SPHERE) {
      const Sphere& sphere = std::get<Sphere>(collider->shape);
      inertiaScale = glm::vec3(sphere.radius * 2.0f);
   }
   else if (collider->type == ColliderType::BOX) {
      const Box& box = std::get<Box>(collider->shape);
      inertiaScale = box.localHalfExtents * 2.0f;
   }

   body.calculateInverseInertia(
      collider->type,
      *collider
   );
   body.updateInertiaWorld();
}

void Processor::refreshBodySpatialState(
   BodyHandle bodyHandle,
   bool shouldRefreshInertia)
{
   RigidBody& body = physicsStorage.getBody(bodyHandle);

   // No colliders
   if (!body.colliderRef.isValid()) {
      body.invRadius = 0.0f;

      if (shouldRefreshInertia)
         refreshBodyInertia(body);

      broadphaseManager.remove(bodyHandle);
      return;
   }

   float maxRadius = 0.0f;

   forEachBodyCollider(
      physicsStorage,
      body.colliderRef,
      [&](ColliderHandle colliderH) {
         Collider& collider = physicsStorage.getCollider(colliderH);

         collider.updateWorldPose(body.pose);
         collider.updateShape();
         collider.updateAABB();

         const AABB& aabb = collider.getAABB();

         // Maximum distance from body origin to AABB corners
         glm::vec3 minDelta =
            glm::abs(aabb.worldMin - body.pose.position);

         glm::vec3 maxDelta =
            glm::abs(aabb.worldMax - body.pose.position);

         float radius = glm::length(
            glm::max(minDelta, maxDelta)
         );

         maxRadius = std::max(maxRadius, radius);
      }
   );

   body.invRadius = maxRadius > 0.0f
      ? 1.0f / maxRadius
      : 0.0f;

   if (shouldRefreshInertia)
      refreshBodyInertia(body);

   if (body.broadphaseHandle.bucket == BroadphaseBucket::None) {
      broadphaseManager.add(
         bodyHandle,
         getBodyBucket(physicsStorage, body)
      );
   }
}

} // namespace physics::internal::cmd
