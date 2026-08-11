#include "pch.h"

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <variant>

#include "physics/engine/cmd/processor.h"

#include "physics/broadphase/broadphase_manager.h"
#include "physics/engine/body_spatial_update.h"
#include "physics/world/physics_world.h"
#include "physics/world/runtime_caches.h"

namespace physics::internal::cmd {


//================================================
// Helper functions
//================================================
namespace {
    template<class Handle>
    bool containsHandle(
        const std::vector<Handle>& handles,
        Handle handle)
    {
        return std::find(
            handles.begin(),
            handles.end(),
            handle) != handles.end();
    }

    template<class Handle>
    void addUniqueHandle(
        std::vector<Handle>& handles,
        Handle handle)
    {
        if (!containsHandle(handles, handle)) {
            handles.push_back(handle);
        }
    }

    template<class Handle>
    void eraseHandle(
        std::vector<Handle>& handles,
        Handle handle)
    {
        handles.erase(
            std::remove(
                handles.begin(),
                handles.end(),
                handle),
            handles.end());
    }

    BroadphaseBucket getBodyBucket(
        const RigidBody& body)
    {
        if (body.type == BodyType::Static) {
            return BroadphaseBucket::Static;
        }

        if (body.type == BodyType::Dynamic &&
            body.asleep &&
            body.motionControl != MotionControl::External) {
            return BroadphaseBucket::Asleep;
        }

        return BroadphaseBucket::Awake;
    }

    bool hasEnabledCollider(
        PhysicsWorld& physicsWorld,
        const RigidBody& body)
    {
        for (ColliderHandle colliderHandle : body.colliderHandles) {
            if (!physicsWorld.isColliderActive(colliderHandle)) {
                continue;
            }

            const Collider* collider = physicsWorld.getCollider(colliderHandle);

            if (collider && collider->enabled) {
                return true;
            }
        }

        return false;
    }

    Collider* findFirstEnabledCollider(
        PhysicsWorld& physicsWorld,
        const RigidBody& body)
    {
        for (ColliderHandle colliderHandle : body.colliderHandles) {
            if (!physicsWorld.isColliderActive(colliderHandle)) {
                continue;
            }

            Collider* collider = physicsWorld.getCollider(colliderHandle);

            if (collider && collider->enabled) {
                return collider;
            }
        }

        return nullptr;
    }

    bool applyShape(
        Collider& collider,
        const ColliderShapeDesc& shape)
    {
        bool validShape = true;

        std::visit([&](const auto& shapeDesc) {
            using ShapeDescType = std::decay_t<decltype(shapeDesc)>;

            if constexpr (std::is_same_v<ShapeDescType, BoxShapeDesc>) {
                collider.type = ColliderType::CUBOID;
                collider.shape = OOBB(
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

        // Rotation som tar current till target.
        glm::quat delta = target * glm::conjugate(current);

        // Välj den kortaste rotationsvägen.
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
}

Processor::Processor(
    PhysicsWorld& physicsWorld,
    RuntimeCaches& caches,
    BroadphaseManager& broadphaseManager)
    : physicsWorld(physicsWorld),
      caches(caches),
      broadphaseManager(broadphaseManager)
{}

void Processor::process(
    const Buffer::Batch& batch,
    float dt)
{
    processLifecycleCommands(batch);
    applyMutationCommands(batch.mutations, dt);
}

void Processor::processLifecycleCommands(
    const Buffer::Batch& batch)
{
    if (batch.bodyCreates.empty() &&
        batch.colliderCreates.empty() &&
        batch.bodyDestroys.empty() &&
        batch.colliderDestroys.empty()) {
        return;
    }

    std::vector<BodyHandle> affectedBodies;

    //----------------------------------
    // Collect affected bodies
    //----------------------------------
    for (BodyHandle bodyHandle : batch.bodyCreates) {
        addUniqueHandle(affectedBodies, bodyHandle);
    }

    for (BodyHandle bodyHandle : batch.bodyDestroys) {
        addUniqueHandle(affectedBodies, bodyHandle);
    }

    for (ColliderHandle colliderHandle : batch.colliderCreates) {
        const Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (collider) {
            addUniqueHandle(affectedBodies, collider->rigidBodyHandle);
        }
    }

    for (ColliderHandle colliderHandle : batch.colliderDestroys) {
        const Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (collider) {
            addUniqueHandle(affectedBodies, collider->rigidBodyHandle);
        }
    }

    //----------------------------------
    // Remove affected active bodies from broadphase
    //----------------------------------
    for (BodyHandle bodyHandle : affectedBodies) {
        if (!physicsWorld.isRigidBodyActive(bodyHandle)) {
            continue;
        }

        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (body && body->broadphaseHandle.bucket != BroadphaseBucket::None) {
            broadphaseManager.remove(bodyHandle);
        }
    }

    bool activeStorageChanged = false;

    //----------------------------------
    // Destroy individual colliders
    //----------------------------------
    for (ColliderHandle colliderHandle : batch.colliderDestroys) {
        Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (!collider) {
            continue;
        }

        BodyHandle bodyHandle = collider->rigidBodyHandle;

        // The body-destroy phase owns all colliders belonging to a destroyed body.
        if (containsHandle(batch.bodyDestroys, bodyHandle)) {
            continue;
        }

        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (body) {
            eraseHandle(body->colliderHandles, colliderHandle);
        }

        if (physicsWorld.isColliderActive(colliderHandle)) {
            physicsWorld.deleteCollider(colliderHandle);
            activeStorageChanged = true;
        }
        else if (physicsWorld.isColliderPending(colliderHandle)) {
            physicsWorld.discardPendingCollider(colliderHandle);
        }
    }

    //----------------------------------
    // Destroy bodies and their colliders
    //----------------------------------
    for (BodyHandle bodyHandle : batch.bodyDestroys) {
        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (!body) {
            continue;
        }

        const std::vector<ColliderHandle> activeColliders =
            body->colliderHandles;

        // Destroy all colliders belonging to the body being destroyed.
        for (ColliderHandle colliderHandle : activeColliders) {
            physicsWorld.deleteCollider(colliderHandle);
            activeStorageChanged = true;
        }

        // Pending colliders are not necessarily present in body->colliderHandles yet.
        for (ColliderHandle colliderHandle : batch.colliderCreates) {
            Collider* collider = physicsWorld.getCollider(colliderHandle);

            // Check if the collider belongs to the body being destroyed
            if (!collider || collider->rigidBodyHandle != bodyHandle) {
                continue;
            }

            // If we are here it means the collider is pending,
            // so discard it from the pending colliders list.
            physicsWorld.discardPendingCollider(colliderHandle);
        }

        if (physicsWorld.isRigidBodyActive(bodyHandle)) {
            physicsWorld.deleteRigidBody(bodyHandle);
            activeStorageChanged = true;
        }
        else if (physicsWorld.isRigidBodyPending(bodyHandle)) {
            physicsWorld.discardPendingRigidBody(bodyHandle);
        }
    }

    //----------------------------------
    // Activate pending bodies
    //----------------------------------
    for (BodyHandle bodyHandle : batch.bodyCreates) {
        if (containsHandle(batch.bodyDestroys, bodyHandle)) {
            continue;
        }

        if (physicsWorld.activateRigidBody(bodyHandle)) {
            activeStorageChanged = true;
        }
    }

    //----------------------------------
    // Activate pending colliders
    //----------------------------------
    for (ColliderHandle colliderHandle : batch.colliderCreates) {
        if (containsHandle(batch.colliderDestroys, colliderHandle)) {
            continue;
        }

        Collider* collider = physicsWorld.getCollider(colliderHandle);

        if (!collider) {
            continue;
        }

        BodyHandle bodyHandle = collider->rigidBodyHandle;

        if (containsHandle(batch.bodyDestroys, bodyHandle) ||
            !physicsWorld.isRigidBodyActive(bodyHandle)) {
            if (physicsWorld.isColliderPending(colliderHandle)) {
                physicsWorld.discardPendingCollider(colliderHandle);
            }

            continue;
        }

        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (!body) {
            physicsWorld.discardPendingCollider(colliderHandle);
            continue;
        }

        collider->updateWorldPose(body->pose, body->scale);
        collider->updateShape();
        collider->updateAABB();

        if (!physicsWorld.activateCollider(colliderHandle)) {
            physicsWorld.discardPendingCollider(colliderHandle);
            continue;
        }

        if (!containsHandle(body->colliderHandles, colliderHandle)) {
            body->colliderHandles.push_back(colliderHandle);
        }

        activeStorageChanged = true;
    }

    //----------------------------------
    // Clear pointer caches after dense storage changes
    //----------------------------------
    if (activeStorageChanged) {
        caches.clear();
    }

    //----------------------------------
    // Rebuild affected surviving bodies
    //----------------------------------
    for (BodyHandle bodyHandle : affectedBodies) {
        if (!physicsWorld.isRigidBodyActive(bodyHandle)) {
            continue;
        }

        RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

        if (!body) {
            continue;
        }

        std::vector<ColliderHandle>& colliderHandles =
            body->colliderHandles;

        colliderHandles.erase(
            std::remove_if(
                colliderHandles.begin(),
                colliderHandles.end(),
                [this](ColliderHandle colliderHandle) {
                    return !physicsWorld.isColliderActive(colliderHandle);
                }),
            colliderHandles.end());

        if (!hasEnabledCollider(physicsWorld, *body)) {
            body->aabb = internal::AABB{};
            continue;
        }

        body->aabb = physicsWorld.computeBodyAABB(*body);

        const float radius = 0.5f *
            glm::length(body->aabb.worldMax - body->aabb.worldMin);

        body->invRadius = radius > 0.0f ? 1.0f / radius : 0.0f;

        refreshBodyInertia(*body);

        broadphaseManager.add(bodyHandle, getBodyBucket(*body));
    }

    // #TODO: Can't clear contact cache here because it may contain contacts
    // for bodies that are still active. Need to implement a more selective
    // contact cache clearing mechanism.
}

void Processor::refreshBodyInertia(
    RigidBody& body)
{
    if (body.type != BodyType::Dynamic) {
        body.invInertiaLocal = glm::mat3(0.0f);
        body.invInertiaWorld = glm::mat3(0.0f);
        return;
    }

    Collider* collider = findFirstEnabledCollider(
        physicsWorld,
        body
    );

    if (!collider) {
        body.invInertiaLocal = glm::mat3(0.0f);
        body.invInertiaWorld = glm::mat3(0.0f);
        return;
    }

    glm::vec3 inertiaScale = collider->worldScale;

    if (collider->type == ColliderType::SPHERE) {
        const Sphere& sphere = std::get<Sphere>(collider->shape);
        inertiaScale = glm::vec3(sphere.radiusWorld * 2.0f);
    }

    body.calculateInverseInertia(
        collider->type,
        *collider,
        inertiaScale
    );
    body.updateInertiaWorld();
}

void Processor::refreshBodySpatialState(
    BodyHandle bodyHandle,
    bool shouldRefreshInertia)
{
    RigidBody* body = physicsWorld.getRigidBody(bodyHandle);

    if (!body || !physicsWorld.isRigidBodyActive(bodyHandle)) {
        return;
    }

    updateCollidersAndBodyAABB(caches, body);

    if (shouldRefreshInertia) {
        refreshBodyInertia(*body);
    }

    const bool hasEnabledCollider =
        findFirstEnabledCollider(physicsWorld, *body) != nullptr;

    if (!hasEnabledCollider) {
        if (body->broadphaseHandle.bucket != BroadphaseBucket::None) {
            broadphaseManager.remove(bodyHandle);
        }

        return;
    }

    if (body->broadphaseHandle.bucket == BroadphaseBucket::None) {
        BroadphaseBucket bucket = BroadphaseBucket::Awake;

        if (body->type == BodyType::Static) {
            bucket = BroadphaseBucket::Static;
        }
        else if (
            body->type == BodyType::Dynamic &&
            body->asleep &&
            body->motionControl != MotionControl::External) {
            bucket = BroadphaseBucket::Asleep;
        }

        broadphaseManager.add(bodyHandle, bucket);
        return;
    }

    broadphaseManager.setBVHDirty(bodyHandle);
}

//================================================
// Command processing
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
// Rigid body command processing
//================================================
void Processor::applyCommand(
    const Buffer::ApplyLinearImpulse& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type != BodyType::Dynamic) {
        return;
    }

    if (body->asleep) {
        broadphaseManager.moveToAwake(command.body);
    }

    body->applyImpulseLinear(command.impulse);
}

void Processor::applyCommand(
    const Buffer::SetLinearVelocity& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type == BodyType::Static) {
        return;
    }

    body->linearVelocity = command.velocity;
}

void Processor::applyCommand(
    const Buffer::SetAngularVelocity& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type == BodyType::Static) {
        return;
    }

    body->angularVelocity = command.velocity;
}

void Processor::applyCommand(
    const Buffer::SetKinematicTarget& command,
    float dt)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type != BodyType::Kinematic) 
    {
        return;
    }

    body->linearVelocity =
        (command.target.position - body->pose.position) / dt;

    body->angularVelocity = calculateAngularVelocity(
        body->pose.orientation,
        command.target.orientation,
        dt);
}

void Processor::applyCommand(
    const Buffer::SetRigidBodyTransform& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    body->pose = command.pose;
    body->scale = command.scale;

    refreshBodySpatialState(command.body);
}

void Processor::applyCommand(
    const Buffer::SetRigidBodySleepState& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    if (command.asleep) {
        body->setAsleep();
        broadphaseManager.moveToAsleep(command.body);
    }
    else {
        body->setAwake();
        broadphaseManager.moveToAwake(command.body);
    }
}

void Processor::applyCommand(
    const Buffer::SetRigidBodyType& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    body->type = command.type;

    switch (command.type) {
    case BodyType::Dynamic:
        if (body->mass <= 0.0f) {
            body->mass = 1.0f;
        }

        body->invMass = 1.0f / body->mass;

        if (body->sleepCounterThreshold <= 0.0f) {
            body->sleepCounterThreshold = 1.5f;
        }

        refreshBodyInertia(*body);

        if (body->allowSleep && body->asleep) {
            body->setAsleep();
            broadphaseManager.moveToAsleep(command.body);
        }
        else {
            body->setAwake();
            broadphaseManager.moveToAwake(command.body);
        }

        break;

    case BodyType::Kinematic:
        body->invMass = 0.0f;
        body->invInertiaLocal = glm::mat3(0.0f);
        body->invInertiaWorld = glm::mat3(0.0f);
        body->linearVelocity = glm::vec3(0.0f);
        body->angularVelocity = glm::vec3(0.0f);
        body->setAwake();
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
    const Buffer::SetRigidBodyMotionControl& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    body->setMotionControl(command.motionControl);
}

void Processor::applyCommand(
    const Buffer::SetRigidBodyResponseMode& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body || !physicsWorld.isRigidBodyActive(command.body)) {
        return;
    }

    body->responseMode = command.responseMode;
}

void Processor::applyCommand(
    const Buffer::SetRigidBodyMass& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (!body ||
        !physicsWorld.isRigidBodyActive(command.body) ||
        body->type != BodyType::Dynamic ||
        command.mass <= 0.0f) {
        return;
    }

    body->mass = command.mass;
    body->invMass = 1.0f / command.mass;
    refreshBodyInertia(*body);
}

void Processor::applyCommand(
    const Buffer::SetRigidBodyAllowGravity& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (body && physicsWorld.isRigidBodyActive(command.body)) {
        body->allowGravity = command.allowGravity;
    }
}

void Processor::applyCommand(
    const Buffer::SetRigidBodyAllowSleep& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (body && physicsWorld.isRigidBodyActive(command.body)) {
        body->allowSleep = command.allowSleep;
    }
}

void Processor::applyCommand(
    const Buffer::SetRigidBodyCanMoveLinearly& command,
    float)
{
    RigidBody* body = physicsWorld.getRigidBody(command.body);

    if (body && physicsWorld.isRigidBodyActive(command.body)) {
        body->canMoveLinearly = command.canMoveLinearly;
    }
}

//================================================
// Collider command processing
//================================================
void Processor::applyCommand(
    const Buffer::SetColliderLocalPose& command,
    float)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (!collider || !physicsWorld.isColliderActive(command.collider)) {
        return;
    }

    collider->localPose = command.localPose;
    refreshBodySpatialState(collider->rigidBodyHandle);
}

void Processor::applyCommand(
    const Buffer::SetColliderLocalTransform& command,
    float)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (!collider || !physicsWorld.isColliderActive(command.collider)) {
        return;
    }

    collider->localPose = command.localPose;
    collider->localScale = command.localScale;
    refreshBodySpatialState(collider->rigidBodyHandle);
}

void Processor::applyCommand(
    const Buffer::SetColliderShape& command,
    float)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (!collider ||
        !physicsWorld.isColliderActive(command.collider) ||
        !applyShape(*collider, command.shape)) {
        return;
    }

    refreshBodySpatialState(collider->rigidBodyHandle);
}

void Processor::applyCommand(
    const Buffer::SetColliderEnabled& command,
    float)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (!collider || !physicsWorld.isColliderActive(command.collider)) {
        return;
    }

    collider->enabled = command.enabled;
    refreshBodySpatialState(collider->rigidBodyHandle);
}

void Processor::applyCommand(
    const Buffer::SetColliderTrigger& command,
    float)
{
    Collider* collider = physicsWorld.getCollider(command.collider);

    if (collider && physicsWorld.isColliderActive(command.collider)) {
        collider->isTrigger = command.isTrigger;
    }
}

//================================================
// Scene-wide command processing
//================================================
void Processor::applyCommand(
    const Buffer::SleepAllObjects&,
    float)
{
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < static_cast<uint32_t>(dense.size()); ++i) {
        RigidBody& body = dense[i];

        if (body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        broadphaseManager.moveToAsleep(
            bodyMap.handle_from_dense_index(i)
        );
    }
}

void Processor::applyCommand(
    const Buffer::AwakenAllObjects&,
    float)
{
    auto& bodyMap = physicsWorld.getRigidBodiesMap();
    auto& dense = bodyMap.dense();

    for (uint32_t i = 0; i < static_cast<uint32_t>(dense.size()); ++i) {
        RigidBody& body = dense[i];

        if (!body.asleep) continue;
        if (body.type == BodyType::Static) continue;
        if (body.type == BodyType::Kinematic) continue;
        if (body.motionControl == MotionControl::External) continue;

        broadphaseManager.moveToAwake(
            bodyMap.handle_from_dense_index(i)
        );
    }
}

}
