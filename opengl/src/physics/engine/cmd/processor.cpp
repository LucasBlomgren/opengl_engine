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

namespace physics::internal::cmd {

namespace {

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
    PhysicsStorage& physicsStorage,
    const RigidBody& body)
{
    for (ColliderHandle colliderHandle : body.colliderHandles) {
        Collider* collider = physicsStorage.tryGetCollider(colliderHandle);

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
void Processor::processLifecycleCommands(
    Buffer::Batch& batch)
{
    std::unordered_set<BodyHandle> bodiesToRefresh;
    bodiesToRefresh.reserve(
        batch.bodyCreates.size() +
        batch.colliderCreates.size() +
        batch.colliderDestroys.size()
    );

    // Remove every destroyed body from broadphase before mutating
    // dense body storage. This keeps all broadphase lookups valid
    // throughout the removal pass
    for (BodyHandle bodyHandle : batch.bodyDestroys) {
        broadphaseManager.remove(bodyHandle);
    }

    // Destroy individual colliders.
    // Buffer guarantees that their parent bodies 
    // will not be destroyed in the same batch.
    for (const auto& [colliderHandle, bodyHandle] : batch.colliderDestroys) {
        RigidBody& body = physicsStorage.getBody(bodyHandle);

        std::erase(
            body.colliderHandles,
            colliderHandle
        );

        physicsStorage.destroyCollider(colliderHandle);
        bodiesToRefresh.insert(bodyHandle);
    }

    // Destroy bodies + their motion state, sleep state, and colliders
    for (BodyHandle bodyHandle : batch.bodyDestroys) {
        RigidBody& body = physicsStorage.getBody(bodyHandle);

        for (ColliderHandle colliderHandle : body.colliderHandles) {
            physicsStorage.destroyCollider(colliderHandle);
        }

        physicsStorage.destroyMotionState(body.motionStateHandle);
        physicsStorage.destroySleepState(body.sleepStateHandle);
        physicsStorage.destroyBody(bodyHandle);
    }

    // Commit bodies before their colliders
    for (auto& [bodyHandle, body] : batch.bodyCreates) {
        physicsStorage.commitBody(
            bodyHandle,
            std::move(body)
        );

        bodiesToRefresh.insert(bodyHandle);
    }

    // Commit colliders and attach them to their active parents
    for (auto& [colliderHandle, collider] : batch.colliderCreates) {
        const BodyHandle bodyHandle = collider.rigidBodyHandle;
        RigidBody& body = physicsStorage.getBody(bodyHandle);

        physicsStorage.commitCollider(
            colliderHandle,
            std::move(collider)
        );

        body.colliderHandles.push_back(colliderHandle);
        bodiesToRefresh.insert(bodyHandle);
    }

    // Rebuild every surviving body affected by lifecycle changes
    for (BodyHandle bodyHandle : bodiesToRefresh) {
        refreshBodySpatialState(bodyHandle);
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
    body->scale = command.scale;

    refreshBodySpatialState(command.body);
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
    refreshBodySpatialState(collider->rigidBodyHandle);
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
    collider->localScale = command.localScale;
    refreshBodySpatialState(collider->rigidBodyHandle);
}

void Processor::applyCommand(
    const Buffer::SetColliderShape& command,
    float)
{
    Collider* collider = physicsStorage.tryGetCollider(command.collider);

    if (!collider || !applyShape(*collider, command.shape)) {
        return;
    }

    refreshBodySpatialState(collider->rigidBodyHandle);
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
    refreshBodySpatialState(collider->rigidBodyHandle);
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
            body.colliderHandles.empty()) {
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

        if (body.colliderHandles.empty()) {
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
    // #TODO: Combine inertia for compound bodies. 
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

    // worldScale already contains body.scale * collider.localScale.
    // Multiplying by body.scale again makes elongated bodies artificially
    // resistant to angular impulses because their inertia grows from a
    // squared scale.
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
    // #TODO: We don't need to update each collider 
    // just because one collider or the body has changed.

    RigidBody& body = physicsStorage.getBody(bodyHandle);

    // If the body has no colliders, reset its AABB, 
    // inverse radius and remove it from the broadphase
    if (body.colliderHandles.empty()) {
        body.aabb = AABB();
        body.invRadius = 0.0f;
        broadphaseManager.remove(bodyHandle);
        return;
    }

    Collider& firstCollider = 
        physicsStorage.getCollider(body.colliderHandles.front());

    AABB combinedAABB = firstCollider.getAABB();

    // Update all colliders and compute the combined AABB
    for (ColliderHandle colliderHandle : body.colliderHandles) {
        Collider& collider = physicsStorage.getCollider(colliderHandle);

        collider.updateWorldPose(body.pose, body.scale);
        collider.updateShape();
        collider.updateAABB();

        const AABB& colliderAABB = collider.getAABB();

        combinedAABB.growToInclude(colliderAABB.worldMin);
        combinedAABB.growToInclude(colliderAABB.worldMax);
    }

    // Compute the combined AABB's center and half extents
    combinedAABB.worldCenter =
        (combinedAABB.worldMin + combinedAABB.worldMax) * 0.5f;

    combinedAABB.worldHalfExtents =
        (combinedAABB.worldMax - combinedAABB.worldMin) * 0.5f;

    body.aabb = combinedAABB;

    const float radius = glm::length(combinedAABB.worldHalfExtents);
    body.invRadius = radius > 0.0f ? 1.0f / radius : 0.0f;

    if (shouldRefreshInertia) {
        refreshBodyInertia(body);
    }

    if (body.broadphaseHandle.bucket == BroadphaseBucket::None) {
        broadphaseManager.add(bodyHandle, getBodyBucket(physicsStorage, body));
        return;
    }
}

} // namespace physics::internal::cmd
