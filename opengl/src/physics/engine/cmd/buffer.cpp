#include "pch.h"

#include "physics/engine/cmd/buffer.h"

#include <algorithm>
#include <utility>

namespace physics::internal::cmd {

//=========================================
// Storage management
//=========================================
void Buffer::reserve(
    size_t bodyCount,
    size_t colliderCount,
    size_t mutationCount) {
    bodyCreates.reserve(bodyCount);
    bodyDestroys.reserve(bodyCount);
    colliderCreates.reserve(colliderCount);
    colliderDestroys.reserve(colliderCount);
    mutations.reserve(mutationCount);
}

void Buffer::clear(PhysicsWorld& physicsWorld) {
    for (const auto& entry : colliderCreates) {
        physicsWorld.releaseColliderReservation(entry.first);
    }

    for (const auto& entry : bodyCreates) {
        physicsWorld.releaseBodyReservation(entry.first);
    }

    bodyCreates.clear();
    colliderCreates.clear();
    bodyDestroys.clear();
    colliderDestroys.clear();
    mutations.clear();
}

Buffer::Batch Buffer::take() {
    Batch batch;

    auto transferNodes = []<class AssociativeContainer>(
        AssociativeContainer& source,
        AssociativeContainer& destination)
    {
        destination.reserve(
            destination.size() + source.size()
        );

        while (!source.empty()) {
            destination.insert(
                source.extract(source.begin())
            );
        }
    };

    transferNodes(bodyCreates, batch.bodyCreates);
    transferNodes(colliderCreates, batch.colliderCreates);
    transferNodes(bodyDestroys, batch.bodyDestroys);
    transferNodes(colliderDestroys, batch.colliderDestroys);

    batch.mutations.reserve(mutations.size());
    for (Mutation& mutation : mutations) {
        batch.mutations.push_back(std::move(mutation));
    }

    mutations.clear();

    return batch;
}

//=========================================
// Lifecycle create
//=========================================
void Buffer::recordBodyCreate(
    BodyHandle body,
    RigidBody&& rigidBody)
{
    bodyCreates.insert_or_assign(
        body,
        std::move(rigidBody)
    );
}

void Buffer::recordColliderCreate(
    ColliderHandle collider,
    Collider&& colliderData)
{
    colliderCreates.insert_or_assign(
        collider,
        std::move(colliderData)
    );
}

//=========================================
// Lifecycle destroy
//=========================================
bool Buffer::recordBodyDestroy(
    BodyHandle body,
    PhysicsWorld& physicsWorld)
{
    if (bodyDestroys.contains(body)) {
        return false;
    }

    std::unordered_set<ColliderHandle> removedColliders;

    // Mark the body and its colliders for destruction
    // (if it exists in the active world)
    auto pendingCreate = bodyCreates.find(body);
    if (pendingCreate == bodyCreates.end()) {
        RigidBody* activeBody = physicsWorld.tryGetBody(body);

        if (!activeBody) {
            return false;
        }

        bodyDestroys.insert(body);
        removedColliders.insert(
            activeBody->colliderHandles.begin(),
            activeBody->colliderHandles.end()
        );
    }

    // Remove any pending collider creates that are associated with this body
    for (auto collider = colliderCreates.begin();
        collider != colliderCreates.end();) {

        if (collider->second.rigidBodyHandle != body) {
            ++collider;
            continue;
        }

        const ColliderHandle colliderHandle = collider->first;

        removedColliders.insert(colliderHandle);
        physicsWorld.releaseColliderReservation(colliderHandle);

        collider = colliderCreates.erase(collider);
    }

    // Absorb any pending collider destroys that are associated with this body
    for (auto collider = colliderDestroys.begin();
        collider != colliderDestroys.end();) {
        if (collider->second != body) {
            ++collider;
            continue;
        }

        removedColliders.insert(collider->first);
        collider = colliderDestroys.erase(collider);
    }

    // Remove any pending mutations that target this body or its colliders
    removeMutationsTargeting(
        body,
        removedColliders
    );

    // If the body was pending creation, remove it from the pending creates
    if (pendingCreate != bodyCreates.end()) {
        bodyCreates.erase(pendingCreate);
        physicsWorld.releaseBodyReservation(body);
    }

    return true;
}

bool Buffer::recordColliderDestroy(
    ColliderHandle collider,
    PhysicsWorld& physicsWorld)
{
    if (colliderDestroys.contains(collider)) {
        return false;
    }

    auto create = colliderCreates.find(collider);

    // If the collider is pending creation, remove it from the 
    // pending creates and release its reservation
    if (create != colliderCreates.end()) {
        const BodyHandle parent = create->second.rigidBodyHandle;

        if (RigidBody* pendingBody = 
            tryGetPendingBodyCreate(parent)) 
        {
            std::vector<ColliderHandle>& handles = 
                pendingBody->colliderHandles;

            std::erase(handles, collider);
        }

        colliderCreates.erase(create);
        removeMutationsTargeting(collider);
        physicsWorld.releaseColliderReservation(collider);
        return true;
    }

    // If the collider is not active or its parent body is pending 
    // destruction, don't destroy it (it will be destroyed when the parent body is destroyed)
    const Collider* activeCollider =
        physicsWorld.tryGetCollider(collider);

    if (!activeCollider ||
        bodyDestroys.contains(activeCollider->rigidBodyHandle)) {
        return false;
    }

    // If the collider is active, mark it for destruction and 
    // remove any pending mutations that target it
    auto [it, inserted] = colliderDestroys.emplace(
        collider,
        activeCollider->rigidBodyHandle
    );

    if (inserted) {
        removeMutationsTargeting(collider);
    }

    return inserted;
}

void Buffer::removeMutationsTargeting(
    BodyHandle body,
    const std::unordered_set<ColliderHandle>& colliders)
{
    auto targetsBody = [](
        const Buffer::Mutation& mutation,
        BodyHandle body)
    {
        return std::visit([body](const auto& command) {
            if constexpr (requires { command.body; }) {
                return command.body == body;
            }
            else {
                return false;
            }
            }, mutation);
    };

    auto targetsCollider = [](
        const Buffer::Mutation& mutation,
        const std::unordered_set<ColliderHandle>& colliders)
    {
        return std::visit([&colliders](const auto& command) {
            if constexpr (requires { command.collider; }) {
                return colliders.contains(command.collider);
            }
            else {
                return false;
            }
            }, mutation);
    };

    // Remove any pending mutations that target this body or its colliders
    std::erase_if(
        mutations,
        [&](const Mutation& mutation) {
            return targetsBody(mutation, body) ||
                targetsCollider(mutation, colliders);
        }
    );
}

void Buffer::removeMutationsTargeting(
    ColliderHandle collider)
{
    auto targetsCollider = [](
        const Buffer::Mutation& mutation,
        ColliderHandle collider)
    {
        return std::visit([collider](const auto& command) {
            if constexpr (requires { command.collider; }) {
                return command.collider == collider;
            }
            else {
                return false;
            }
            }, mutation);
    };

    // Remove any pending mutations that target this collider
    std::erase_if(
        mutations,
        [collider, &targetsCollider](const Mutation& mutation) {
            return targetsCollider(mutation, collider);
        }
    );
}

//=========================================
// Pending create/destroy queries
//=========================================
RigidBody* Buffer::tryGetPendingBodyCreate(BodyHandle body) {
    auto found = bodyCreates.find(body);
    return found != bodyCreates.end() ? &found->second : nullptr;
}
const RigidBody* Buffer::tryGetPendingBodyCreate(BodyHandle body) const {
    auto found = bodyCreates.find(body);
    return found != bodyCreates.end() ? &found->second : nullptr;
}

Collider* Buffer::tryGetPendingColliderCreate(ColliderHandle collider) {
    auto found = colliderCreates.find(collider);
    return found != colliderCreates.end() ? &found->second : nullptr;
}
const Collider* Buffer::tryGetPendingColliderCreate(ColliderHandle collider) const {
    auto found = colliderCreates.find(collider);
    return found != colliderCreates.end() ? &found->second : nullptr;
}

bool Buffer::isBodyPendingDestroy(
    BodyHandle body) const {
    return bodyDestroys.contains(body);
}
bool Buffer::isColliderPendingDestroy(
    ColliderHandle collider) const {
    return colliderDestroys.contains(collider);
}

//=========================================
// Rigid body command recording
//=========================================
void Buffer::recordApplyLinearImpulse(
    BodyHandle body,
    const glm::vec3& impulse) {
    mutations.emplace_back(ApplyLinearImpulse{ body, impulse });
}

void Buffer::recordSetLinearVelocity(
    BodyHandle body,
    const glm::vec3& velocity) {
    mutations.emplace_back(SetLinearVelocity{ body, velocity });
}

void Buffer::recordSetAngularVelocity(
    BodyHandle body,
    const glm::vec3& velocity) {
    mutations.emplace_back(SetAngularVelocity{ body, velocity });
}

void Buffer::recordSetKinematicTarget(
    BodyHandle body,
    const Pose& target) {
    mutations.emplace_back(SetKinematicTarget{ body, target });
}

void Buffer::recordSetRigidBodyTransform(
    BodyHandle body,
    const Pose& pose,
    const glm::vec3& scale) {
    mutations.emplace_back(SetRigidBodyTransform{ body, pose, scale });
}

void Buffer::recordSetRigidBodySleepState(
    BodyHandle body,
    bool asleep) {
    mutations.emplace_back(SetRigidBodySleepState{ body, asleep });
}

void Buffer::recordSetRigidBodyType(
    BodyHandle body,
    BodyType type) {
    mutations.emplace_back(SetRigidBodyType{ body, type });
}

void Buffer::recordSetRigidBodyMotionControl(
    BodyHandle body,
    MotionControl motionControl) {
    mutations.emplace_back(SetRigidBodyMotionControl{ body, motionControl });
}

void Buffer::recordSetRigidBodyResponseMode(
    BodyHandle body,
    ResponseMode responseMode) {
    mutations.emplace_back(SetRigidBodyResponseMode{ body, responseMode });
}

void Buffer::recordSetRigidBodyMass(
    BodyHandle body,
    float mass) {
    mutations.emplace_back(SetRigidBodyMass{ body, mass });
}

void Buffer::recordSetRigidBodyAllowGravity(
    BodyHandle body,
    bool allowGravity) {
    mutations.emplace_back(SetRigidBodyAllowGravity{ body, allowGravity });
}

void Buffer::recordSetRigidBodyAllowSleep(
    BodyHandle body,
    bool allowSleep) {
    mutations.emplace_back(SetRigidBodyAllowSleep{ body, allowSleep });
}

//=========================================
// Collider command recording
//=========================================
void Buffer::recordSetColliderLocalPose(
    ColliderHandle collider,
    const Pose& localPose) {
    mutations.emplace_back(SetColliderLocalPose{ collider, localPose });
}

void Buffer::recordSetColliderLocalTransform(
    ColliderHandle collider,
    const Pose& localPose,
    const glm::vec3& localScale) {
    mutations.emplace_back(SetColliderLocalTransform{
        collider,
        localPose,
        localScale
        });
}

void Buffer::recordSetColliderShape(
    ColliderHandle collider,
    const ColliderShapeDesc& shape) {
    mutations.emplace_back(SetColliderShape{ collider, shape });
}

void Buffer::recordSetColliderEnabled(
    ColliderHandle collider,
    bool enabled) {
    mutations.emplace_back(SetColliderEnabled{ collider, enabled });
}

void Buffer::recordSetColliderTrigger(
    ColliderHandle collider,
    bool isTrigger) {
    mutations.emplace_back(SetColliderTrigger{ collider, isTrigger });
}

//=========================================
// Scene-wide command recording
//=========================================
void Buffer::recordSleepAllObjects() {
    mutations.emplace_back(SleepAllObjects{});
}

void Buffer::recordAwakenAllObjects() {
    mutations.emplace_back(AwakenAllObjects{});
}
}