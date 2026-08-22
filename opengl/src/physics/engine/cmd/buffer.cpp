#include "pch.h"

#include "physics/engine/cmd/buffer.h"

#include <algorithm>
#include <utility>

#include "physics/world/physics_world.h"

namespace physics::internal::cmd {

namespace {
    template<class AssociativeContainer>
    void transferNodes(
        AssociativeContainer& source,
        AssociativeContainer& destination)
    {
        destination.reserve(source.size());

        while (!source.empty()) {
            destination.insert(source.extract(source.begin()));
        }
    }

    bool targetsBody(
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
    }

    bool targetsCollider(
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
    }
}

//=========================================
// Storage management
//=========================================
void Buffer::reserve(
    size_t bodyCount,
    size_t colliderCount,
    size_t mutationCount)
{
    pendingBodyCreates.reserve(bodyCount);
    pendingBodyDestroys.reserve(bodyCount);

    pendingColliderCreates.reserve(colliderCount);
    pendingColliderDestroys.reserve(colliderCount);

    mutations.reserve(mutationCount);
}

void Buffer::clear(PhysicsWorld& physicsWorld) {
    for (const auto& entry : pendingColliderCreates) {
        physicsWorld.releaseColliderReservation(entry.first);
    }

    for (const auto& entry : pendingBodyCreates) {
        physicsWorld.releaseBodyReservation(entry.first);
    }

    pendingBodyCreates.clear();
    pendingColliderCreates.clear();
    pendingBodyDestroys.clear();
    pendingColliderDestroys.clear();
    mutations.clear();
}

bool Buffer::empty() const noexcept {
    return pendingBodyCreates.empty() &&
        pendingColliderCreates.empty() &&
        pendingBodyDestroys.empty() &&
        pendingColliderDestroys.empty() &&
        mutations.empty();
}

Buffer::Batch Buffer::take() {
    Batch batch;

    transferNodes(pendingBodyCreates, batch.bodyCreates);
    transferNodes(pendingColliderCreates, batch.colliderCreates);
    transferNodes(pendingBodyDestroys, batch.bodyDestroys);
    transferNodes(pendingColliderDestroys, batch.colliderDestroys);

    batch.mutations.reserve(mutations.size());
    for (Mutation& mutation : mutations) {
        batch.mutations.push_back(std::move(mutation));
    }

    mutations.clear();

    return batch;
}

//=========================================
// Lifecycle recording
//=========================================
void Buffer::recordBodyCreate(
    BodyHandle body,
    RigidBody&& rigidBody)
{
    pendingBodyCreates.insert_or_assign(
        body,
        std::move(rigidBody)
    );
}

void Buffer::recordColliderCreate(
    ColliderHandle collider,
    Collider&& colliderData)
{
    pendingColliderCreates.insert_or_assign(
        collider,
        std::move(colliderData)
    );
}

bool Buffer::recordBodyDestroy(
    BodyHandle body,
    PhysicsWorld& physicsWorld)
{
    if (pendingBodyDestroys.contains(body)) {
        return false;
    }

    std::unordered_set<ColliderHandle> removedColliders;
    auto create = pendingBodyCreates.find(body);

    if (create != pendingBodyCreates.end()) {
        removedColliders.insert(
            create->second.colliderHandles.begin(),
            create->second.colliderHandles.end()
        );

        cancelPendingCollidersForBody(
            body,
            physicsWorld,
            removedColliders
        );
        absorbColliderDestroysForBody(body, removedColliders);
        removeMutationsTargeting(body, removedColliders);

        pendingBodyCreates.erase(create);
        physicsWorld.releaseBodyReservation(body);
        return true;
    }

    RigidBody* activeBody = physicsWorld.tryGetBody(body);

    if (!activeBody) {
        return false;
    }

    pendingBodyDestroys.insert(body);
    removedColliders.insert(
        activeBody->colliderHandles.begin(),
        activeBody->colliderHandles.end()
    );

    cancelPendingCollidersForBody(
        body,
        physicsWorld,
        removedColliders
    );
    absorbColliderDestroysForBody(body, removedColliders);
    removeMutationsTargeting(body, removedColliders);

    return true;
}

bool Buffer::recordColliderDestroy(
    ColliderHandle collider,
    PhysicsWorld& physicsWorld)
{
    if (pendingColliderDestroys.contains(collider)) {
        return false;
    }

    auto create = pendingColliderCreates.find(collider);

    if (create != pendingColliderCreates.end()) {
        const BodyHandle parent = create->second.rigidBodyHandle;

        if (RigidBody* pendingBody = tryGetPendingBody(parent)) {
            std::vector<ColliderHandle>& colliderHandles =
                pendingBody->colliderHandles;

            colliderHandles.erase(
                std::remove(
                    colliderHandles.begin(),
                    colliderHandles.end(),
                    collider
                ),
                colliderHandles.end()
            );
        }

        pendingColliderCreates.erase(create);
        removeMutationsTargeting(collider);
        physicsWorld.releaseColliderReservation(collider);
        return true;
    }

    const Collider* activeCollider =
        physicsWorld.tryGetCollider(collider);

    if (!activeCollider ||
        pendingBodyDestroys.contains(activeCollider->rigidBodyHandle)) {
        return false;
    }

    auto [it, inserted] = pendingColliderDestroys.emplace(
        collider,
        activeCollider->rigidBodyHandle
    );

    if (inserted) {
        removeMutationsTargeting(collider);
    }

    return inserted;
}

RigidBody* Buffer::tryGetPendingBody(BodyHandle body) {
    auto found = pendingBodyCreates.find(body);
    return found != pendingBodyCreates.end() ? &found->second : nullptr;
}

const RigidBody* Buffer::tryGetPendingBody(
    BodyHandle body) const
{
    auto found = pendingBodyCreates.find(body);
    return found != pendingBodyCreates.end() ? &found->second : nullptr;
}

Collider* Buffer::tryGetPendingCollider(
    ColliderHandle collider)
{
    auto found = pendingColliderCreates.find(collider);
    return found != pendingColliderCreates.end() ? &found->second : nullptr;
}

const Collider* Buffer::tryGetPendingCollider(
    ColliderHandle collider) const
{
    auto found = pendingColliderCreates.find(collider);
    return found != pendingColliderCreates.end() ? &found->second : nullptr;
}

bool Buffer::isBodyPendingDestroy(
    BodyHandle body) const {
    return pendingBodyDestroys.contains(body);
}

bool Buffer::isColliderPendingDestroy(
    ColliderHandle collider) const {
    return pendingColliderDestroys.contains(collider);
}

void Buffer::cancelPendingCollidersForBody(
    BodyHandle body,
    PhysicsWorld& physicsWorld,
    std::unordered_set<ColliderHandle>& removedColliders)
{
    for (auto collider = pendingColliderCreates.begin();
        collider != pendingColliderCreates.end();) {
        if (collider->second.rigidBodyHandle != body) {
            ++collider;
            continue;
        }

        const ColliderHandle colliderHandle = collider->first;
        removedColliders.insert(colliderHandle);
        physicsWorld.releaseColliderReservation(colliderHandle);
        collider = pendingColliderCreates.erase(collider);
    }
}

void Buffer::absorbColliderDestroysForBody(
    BodyHandle body,
    std::unordered_set<ColliderHandle>& removedColliders)
{
    for (auto collider = pendingColliderDestroys.begin();
        collider != pendingColliderDestroys.end();) {
        if (collider->second != body) {
            ++collider;
            continue;
        }

        removedColliders.insert(collider->first);
        collider = pendingColliderDestroys.erase(collider);
    }
}

void Buffer::removeMutationsTargeting(
    BodyHandle body,
    const std::unordered_set<ColliderHandle>& colliders)
{
    mutations.erase(
        std::remove_if(
            mutations.begin(),
            mutations.end(),
            [body, &colliders](const Mutation& mutation) {
                if (targetsBody(mutation, body)) {
                    return true;
                }

                return std::visit([&colliders](const auto& command) {
                    if constexpr (requires { command.collider; }) {
                        return colliders.contains(command.collider);
                    }
                    else {
                        return false;
                    }
                    }, mutation);
            }
        ),
        mutations.end()
    );
}

void Buffer::removeMutationsTargeting(
    ColliderHandle collider)
{
    mutations.erase(
        std::remove_if(
            mutations.begin(),
            mutations.end(),
            [collider](const Mutation& mutation) {
                return targetsCollider(mutation, collider);
            }
        ),
        mutations.end()
    );
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

void Buffer::recordSetRigidBodyCanMoveLinearly(
    BodyHandle body,
    bool canMoveLinearly) {
    mutations.emplace_back(SetRigidBodyCanMoveLinearly{
        body,
        canMoveLinearly
        });
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

//=========================================
// Command buffer inspection
//=========================================
const RigidBody* Buffer::getPendingBodyCreate(BodyHandle body) const {
    auto found = pendingBodyCreates.find(body);
    if (found == pendingBodyCreates.end()) {
        return nullptr;
    }
    return &found->second;
}

const Collider* Buffer::getPendingColliderCreate(ColliderHandle collider) const {
    auto found = pendingColliderCreates.find(collider);
    if (found == pendingColliderCreates.end()) {
        return nullptr;
    }
    return &found->second;

}
}