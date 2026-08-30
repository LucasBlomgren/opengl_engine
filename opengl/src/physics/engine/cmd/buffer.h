#pragma once

#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>
#include <glm/vec3.hpp>

#include "physics/public/collider_desc.h"
#include "physics/public/handles.h"
#include "physics/public/physics_types.h"

#include "physics/world/physics_world.h"
#include "physics/bodies/rigidbody.h"
#include "physics/colliders/collider.h"

namespace physics::internal::cmd {

class Buffer {
public:
    //=========================================
    // Rigid body mutation commands
    //=========================================
    struct ApplyLinearImpulse {
        BodyHandle body;
        glm::vec3 impulse;
    };

    struct SetLinearVelocity {
        BodyHandle body;
        glm::vec3 velocity;
    };

    struct SetAngularVelocity {
        BodyHandle body;
        glm::vec3 velocity;
    };

    struct SetKinematicTarget {
        BodyHandle body;
        Pose target;
    };

    struct SetRigidBodyTransform {
        BodyHandle body;
        Pose pose;
        glm::vec3 scale;
    };

    struct SetRigidBodySleepState {
        BodyHandle body;
        bool asleep;
    };

    struct SetRigidBodyType {
        BodyHandle body;
        BodyType type;
    };

    struct SetRigidBodyMotionControl {
        BodyHandle body;
        MotionControl motionControl;
    };

    struct SetRigidBodyResponseMode {
        BodyHandle body;
        ResponseMode responseMode;
    };

    struct SetRigidBodyMass {
        BodyHandle body;
        float mass;
    };

    struct SetRigidBodyAllowGravity {
        BodyHandle body;
        bool allowGravity;
    };

    struct SetRigidBodyAllowSleep {
        BodyHandle body;
        bool allowSleep;
    };

    //=========================================
    // Collider mutation commands
    //=========================================
    struct SetColliderLocalPose {
        ColliderHandle collider;
        Pose localPose;
    };

    struct SetColliderLocalTransform {
        ColliderHandle collider;
        Pose localPose;
        glm::vec3 localScale;
    };

    struct SetColliderShape {
        ColliderHandle collider;
        ColliderShapeDesc shape;
    };

    struct SetColliderEnabled {
        ColliderHandle collider;
        bool enabled;
    };

    struct SetColliderTrigger {
        ColliderHandle collider;
        bool isTrigger;
    };

    //=========================================
    // Scene-wide mutation commands
    //=========================================
    struct SleepAllObjects {};
    struct AwakenAllObjects {};

    using Mutation = std::variant<
        ApplyLinearImpulse,
        SetLinearVelocity,
        SetAngularVelocity,
        SetKinematicTarget,
        SetRigidBodyTransform,
        SetRigidBodySleepState,
        SetRigidBodyType,
        SetRigidBodyMotionControl,
        SetRigidBodyResponseMode,
        SetRigidBodyMass,
        SetRigidBodyAllowGravity,
        SetRigidBodyAllowSleep,
        SetColliderLocalPose,
        SetColliderLocalTransform,
        SetColliderShape,
        SetColliderEnabled,
        SetColliderTrigger,
        SleepAllObjects,
        AwakenAllObjects
    >;

    //=========================================
    // Complete command batch
    //=========================================
    struct Batch {
        std::unordered_map<BodyHandle, RigidBody> bodyCreates;
        std::unordered_map<ColliderHandle, Collider> colliderCreates;
        std::unordered_set<BodyHandle> bodyDestroys;
        std::unordered_map<ColliderHandle, BodyHandle> colliderDestroys;
        std::vector<Mutation> mutations;

        bool empty() const noexcept {
            return bodyCreates.empty() &&
                colliderCreates.empty() &&
                bodyDestroys.empty() &&
                colliderDestroys.empty() &&
                mutations.empty();
        }
    };

    //=========================================
    // Storage management
    //=========================================
    void reserve(size_t bodyCount, size_t colliderCount, size_t mutationCount);
    void clear(PhysicsWorld& physicsWorld);
    [[nodiscard]] Batch take();

    //=========================================
    // Lifecycle recording
    //=========================================
    void recordBodyCreate(BodyHandle body, RigidBody&& rigidBody);
    void recordColliderCreate(ColliderHandle collider, Collider&& colliderData);

    bool recordBodyDestroy(BodyHandle body, PhysicsWorld& physicsWorld);
    bool recordColliderDestroy(ColliderHandle collider, PhysicsWorld& physicsWorld);

    RigidBody* tryGetPendingBodyCreate(BodyHandle body);
    const RigidBody* tryGetPendingBodyCreate(BodyHandle body) const;

    Collider* tryGetPendingColliderCreate(ColliderHandle collider);
    const Collider* tryGetPendingColliderCreate(ColliderHandle collider) const;

    [[nodiscard]] bool isBodyPendingDestroy(BodyHandle body) const;
    [[nodiscard]] bool isColliderPendingDestroy(ColliderHandle collider) const;

    //=========================================
    // Rigid body command recording
    //=========================================
    void recordApplyLinearImpulse(BodyHandle body, const glm::vec3& impulse);
    void recordSetLinearVelocity(BodyHandle body, const glm::vec3& velocity);
    void recordSetAngularVelocity(BodyHandle body, const glm::vec3& velocity);
    void recordSetKinematicTarget(BodyHandle body, const Pose& target);

    void recordSetRigidBodyTransform(
        BodyHandle body,
        const Pose& pose,
        const glm::vec3& scale);

    void recordSetRigidBodySleepState(BodyHandle body, bool asleep);
    void recordSetRigidBodyType(BodyHandle body, BodyType type);
    void recordSetRigidBodyMotionControl(BodyHandle body, MotionControl motionControl);

    void recordSetRigidBodyResponseMode(
        BodyHandle body,
        ResponseMode responseMode);

    void recordSetRigidBodyMass(BodyHandle body, float mass);
    void recordSetRigidBodyAllowGravity(BodyHandle body, bool allowGravity);
    void recordSetRigidBodyAllowSleep(BodyHandle body, bool allowSleep);

    //=========================================
    // Collider command recording
    //=========================================
    void recordSetColliderLocalPose(ColliderHandle collider, const Pose& localPose);

    void recordSetColliderLocalTransform(
        ColliderHandle collider,
        const Pose& localPose,
        const glm::vec3& localScale);

    void recordSetColliderShape(
        ColliderHandle collider,
        const ColliderShapeDesc& shape);

    void recordSetColliderEnabled(ColliderHandle collider, bool enabled);
    void recordSetColliderTrigger(ColliderHandle collider, bool isTrigger);

    //=========================================
    // Scene-wide command recording
    //=========================================
    void recordSleepAllObjects();
    void recordAwakenAllObjects();

private:
    void absorbColliderDestroysForBody(
        BodyHandle body,
        std::unordered_set<ColliderHandle>& removedColliders);

    void removeMutationsTargeting(
        BodyHandle body,
        const std::unordered_set<ColliderHandle>& colliders);

    void removeMutationsTargeting(ColliderHandle collider);

    std::vector<Mutation> mutations;
    std::unordered_map<BodyHandle, RigidBody> bodyCreates;
    std::unordered_set<BodyHandle> bodyDestroys;
    std::unordered_map<ColliderHandle, Collider> colliderCreates;
    std::unordered_map<ColliderHandle, BodyHandle> colliderDestroys;
};

}
