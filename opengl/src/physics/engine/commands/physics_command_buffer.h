#pragma once

#include <cstddef>
#include <variant>
#include <vector>

#include <glm/vec3.hpp>

#include "physics/public/physics_handles.h"
#include "physics/public/physics_types.h"

class PhysicsCommandBuffer {
public:
    //=========================================
    // Rigid body mutation commands
    //=========================================
    struct ApplyLinearImpulse {
        RigidBodyHandle body;
        glm::vec3 impulse;
    };

    struct SetLinearVelocity {
        RigidBodyHandle body;
        glm::vec3 velocity;
    };

    struct SetAngularVelocity {
        RigidBodyHandle body;
        glm::vec3 velocity;
    };

    struct SetKinematicTarget {
        RigidBodyHandle body;
        PhysicsPose target;
    };

    struct SetRigidBodySleepState {
        RigidBodyHandle body;
        bool asleep;
    };

    struct SetRigidBodyType {
        RigidBodyHandle body;
        BodyType type;
    };

    struct SetRigidBodyMotionControl {
        RigidBodyHandle body;
        MotionControl motionControl;
    };

    //=========================================
    // Collider mutation commands
    //=========================================
    struct SetColliderLocalPose {
        ColliderHandle collider;
        PhysicsPose localPose;
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

    struct SyncBodyFromTransform {
        RigidBodyHandle body;
    };

    using Mutation = std::variant<
        ApplyLinearImpulse,
        SetLinearVelocity,
        SetAngularVelocity,
        SetKinematicTarget,
        SetRigidBodySleepState,
        SetRigidBodyType,
        SetRigidBodyMotionControl,
        SetColliderLocalPose,
        SetColliderEnabled,
        SetColliderTrigger,
        SleepAllObjects,
        AwakenAllObjects,
        SyncBodyFromTransform
    >;

    //=========================================
    // Complete command batch
    //=========================================
    struct Batch {
        std::vector<RigidBodyHandle> bodyCreates;
        std::vector<ColliderHandle> colliderCreates;
        std::vector<RigidBodyHandle> bodyDestroys;
        std::vector<ColliderHandle> colliderDestroys;
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
    void clear();

    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] Batch take();

    //=========================================
    // Lifecycle recording
    //=========================================
    void recordBodyCreate(RigidBodyHandle body);
    void recordColliderCreate(ColliderHandle collider);

    bool recordBodyDestroy(RigidBodyHandle body);
    bool recordColliderDestroy(ColliderHandle collider);

    [[nodiscard]] bool isBodyPendingDestroy(RigidBodyHandle body) const;
    [[nodiscard]] bool isColliderPendingDestroy(ColliderHandle collider) const;

    //=========================================
    // Rigid body command recording
    //=========================================
    void recordApplyLinearImpulse(RigidBodyHandle body, const glm::vec3& impulse);
    void recordSetLinearVelocity(RigidBodyHandle body, const glm::vec3& velocity);
    void recordSetAngularVelocity(RigidBodyHandle body, const glm::vec3& velocity);
    void recordSetKinematicTarget(RigidBodyHandle body, const PhysicsPose& target);
    void recordSetRigidBodySleepState(RigidBodyHandle body, bool asleep);
    void recordSetRigidBodyType(RigidBodyHandle body, BodyType type);
    void recordSetRigidBodyMotionControl(RigidBodyHandle body, MotionControl motionControl);

    //=========================================
    // Collider command recording
    //=========================================
    void recordSetColliderLocalPose(ColliderHandle collider, const PhysicsPose& localPose);
    void recordSetColliderEnabled(ColliderHandle collider, bool enabled);
    void recordSetColliderTrigger(ColliderHandle collider, bool isTrigger);

    //=========================================
    // Scene-wide command recording
    //=========================================
    void recordSleepAllObjects();
    void recordAwakenAllObjects();
    void recordSyncBodyFromTransform(RigidBodyHandle body);

private:
    std::vector<RigidBodyHandle> bodyCreates;
    std::vector<ColliderHandle> colliderCreates;
    std::vector<RigidBodyHandle> bodyDestroys;
    std::vector<ColliderHandle> colliderDestroys;
    std::vector<Mutation> mutations;
};
