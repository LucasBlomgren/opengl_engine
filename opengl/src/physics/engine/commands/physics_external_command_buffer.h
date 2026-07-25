#pragma once

#include <cstddef>
#include <variant>
#include <vector>

#include <glm/vec3.hpp>

#include "physics/public/physics_handles.h"
#include "physics/public/physics_types.h"

class PhysicsExternalCommandBuffer {
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

    struct SetRigidBodyAwake {
        RigidBodyHandle body;
        bool awake;
    };

    struct SetRigidBodyAsleep {
        RigidBodyHandle body;
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
        SetRigidBodyAwake,
        SetRigidBodyAsleep,
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
    void queueApplyLinearImpulse(RigidBodyHandle body, const glm::vec3& impulse);
    void queueSetLinearVelocity(RigidBodyHandle body, const glm::vec3& velocity);
    void queueSetAngularVelocity(RigidBodyHandle body, const glm::vec3& velocity);
    void queueSetKinematicTarget(RigidBodyHandle body, const PhysicsPose& target);
    void queueSetRigidBodyAwake(RigidBodyHandle body);
    void queueSetRigidBodyAsleep(RigidBodyHandle body);
    void queueSetRigidBodyType(RigidBodyHandle body, BodyType type);
    void queueSetRigidBodyMotionControl(RigidBodyHandle body, MotionControl motionControl);

    //=========================================
    // Collider command recording
    //=========================================
    void queueSetColliderLocalPose(ColliderHandle collider, const PhysicsPose& localPose);
    void queueSetColliderEnabled(ColliderHandle collider, bool enabled);
    void queueSetColliderTrigger(ColliderHandle collider, bool isTrigger);

    //=========================================
    // Scene-wide command recording
    //=========================================
    void queueSleepAllObjects();
    void queueAwakenAllObjects();
    void queueSyncBodyFromTransform(RigidBodyHandle body);

private:
    std::vector<RigidBodyHandle> bodyCreates;
    std::vector<ColliderHandle> colliderCreates;
    std::vector<RigidBodyHandle> bodyDestroys;
    std::vector<ColliderHandle> colliderDestroys;
    std::vector<Mutation> mutations;
};