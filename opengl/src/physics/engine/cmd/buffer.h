#pragma once

#include <variant>
#include <vector>
#include <glm/vec3.hpp>

#include "physics/public/collider_desc.h"
#include "physics/public/handles.h"
#include "physics/public/physics_types.h"

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

    struct SetRigidBodyCanMoveLinearly {
        BodyHandle body;
        bool canMoveLinearly;
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
        SetRigidBodyCanMoveLinearly,
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
        std::vector<BodyHandle> bodyCreates;
        std::vector<ColliderHandle> colliderCreates;
        std::vector<BodyHandle> bodyDestroys;
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
    void recordBodyCreate(BodyHandle body);
    void recordColliderCreate(ColliderHandle collider);

    bool recordBodyDestroy(BodyHandle body);
    bool recordColliderDestroy(ColliderHandle collider);

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

    void recordSetRigidBodyCanMoveLinearly(
        BodyHandle body,
        bool canMoveLinearly);

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
    std::vector<BodyHandle> bodyCreates;
    std::vector<ColliderHandle> colliderCreates;
    std::vector<BodyHandle> bodyDestroys;
    std::vector<ColliderHandle> colliderDestroys;
    std::vector<Mutation> mutations;
};

}
