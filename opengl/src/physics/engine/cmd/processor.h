#pragma once

#include "physics/engine/cmd/buffer.h"

namespace physics::internal {

class BroadphaseManager;
class PhysicsWorld;
class RigidBody;

namespace cmd {

class Processor {
public:
    Processor(
        PhysicsWorld& physicsWorld,
        BroadphaseManager& broadphaseManager);

    void process(
        Buffer::Batch& batch,
        float dt);

private:
    void processLifecycleCommands(
        Buffer::Batch& batch);

    void applyMutationCommands(
        const std::vector<Buffer::Mutation>& mutations,
        float dt);

    void refreshBodyInertia(RigidBody& body);

    void refreshBodySpatialState(
        BodyHandle bodyHandle,
        bool refreshInertia = true);

    void applyCommand(const Buffer::ApplyLinearImpulse&, float dt);
    void applyCommand(const Buffer::SetLinearVelocity&, float dt);
    void applyCommand(const Buffer::SetAngularVelocity&, float dt);
    void applyCommand(const Buffer::SetKinematicTarget&, float dt);
    void applyCommand(const Buffer::SetRigidBodyTransform&, float dt);
    void applyCommand(const Buffer::SetRigidBodySleepState&, float dt);
    void applyCommand(const Buffer::SetRigidBodyType&, float dt);
    void applyCommand(const Buffer::SetRigidBodyMotionControl&, float dt);
    void applyCommand(const Buffer::SetRigidBodyResponseMode&, float dt);
    void applyCommand(const Buffer::SetRigidBodyMass&, float dt);
    void applyCommand(const Buffer::SetRigidBodyAllowGravity&, float dt);
    void applyCommand(const Buffer::SetRigidBodyAllowSleep&, float dt);
    void applyCommand(const Buffer::SetRigidBodyCanMoveLinearly&, float dt);
    void applyCommand(const Buffer::SetColliderLocalPose&, float dt);
    void applyCommand(const Buffer::SetColliderLocalTransform&, float dt);
    void applyCommand(const Buffer::SetColliderShape&, float dt);
    void applyCommand(const Buffer::SetColliderEnabled&, float dt);
    void applyCommand(const Buffer::SetColliderTrigger&, float dt);
    void applyCommand(const Buffer::SleepAllObjects&, float dt);
    void applyCommand(const Buffer::AwakenAllObjects&, float dt);

    PhysicsWorld& physicsWorld;
    BroadphaseManager& broadphaseManager;
};

}

}
