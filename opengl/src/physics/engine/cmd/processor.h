#pragma once

#include "physics/engine/cmd/buffer.h"

namespace physics::internal {

class BroadphaseManager;
class SleepManager;
class PhysicsWorld;
class RigidBody;

namespace cmd {

class Processor {
public:
    Processor(
        PhysicsWorld& physicsWorld,
        SleepManager& sleepManager,
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
    void applyCommand(const Buffer::SetBodyTransform&, float dt);
    void applyCommand(const Buffer::SetBodySleepState&, float dt);
    void applyCommand(const Buffer::SetBodyType&, float dt);
    void applyCommand(const Buffer::SetBodyReportContacts&, float dt);
    void applyCommand(const Buffer::SetBodyMass&, float dt);
    void applyCommand(const Buffer::SetBodyAllowGravity&, float dt);
    void applyCommand(const Buffer::SetBodyAllowSleep&, float dt);
    void applyCommand(const Buffer::SetColliderLocalPose&, float dt);
    void applyCommand(const Buffer::SetColliderLocalTransform&, float dt);
    void applyCommand(const Buffer::SetColliderShape&, float dt);
    void applyCommand(const Buffer::SetColliderEnabled&, float dt);
    void applyCommand(const Buffer::SetColliderTrigger&, float dt);
    void applyCommand(const Buffer::SleepAllObjects&, float dt);
    void applyCommand(const Buffer::AwakenAllObjects&, float dt);

    PhysicsWorld& physicsWorld;
    SleepManager& sleepManager;
    BroadphaseManager& broadphaseManager;
};

}

}
