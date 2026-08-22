#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "physics/public/collider_desc.h"
#include "physics/public/collider_state.h"
#include "physics/public/contact_types.h"
#include "physics/public/debug_types.h"
#include "physics/public/handles.h"
#include "physics/public/query_types.h"
#include "physics/public/rigid_body_desc.h"
#include "physics/public/rigid_body_state.h"

#include "core/timer.h"

#include "physics/broadphase/broadphase_manager.h"
#include "physics/colliders/tri.h"
#include "physics/engine/cmd/buffer.h"
#include "physics/engine/cmd/processor.h"
#include "physics/narrowphase/collision_manifold.h"
#include "physics/narrowphase/narrowphase_manager.h"
#include "physics/solver/pgs_solver.h"
#include "physics/world/physics_world.h"

class EngineState;
class FrameTimers;

namespace physics {

class Engine {
public:
    Engine() = default;
    ~Engine() = default;

    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;
    Engine(Engine&&) = delete;
    Engine& operator=(Engine&&) = delete;

    //======================================
    // Scene
    //======================================
    void init(FrameTimers* frameTimers);
    void activateScene(const std::vector<Triangle>& terrainTriangles);
    void clear();

    void sleepAllObjects();
    void awakenAllObjects();

    //======================================
    // Objects
    //======================================
    [[nodiscard]] BodyHandle createRigidBody(const BodyDesc& desc);
    bool destroyRigidBody(BodyHandle body);

    [[nodiscard]] ColliderHandle createCollider(
        BodyHandle body,
        const ColliderDesc& desc);

    bool destroyCollider(ColliderHandle collider);

    bool applyLinearImpulse(
        BodyHandle body,
        const glm::vec3& impulse);

    bool setLinearVelocity(
        BodyHandle body,
        const glm::vec3& velocity);

    bool setAngularVelocity(
        BodyHandle body,
        const glm::vec3& velocity);

    bool setKinematicTarget(
        BodyHandle body,
        const Pose& target);

    bool setRigidBodyTransform(
        BodyHandle body,
        const Pose& pose,
        const glm::vec3& scale);

    bool setRigidBodySleepState(
        BodyHandle body,
        bool asleep);

    bool setRigidBodyType(
        BodyHandle body,
        BodyType type);

    bool setRigidBodyMotionControl(
        BodyHandle body,
        MotionControl motionControl);

    bool setRigidBodyResponseMode(
        BodyHandle body,
        ResponseMode responseMode);

    bool setRigidBodyMass(
        BodyHandle body,
        float mass);

    bool setRigidBodyAllowGravity(
        BodyHandle body,
        bool allowGravity);

    bool setRigidBodyAllowSleep(
        BodyHandle body,
        bool allowSleep);

    bool setRigidBodyCanMoveLinearly(
        BodyHandle body,
        bool canMoveLinearly);

    bool setColliderLocalPose(
        ColliderHandle collider,
        const Pose& localPose);

    bool setColliderLocalTransform(
        ColliderHandle collider,
        const Pose& localPose,
        const glm::vec3& localScale);

    bool setColliderShape(
        ColliderHandle collider,
        const ColliderShapeDesc& shape);

    bool setColliderEnabled(
        ColliderHandle collider,
        bool enabled);

    bool setColliderTrigger(
        ColliderHandle collider,
        bool isTrigger);

    //======================================
    // Simulation
    //======================================
    void prepareStepLoop();
    void step(float deltaTime, EngineState& engine);

    int getPgsIterations() const;
    void setPgsIterations(int iterations);

    const std::vector<ExternalMotionContact>& getExternalMotionContacts() const;

    //======================================
    // Queries
    //======================================
    std::optional<BodyState> getRigidBodyState(BodyHandle body) const;
    std::optional<ColliderState> getColliderState(ColliderHandle collider) const;
    const std::vector<BodyHandle>& getAwakeList() const;

    RaycastHit raycast(
        const Ray& ray,
        BodyHandle ignoredBody = {});

    std::vector<BodyHandle> queryBodies(
        const physics::AABB& bounds,
        BodySet bodySet) const;

    //======================================
    // Debug
    //======================================
    debug::Data getDebugData() const;
    debug::StepPhase getDebugPhase() const;
    debug::Bvh getDebugBvh(debug::BvhType type) const;
    debug::Bvh getTerrainDebugBvh() const;
    std::vector<debug::Contact> getDebugContacts() const;
    std::vector<debug::SpeculativeContact> getDebugSpeculativeContacts() const;
    std::vector<physics::AABB> getDebugSweeps() const;

    void updateBVHRenderData(const debug::BvhType& type, bool update);

private:
    //======================================
    // Simulation
    //======================================
    void beginPhysicsStep(float outerDt);
    void stepDiscrete(float deltaTime);
    void endPhysicsStep(float outerDt);

    void integrateForcesAndVelocities(
        const std::vector<BodyHandle>& bodies,
        float dt);

    void integratePositionsAndColliders(
        const std::vector<BodyHandle>& bodies,
        float dt);

    void processWakeList();
    void processSleepList(float outerDt);
    void decideSleep();
    void updateSleepThresholds();
    void addSleepDamping();

    void processPendingCommands();

    void updateContactCache();
    void resolveCollisions();

    //======================================
    // Runtime data
    //======================================
    float dt = 0.0f;
    float savedPhysicsSurpassedTime = 0.0f;
    debug::StepPhase debugPhase = debug::StepPhase::Ready;
    FrameTimers* frameTimers = nullptr;

    uint32_t contactsThisFrame = 0;
    uint32_t speculativeContactsThisFrame = 0;
    int pgsIterations = 8;

    internal::PairBatch pairBatch;
    internal::ContactBatch contactBatch;
    std::unordered_map<size_t, internal::Contact> contactCache;
    std::vector<internal::Tri> terrainTriangles;

    std::vector<BodyHandle> toWake;
    std::vector<BodyHandle> toSleep;

    std::vector<internal::AABB> debugSweeps;
    std::vector<internal::DebugSpeculativeContact> debugSpeculativeContacts;

    //======================================
    // Internal systems
    //======================================
    internal::PhysicsWorld physicsWorld;

    internal::BroadphaseManager broadphaseManager;
    internal::NarrowphaseManager narrowphaseManager;
    internal::CollisionManifold collisionManifold;
    internal::PGSSolver pgsSolver;

    internal::cmd::Buffer commandBuffer;
    internal::cmd::Processor commandProcessor{
        physicsWorld,
        broadphaseManager
    };
};

} // namespace physics
