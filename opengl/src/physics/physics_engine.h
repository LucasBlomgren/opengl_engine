#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "physics/public/collider_desc.h"
#include "physics/public/collider_state.h"
#include "physics/public/physics_contact_types.h"
#include "physics/public/physics_debug_types.h"
#include "physics/public/physics_handles.h"
#include "physics/public/physics_query_types.h"
#include "physics/public/physics_scene_types.h"
#include "physics/public/rigid_body_desc.h"
#include "physics/public/rigid_body_state.h"

#include "core/timer.h"

#include "physics/broadphase/broadphase_manager.h"
#include "physics/colliders/tri.h"
#include "physics/engine/commands/physics_command_buffer.h"
#include "physics/narrowphase/collision_manifold.h"
#include "physics/narrowphase/narrowphase_manager.h"
#include "physics/solver/pgs_solver.h"
#include "physics/world/physics_world.h"
#include "physics/world/runtime_caches.h"

class EngineState;
class FrameTimers;

namespace physics {

class Engine {
public:
    //======================================
    // Construction and ownership
    //======================================
    Engine() = default;
    ~Engine() = default;

    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;
    Engine(Engine&&) = delete;
    Engine& operator=(Engine&&) = delete;

    //======================================
    // Initialization and scene lifetime
    //======================================
    void init(FrameTimers* frameTimers);
    void setupScene(const std::vector<Triangle>& terrainTriangles);
    void clear();

    //======================================
    // Simulation
    //======================================
    void prepareStepLoop();
    void step(float deltaTime, EngineState& engine);

    //================================================
    // Rigid body & collider creation and destruction
    //================================================
    [[nodiscard]] BodyHandle createRigidBody(const BodyDesc& desc);
    bool destroyRigidBody(BodyHandle body);

    [[nodiscard]] ColliderHandle createCollider(
        BodyHandle body, 
        const ColliderDesc& desc);
    bool destroyCollider(ColliderHandle collider);

    //======================================
    // Rigid body commands
    //======================================
    bool applyLinearImpulse(BodyHandle body, const glm::vec3& impulse);
    bool setLinearVelocity(BodyHandle body, const glm::vec3& velocity);
    bool setAngularVelocity(BodyHandle body, const glm::vec3& velocity);
    bool setKinematicTarget(BodyHandle body, const Pose& target);

    bool setRigidBodyTransform(
        BodyHandle body,
        const Pose& pose,
        const glm::vec3& scale);

    bool setRigidBodySleepState(BodyHandle body, bool asleep);
    bool setRigidBodyType(BodyHandle body, BodyType type);
    bool setRigidBodyMotionControl(BodyHandle body, MotionControl motionControl);

    bool setRigidBodyResponseMode(
        BodyHandle body,
        ResponseMode responseMode);

    bool setRigidBodyMass(BodyHandle body, float mass);
    bool setRigidBodyAllowGravity(BodyHandle body, bool allowGravity);
    bool setRigidBodyAllowSleep(BodyHandle body, bool allowSleep);

    bool setRigidBodyCanMoveLinearly(
        BodyHandle body,
        bool canMoveLinearly);

    //======================================
    // Collider commands
    //======================================
    bool setColliderLocalPose(ColliderHandle collider, const Pose& localPose);

    bool setColliderLocalTransform(
        ColliderHandle collider,
        const Pose& localPose,
        const glm::vec3& localScale);

    bool setColliderShape(
        ColliderHandle collider,
        const ColliderShapeDesc& shape);

    bool setColliderEnabled(ColliderHandle collider, bool enabled);
    bool setColliderTrigger(ColliderHandle collider, bool isTrigger);

    //======================================
    // State queries
    //======================================
    std::optional<BodyState> getRigidBodyState(BodyHandle body) const;
    std::optional<ColliderState> getColliderState(ColliderHandle collider) const;

    //======================================
    // Physics queries
    //======================================
    RaycastHit raycast(
        const Ray& ray,
        BodyHandle ignoredBody = {});

    std::vector<BodyHandle> queryBodies(
        const physics::AABB& bounds,
        BodySet bodySet) const;

    //======================================
    // Scene-wide commands
    //======================================
    void sleepAllObjects();
    void awakenAllObjects();

    //======================================
    // Solver configuration
    //======================================
    int getPgsIterations() const;
    void setPgsIterations(int iterations);

    //======================================
    // Simulation output
    //======================================
    std::vector<ExternalMotionContact>& getExternalMotionContacts();

    //======================================
    // Debug state
    //======================================
    debug::Data getDebugData() const;
    debug::StepPhase getDebugPhase() const;

    //======================================
    // Debug visualization
    //======================================
    void updateBVHRenderData(const debug::BvhType& type, bool update);

    std::vector<physics::AABB> getDebugSweeps() const;

    std::vector<debug::SpeculativeContact>
        getDebugSpeculativeContacts() const;

    std::vector<debug::Contact> getDebugContacts() const;

    //======================================
    // Debug spatial data
    //======================================
    const std::vector<BodyHandle>& getAwakeList() const;
    debug::Bvh getDebugBvh(debug::BvhType type) const;
    debug::Bvh getTerrainDebugBvh() const;

private:
    //=========================================
    // Simulation phases
    //=========================================
    void beginPhysicsStep(float outerDt);
    void stepDiscrete(float deltaTime);
    void endPhysicsStep(float outerDt);

    //=========================================
    // Body integration and collider updates
    //=========================================
    void integrateForcesAndVelocities(
        const std::vector<BodyHandle>& bodies, 
        float dt);
    void integratePositionsAndColliders(
        const std::vector<BodyHandle>& bodies, 
        float dt);
    void updateCollidersAndBodyAABB(internal::RigidBody* body);
    void refreshBodyInertia(internal::RigidBody& body);

    void refreshBodySpatialState(
        BodyHandle bodyHandle,
        bool refreshInertia = true);

    //=========================================
    // Sleep and wake processing
    //=========================================
    void processWakeList();
    void processSleepList(float outerDt);
    void decideSleep();
    void updateSleepThresholds();
    void addSleepDamping();

    //=========================================
    // Pending command processing
    //=========================================
    void processPendingCommands();

    void processLifecycleCommands(
        const internal::CommandBuffer::Batch& batch);

    void applyMutationCommands(
        const std::vector<internal::CommandBuffer::Mutation>& mutations);

    //=========================================
    // Rigid body command application
    //=========================================
    void applyCommand(const internal::CommandBuffer::ApplyLinearImpulse&);
    void applyCommand(const internal::CommandBuffer::SetLinearVelocity&);
    void applyCommand(const internal::CommandBuffer::SetAngularVelocity&);
    void applyCommand(const internal::CommandBuffer::SetKinematicTarget&);
    void applyCommand(const internal::CommandBuffer::SetRigidBodyTransform&);
    void applyCommand(const internal::CommandBuffer::SetRigidBodySleepState&);
    void applyCommand(const internal::CommandBuffer::SetRigidBodyType&);
    void applyCommand(const internal::CommandBuffer::SetRigidBodyMotionControl&);
    void applyCommand(const internal::CommandBuffer::SetRigidBodyResponseMode&);
    void applyCommand(const internal::CommandBuffer::SetRigidBodyMass&);
    void applyCommand(const internal::CommandBuffer::SetRigidBodyAllowGravity&);
    void applyCommand(const internal::CommandBuffer::SetRigidBodyAllowSleep&);
    void applyCommand(const internal::CommandBuffer::SetRigidBodyCanMoveLinearly&);

    //=========================================
    // Collider command application
    //=========================================
    void applyCommand(const internal::CommandBuffer::SetColliderLocalPose&);
    void applyCommand(const internal::CommandBuffer::SetColliderLocalTransform&);
    void applyCommand(const internal::CommandBuffer::SetColliderShape&);
    void applyCommand(const internal::CommandBuffer::SetColliderEnabled&);
    void applyCommand(const internal::CommandBuffer::SetColliderTrigger&);

    //=========================================
    // Scene command application
    //=========================================
    void applyCommand(const internal::CommandBuffer::SleepAllObjects&);
    void applyCommand(const internal::CommandBuffer::AwakenAllObjects&);

    //=========================================
    // Broadphase maintenance
    //=========================================
    void setBVHDirty(BodyHandle handle);

    //=========================================
    // Contact processing
    //=========================================
    void updateContactCache();
    void resolveCollisions();

    //=========================================
    // Simulation state
    //=========================================
    debug::StepPhase debugPhase = debug::StepPhase::Ready;

    float dt = 0.0f;
    float savedPhysicsSurpassedTime = 0.0f;

    uint32_t contactsThisFrame = 0;
    uint32_t speculativeContactsThisFrame = 0;
    int pgsIterations = 8;

    //=========================================
    // External dependencies
    //=========================================
    FrameTimers* frameTimers = nullptr;
    std::vector<internal::Tri> terrainTriangles;

    //=========================================
    // External command input
    //=========================================
    internal::CommandBuffer commandBuffer;

    //=========================================
    // World storage and runtime caches
    //=========================================
    internal::PhysicsWorld physicsWorld;
    internal::RuntimeCaches caches;

    //=========================================
    // Step-local working data
    //=========================================
    internal::PairBatch pairBatch;
    internal::ContactBatch contactBatch;

    //=========================================
    // Physics subsystems
    //=========================================
    internal::BroadphaseManager broadphaseManager;
    internal::NarrowphaseManager narrowphaseManager;
    internal::PGSSolver pgsSolver;
    std::unique_ptr<internal::CollisionManifold> collisionManifold;

    //=========================================
    // Sleep and wake queues
    //=========================================
    std::vector<BodyHandle> toWake;
    std::vector<BodyHandle> toSleep;

    //=========================================
    // Persistent contact data
    //=========================================
    std::unordered_map<size_t, internal::Contact> contactCache;

    //=========================================
    // Debug data
    //=========================================
    std::vector<internal::AABB> debugSweeps;
    std::vector<internal::DebugSpeculativeContact> debugSpeculativeContacts;
};

}
