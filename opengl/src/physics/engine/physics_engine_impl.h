#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "commands/physics_command_buffer.h"

#include "core/timer.h"

#include "physics/public/physics_engine.h"
#include "physics/world/physics_world.h"
#include "physics/world/runtime_caches.h"
#include "physics/colliders/tri.h"
#include "physics/broadphase/broadphase_manager.h"
#include "physics/narrowphase/collision_manifold.h"
#include "physics/narrowphase/narrowphase_manager.h"
#include "physics/solver/pgs_solver.h"

namespace physics::internal {

struct EngineImpl {
public:
//######################################################################################
//   Engine facade interface
//######################################################################################

    //=========================================
    // Initialization and scene lifetime
    //=========================================
    void init(FrameTimers* frameTimers);
    void setupScene(const std::vector<Triangle>& terrainTriangles);
    void clear();

    //=========================================
    // Simulation entry points
    //=========================================
    void prepareStepLoop();
    void step(float deltaTime, EngineState& engine);

    //=========================================
    // Solver configuration
    //=========================================
    int getPgsIterations() const;
    void setPgsIterations(int iterations);

    //=========================================
    // Rigid body command submission
    //=========================================
    [[nodiscard]] BodyHandle submitCreateRigidBody(const BodyDesc& desc);
    bool submitDestroyRigidBody(BodyHandle body);

    bool submitApplyLinearImpulse(BodyHandle body, const glm::vec3& impulse);
    bool submitSetLinearVelocity(BodyHandle body, const glm::vec3& velocity);
    bool submitSetAngularVelocity(BodyHandle body, const glm::vec3& velocity);
    bool submitSetKinematicTarget(BodyHandle body, const Pose& target);
    bool submitSetRigidBodyTransform(
        BodyHandle body,
        const Pose& pose,
        const glm::vec3& scale);

    bool submitSetRigidBodySleepState(BodyHandle body, bool asleep);
    bool submitSetRigidBodyType(BodyHandle body, BodyType type);
    bool submitSetRigidBodyMotionControl(BodyHandle body, MotionControl motionControl);
    bool submitSetRigidBodyResponseMode(
        BodyHandle body,
        ResponseMode responseMode);

    bool submitSetRigidBodyMass(BodyHandle body, float mass);
    bool submitSetRigidBodyAllowGravity(BodyHandle body, bool allowGravity);
    bool submitSetRigidBodyAllowSleep(BodyHandle body, bool allowSleep);
    bool submitSetRigidBodyCanMoveLinearly(
        BodyHandle body,
        bool canMoveLinearly);

    //=========================================
    // Collider command submission
    //=========================================
    [[nodiscard]] ColliderHandle submitCreateCollider(BodyHandle body, const ColliderDesc& desc);
    bool submitDestroyCollider(ColliderHandle collider);

    bool submitSetColliderLocalPose(ColliderHandle collider, const Pose& localPose);
    bool submitSetColliderLocalTransform(
        ColliderHandle collider,
        const Pose& localPose,
        const glm::vec3& localScale);
    bool submitSetColliderShape(
        ColliderHandle collider,
        const ColliderShapeDesc& shape);
    bool submitSetColliderEnabled(ColliderHandle collider, bool enabled);
    bool submitSetColliderTrigger(ColliderHandle collider, bool isTrigger);

    //=========================================
    // Scene command submission
    //=========================================
    void submitSleepAllObjects();
    void submitAwakenAllObjects();

    //=========================================
    // State queries
    //=========================================
    std::optional<BodyState> getRigidBodyState(BodyHandle body) const;
    std::optional<ColliderState> getColliderState(ColliderHandle collider) const;

    //=========================================
    // Spatial queries
    //=========================================
    RaycastHit raycast(
        const Ray& ray,
        BodyHandle ignoredBody);
    std::vector<BodyHandle> queryBodies(
        const physics::AABB& bounds,
        BodySet bodySet) const;

    //=========================================
    // Simulation output
    //=========================================
    std::vector<ExternalMotionContact>& getExternalMotionContacts();

    //=========================================
    // Debug queries
    //=========================================
    physics::debug::Data getDebugData() const;
    physics::debug::StepPhase getDebugPhase() const;
    std::vector<physics::AABB> getDebugSweeps() const;
    std::vector<physics::debug::SpeculativeContact>
        getDebugSpeculativeContacts() const;
    std::vector<physics::debug::Contact> getDebugContacts() const;
    const std::vector<BodyHandle>& getAwakeList() const;
    physics::debug::Bvh getDebugBvh(physics::debug::BvhType type) const;
    physics::debug::Bvh getTerrainDebugBvh() const;

    //=========================================
    // Debug visualization
    //=========================================
    void updateBVHRenderData(const physics::debug::BvhType& type, bool update);

//######################################################################################
//  Internal physics runtime: operations
//######################################################################################

    //=========================================
    // Simulation phases
    //=========================================
    void beginPhysicsStep(float outerDt);
    void stepDiscrete(float deltaTime);
    void endPhysicsStep(float outerDt);

    //=========================================
    // Body integration and collider updates
    //=========================================
    void integrateForcesAndVelocities(const std::vector<BodyHandle>& bodies, float dt);
    void integratePositionsAndColliders(const std::vector<BodyHandle>& bodies, float dt);
    void updateCollidersAndBodyAABB(RigidBody* body);
    void refreshBodyInertia(RigidBody& body);
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
    void processLifecycleCommands(const PhysicsCommandBuffer::Batch& batch);
    void applyMutationCommands(const std::vector<PhysicsCommandBuffer::Mutation>& mutations);

    //=========================================
    // Rigid body command application
    //=========================================
    void applyCommand(const PhysicsCommandBuffer::ApplyLinearImpulse& command);
    void applyCommand(const PhysicsCommandBuffer::SetLinearVelocity& command);
    void applyCommand(const PhysicsCommandBuffer::SetAngularVelocity& command);
    void applyCommand(const PhysicsCommandBuffer::SetKinematicTarget& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyTransform& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodySleepState& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyType& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyMotionControl& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyResponseMode& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyMass& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyAllowGravity& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyAllowSleep& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyCanMoveLinearly& command);

    //=========================================
    // Collider command application
    //=========================================
    void applyCommand(const PhysicsCommandBuffer::SetColliderLocalPose& command);
    void applyCommand(const PhysicsCommandBuffer::SetColliderLocalTransform& command);
    void applyCommand(const PhysicsCommandBuffer::SetColliderShape& command);
    void applyCommand(const PhysicsCommandBuffer::SetColliderEnabled& command);
    void applyCommand(const PhysicsCommandBuffer::SetColliderTrigger& command);

    //=========================================
    // Scene command application
    //=========================================
    void applyCommand(const PhysicsCommandBuffer::SleepAllObjects& command);
    void applyCommand(const PhysicsCommandBuffer::AwakenAllObjects& command);

    //=========================================
    // Broadphase maintenance
    //=========================================
    void setBVHDirty(BodyHandle handle);

    //=========================================
    // Contact processing
    //=========================================
    void updateContactCache();
    void resolveCollisions();


//######################################################################################
//  Internal physics runtime: state
//######################################################################################

    //=========================================
    // Simulation state
    //=========================================
    physics::debug::StepPhase debugPhase = physics::debug::StepPhase::Ready;

    float dt = 0.0f;
    float savedPhysicsSurpassedTime = 0.0f;

    uint32_t contactsGeneratedThisFrame = 0;
    int pgsIterations = 8;

    //=========================================
    // External dependencies
    //=========================================
    FrameTimers* frameTimers = nullptr;
    std::vector<Tri> terrainTriangles;

    //=========================================
    // External command input
    //=========================================
    PhysicsCommandBuffer commandBuffer;

    //=========================================
    // World storage and runtime caches
    //=========================================
    PhysicsWorld physicsWorld;
    RuntimeCaches caches;

    //=========================================
    // Step-local working data
    //=========================================
    PairBatch pairBatch;
    ContactBatch contactBatch;

    //=========================================
    // Physics subsystems
    //=========================================
    BroadphaseManager broadphaseManager;
    NarrowphaseManager narrowphaseManager;
    PGSSolver pgsSolver;
    std::unique_ptr<CollisionManifold> collisionManifold;

    //=========================================
    // Sleep and wake queues
    //=========================================
    std::vector<BodyHandle> toWake;
    std::vector<BodyHandle> toSleep;

    //=========================================
    // Persistent contact data
    //=========================================
    std::unordered_map<size_t, Contact> contactCache;

    //=========================================
    // Debug data
    //=========================================
    std::vector<AABB> debugSweeps;
    std::vector<DebugSpeculativeContact> debugSpeculativeContacts;
};

}
