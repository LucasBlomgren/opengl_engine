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
#include "physics/bvh/bvh.h"
#include "physics/bvh/bvh_terrain.h"
#include "physics/narrowphase/collision_manifold.h"
#include "physics/narrowphase/narrowphase_manager.h"
#include "physics/raycast/raycast.h"
#include "physics/solver/pgs_solver.h"

struct PhysicsEngine::Impl {
public:
//######################################################################################
//   PhysicsEngine facade interface
//######################################################################################

    //=========================================
    // Initialization and scene lifetime
    //=========================================
    void init(World* world, FrameTimers* frameTimers);
    void setupScene(std::vector<Tri>* terrainTriangles);
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
    [[nodiscard]] RigidBodyHandle submitCreateRigidBody(const RigidBodyDesc& desc);
    bool submitDestroyRigidBody(RigidBodyHandle body);

    bool submitApplyLinearImpulse(RigidBodyHandle body, const glm::vec3& impulse);
    bool submitSetLinearVelocity(RigidBodyHandle body, const glm::vec3& velocity);
    bool submitSetAngularVelocity(RigidBodyHandle body, const glm::vec3& velocity);
    bool submitSetKinematicTarget(RigidBodyHandle body, const PhysicsPose& target);
    bool submitSetRigidBodySleepState(RigidBodyHandle body, bool asleep);
    bool submitSetRigidBodyType(RigidBodyHandle body, BodyType type);
    bool submitSetRigidBodyMotionControl(RigidBodyHandle body, MotionControl motionControl);

    //=========================================
    // Collider command submission
    //=========================================
    [[nodiscard]] ColliderHandle submitCreateCollider(RigidBodyHandle body, const ColliderDesc& desc);
    bool submitDestroyCollider(ColliderHandle collider);

    bool submitSetColliderLocalPose(ColliderHandle collider, const PhysicsPose& localPose);
    bool submitSetColliderEnabled(ColliderHandle collider, bool enabled);
    bool submitSetColliderTrigger(ColliderHandle collider, bool isTrigger);

    //=========================================
    // Scene command submission
    //=========================================
    void submitSleepAllObjects();
    void submitAwakenAllObjects();
    void submitSyncBodyFromTransform(RigidBodyHandle body);

    //=========================================
    // State queries
    //=========================================
    std::optional<RigidBodyState> getRigidBodyState(RigidBodyHandle body) const;
    std::optional<ColliderState> getColliderState(ColliderHandle collider) const;

    //=========================================
    // Spatial queries
    //=========================================
    Raycast::RaycastHit raycast(Raycast::Ray& ray);

    //=========================================
    // Simulation output
    //=========================================
    std::vector<ExternalMotionContact>& getExternalMotionContacts();

    //=========================================
    // Debug queries
    //=========================================
    DebugData getDebugData() const;
    PhysicsStepDebugPhase getDebugPhase() const;
    const std::vector<AABB>& getDebugSweeps() const;
    const std::vector<DebugSpeculativeContact>&
        getDebugSpeculativeContacts() const;
    const std::vector<RigidBodyHandle>& getAwakeList() const;
    const BVHTree& getDynamicAwakeBvh() const;
    const BVHTree& getDynamicAsleepBvh() const;
    const BVHTree& getStaticBvh() const;
    const TerrainBVH& getTerrainBvh() const;
    const std::unordered_map<size_t, Contact>& getContactCache() const;

    //=========================================
    // Debug visualization
    //=========================================
    void updateBVHRenderData(const BVHType& type, bool update);

    //=========================================
    // Temporary legacy access
    //=========================================
    PhysicsWorld* getPhysicsWorld();


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
    void integrateForcesAndVelocities(const std::vector<RigidBodyHandle>& bodies, float dt);
    void integratePositionsAndColliders(const std::vector<RigidBodyHandle>& bodies, float dt);
    void updateCollidersAndBodyAABB(RigidBody* body, Transform* rootTransform);

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
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodySleepState& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyType& command);
    void applyCommand(const PhysicsCommandBuffer::SetRigidBodyMotionControl& command);

    //=========================================
    // Collider command application
    //=========================================
    void applyCommand(const PhysicsCommandBuffer::SetColliderLocalPose& command);
    void applyCommand(const PhysicsCommandBuffer::SetColliderEnabled& command);
    void applyCommand(const PhysicsCommandBuffer::SetColliderTrigger& command);

    //=========================================
    // Scene command application
    //=========================================
    void applyCommand(const PhysicsCommandBuffer::SleepAllObjects& command);
    void applyCommand(const PhysicsCommandBuffer::AwakenAllObjects& command);
    void applyCommand(const PhysicsCommandBuffer::SyncBodyFromTransform& command);

    //=========================================
    // Broadphase maintenance
    //=========================================
    void setBVHDirty(RigidBodyHandle handle);

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
    PhysicsStepDebugPhase debugPhase = PhysicsStepDebugPhase::Ready;

    float dt = 0.0f;
    float savedPhysicsSurpassedTime = 0.0f;

    uint32_t contactsGeneratedThisFrame = 0;
    int pgsIterations = 8;

    //=========================================
    // External dependencies
    //=========================================
    World* world = nullptr;
    FrameTimers* frameTimers = nullptr;
    std::vector<Tri>* terrainTriangles = nullptr;

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
    std::vector<RigidBodyHandle> toWake;
    std::vector<RigidBodyHandle> toSleep;

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
