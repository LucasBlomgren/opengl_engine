#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "commands/physics_external_command_buffer.h"

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
    // Rigid body creation and destruction
    //=========================================
    [[nodiscard]] RigidBodyHandle createRigidBody(
        const RigidBodyDesc& desc
    );
    bool destroyRigidBody(RigidBodyHandle body);

    //=========================================
    // Collider creation and destruction
    //=========================================
    [[nodiscard]] ColliderHandle createCollider(
        RigidBodyHandle body, 
        const ColliderDesc& desc
    );
    bool destroyCollider(ColliderHandle collider);

    //=========================================
    // Rigid body commands
    //=========================================
    bool applyLinearImpulse(
        RigidBodyHandle body, 
        const glm::vec3& impulse
    );
    bool setLinearVelocity(
        RigidBodyHandle body, 
        const glm::vec3& velocity
    );
    bool setAngularVelocity(
        RigidBodyHandle body, 
        const glm::vec3& velocity
    );
    bool setKinematicTarget(
        RigidBodyHandle body, 
        const PhysicsPose& target
    );
    bool setRigidBodyAwake(RigidBodyHandle body);
    bool setRigidBodyAsleep(RigidBodyHandle body);
    bool setRigidBodyType(RigidBodyHandle body, BodyType type);
    bool setRigidBodyMotionControl(
        RigidBodyHandle body, 
        MotionControl motionControl
    );

    //=========================================
    // Collider commands
    //=========================================
    bool setColliderLocalPose(
        ColliderHandle collider, 
        const PhysicsPose& localPose
    );
    bool setColliderEnabled(ColliderHandle collider, bool enabled);
    bool setColliderTrigger(ColliderHandle collider, bool isTrigger);

    //=========================================
    // State queries
    //=========================================
    std::optional<RigidBodyState> getRigidBodyState(
        RigidBodyHandle body
    ) const;
    std::optional<ColliderState> getColliderState(
        ColliderHandle collider
    ) const;

    //=========================================
    // Physics queries
    //=========================================
    Raycast::RaycastHit raycast(Raycast::Ray& ray);

    //=========================================
    // Scene-wide commands
    //=========================================
    void sleepAllObjects();
    void awakenAllObjects();

    //=========================================
    // Simulation output
    //=========================================
    std::vector<ExternalMotionContact>& getExternalMotionContacts();

    //=========================================
    // Debug state
    //=========================================
    DebugData getDebugData() const;

    //=========================================
    // Debug visualization
    //=========================================
    void updateBVHRenderData(const BVHType& type, bool update);

    //=========================================
    // Debug spatial data
    //=========================================
    const std::vector<RigidBodyHandle>& getAwakeList() const;
    const BVHTree& getDynamicAwakeBvh() const;
    const BVHTree& getDynamicAsleepBvh() const;
    const BVHTree& getStaticBvh() const;
    const TerrainBVH& getTerrainBvh() const;
    const std::unordered_map<size_t, Contact>& getContactCache() const;

    //=========================================
    // Internal simulation phases
    //=========================================
    void beginPhysicsStep(float outerDt);
    void stepDiscrete(float deltaTime);
    void endPhysicsStep(float outerDt);

    //=========================================
    // Internal body integration
    //=========================================
    void integrateForcesAndVelocities(
        const std::vector<RigidBodyHandle>& bodies, 
        float dt
    );
    void integratePositionsAndColliders(
        const std::vector<RigidBodyHandle>& bodies, 
        float dt
    );
    void updateCollidersAndBodyAABB(
        RigidBody* body, 
        Transform* rootTransform
    );

    //=========================================
    // Internal sleep and wake processing
    //=========================================
    void processWakeList();
    void processSleepList(float outerDt);
    void decideSleep();
    void updateSleepThresholds();
    void addSleepDamping();

    //=========================================
    // Internal external-command processing
    //=========================================
    void flushExternalCommands();
    void processLifecycleCommands(
        const PhysicsExternalCommandBuffer::Batch& batch
    );
    void processMutationCommands(
        const std::vector<PhysicsExternalCommandBuffer::Mutation>& mutations
    );

    //=========================================
    // Internal broadphase processing
    //=========================================
    void setBVHDirty(RigidBodyHandle& handle);

    //=========================================
    // Internal contact processing
    //=========================================
    void updateContactCache();
    void resolveCollisions();

    //=========================================
    // Temporary legacy API
    //=========================================
    PhysicsWorld* getPhysicsWorld();
    void syncBodyFromTransform(RigidBodyHandle body);

    //=========================================
    // Simulation state
    //=========================================
    PhysicsStepDebugPhase debugPhase = PhysicsStepDebugPhase::Ready;

    float pausedDt = 0.0f;
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
    // External command buffer
    //=========================================
    PhysicsExternalCommandBuffer externalCommands;

    //=========================================
    // Physics world and runtime caches
    //=========================================
    PhysicsWorld physicsWorld;
    RuntimeCaches caches;

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
    // Contact and debug data
    //=========================================
    std::vector<AABB> debugSweeps;
    std::vector<DebugSpeculativeContact> debugSpeculativeContacts;
    std::unordered_map<size_t, Contact> contactCache;
};
