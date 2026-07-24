#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "core/timer.h"


#include "physics/public/physics_engine.h"

#include "physics/world/physics_world.h"
#include "physics/world/runtime_caches.h"

#include "physics/colliders/tri.h"

#include "physics/broadphase/broadphase_manager.h"
#include "physics/broadphase/rigidbody_broadphase_types.h"

#include "physics/bvh/bvh.h"
#include "physics/bvh/bvh_terrain.h"

#include "physics/narrowphase/collision_manifold.h"
#include "physics/narrowphase/narrowphase_manager.h"

#include "physics/raycast/raycast.h"
#include "physics/solver/pgs_solver.h"

class PhysicsEngine::Impl {
public:
    void init(World* world, FrameTimers* frameTimers);
    void setupScene(std::vector<Tri>* terrainTriangles);
    void clear();

    void prepareStepLoop();
    void beginPhysicsStep(float outerDt);
    void step(float deltaTime, EngineState& engine);
    void stepDiscrete(float deltaTime);
    void endPhysicsStep(float outerDt);

    void processWakeList();
    void processSleepList(float outerDt);
    void decideSleep();
    void updateSleepThresholds();
    void addSleepDamping();

    void sleepAllObjects();
    void awakenAllObjects();

    void syncBodyFromTransform(RigidBodyHandle body);
    void integratePositionsAndColliders(const std::vector<RigidBodyHandle>& bodies, float dt);
    void integrateForcesAndVelocities(const std::vector<RigidBodyHandle>& bodies, float dt);
    void updateCollidersAndBodyAABB(RigidBody* body, Transform* rootTransform);

    void queueAdd(RigidBodyHandle& handle, BroadphaseBucket& target);
    void queueRemove(RigidBodyHandle& handle);
    void queueMove(RigidBodyHandle& handle, BroadphaseBucket& target);
    void flushBroadphaseCommands();

    void setBVHDirty(RigidBodyHandle& handle);

    void updateContactCache();
    void resolveCollisions();

    RaycastHit performRaycast(Ray& ray);
    void updateBVHRenderData(const BVHType& type, bool update);

    DebugData getDebugData() const;

    std::vector<ExternalMotionContact>& getExternalMotionContacts();

    const std::vector<RigidBodyHandle>& getAwakeList() const;
    const BVHTree& getDynamicAwakeBvh() const;
    const BVHTree& getDynamicAsleepBvh() const;
    const BVHTree& getStaticBvh() const;
    const TerrainBVH& getTerrainBvh() const;
    const std::unordered_map<size_t, Contact>& getContactCache() const;

    PhysicsWorld* getPhysicsWorld();

    PhysicsStepDebugPhase debugPhase = PhysicsStepDebugPhase::Ready;

    float pausedDt = 0.0f;
    float dt = 0.0f;

    int pgsIterations = 8;

    PhysicsWorld physicsWorld;

    World* world = nullptr;
    FrameTimers* frameTimers = nullptr;

    uint32_t contactsGeneratedThisFrame = 0;
    float savedPhysicsSurpassedTime = 0.0f;

    struct PhysCmd {
        enum class Type {
            Add,
            Remove,
            Move
        };

        Type type;
        RigidBodyHandle handle;
        BroadphaseBucket dst = BroadphaseBucket::None;
    };

    std::vector<PhysCmd> pending;

    std::vector<Tri>* terrainTriangles = nullptr;

    BroadphaseManager broadphaseManager;
    NarrowphaseManager narrowphaseManager;
    PGSSolver pgsSolver;

    RuntimeCaches caches;

    std::unique_ptr<CollisionManifold> collisionManifold;

    std::vector<RigidBodyHandle> toWake;
    std::vector<RigidBodyHandle> toSleep;

    std::vector<AABB> debugSweeps;
    std::vector<DebugSpeculativeContact> debugSpeculativeContacts;
    std::unordered_map<size_t, Contact> contactCache;
};