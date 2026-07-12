#pragma once

#include "substeps/physics_scope.h"
#include "substeps/build_islands_types.h"
#include "runtime_caches.h"
#include "physics_world.h"
#include "timer.h"
#include "narrowphase/collision_manifold.h"
#include "raycast.h"
#include "bvh/bvh.h"
#include "bvh/bvh_terrain.h"
#include "broadphase/broadphase_manager.h"
#include "narrowphase/narrowphase_manager.h"
#include "solver/pgs_solver.h"
#include "tri.h"

class EngineState;
class World;

struct DebugData {
    size_t awake = 0;
    size_t asleep = 0;
    size_t Static = 0;
    size_t colliders = 0;
    size_t terrainTris = 0;
    size_t contacts = 0;
    size_t currentSubstepAmount = 0;
};

enum class StepMode {
    Global = 0,
    Island = 1
};

class PhysicsEngine {
public:
    void init(World* world, FrameTimers* ft);

    StepMode stepMode = StepMode::Island;

    std::vector<PhysicsScope> islandScopes;
    PhysicsScope restScope;
    std::vector<uint8_t> isIslandBody;
    MotionRisk computeMotionRisk(RigidBodyHandle h, float frameDt);
    void createPredictedIslandsMVP(float frameDt);
    void stepIslandModeMVP(float frameDt);
    void buildPairsForScope(PhysicsScope& scope, PairBatch& pairs);

    //------------------------
    //     Main functions
    //------------------------
    void setupScene(std::vector<Tri>* terrainTriangles);
    void clear();
    void prepareStepLoop();
    int computeGlobalSubsteps(float dt);

    void beginPhysicsStep(float outerDt);
    void stepScope(PhysicsScope& scope, float dt);
    void step(float deltaTime, EngineState& engine);
    void stepDiscrete(float deltaTime);
    void endPhysicsStep(float outerDt);
    void processWakeList();
    void processSleepList(float outerDt);

    void sleepAllObjects();
    void awakenAllObjects();

    void queueAdd(RigidBodyHandle& handle, BroadphaseBucket& target);
    void queueRemove(RigidBodyHandle& handle);
    void queueMove(RigidBodyHandle& handle, BroadphaseBucket& target);

    void setBVHDirty(RigidBodyHandle& handle);
    void updateBVHRenderData(const BVHType& type, bool update);
    RaycastHit performRaycast(Ray& ray);

    // #TODO: fix better public API
    //------------------------
    //        Getters
    //------------------------
    PhysicsWorld* getPhysicsWorld();
    const DebugData getDebugData();
    std::vector<ExternalMotionContact>& getExternalMotionContacts();
    const std::vector<RigidBodyHandle>& getAwakeList() const;
    const BVHTree& getDynamicAwakeBvh() const;
    const BVHTree& getDynamicAsleepBvh() const;
    const BVHTree& getStaticBvh() const;
    const TerrainBVH& getTerrainBvh() const;
    const std::unordered_map<size_t, Contact>& GetContactCache() const;

    int maxSubsteps = 16;
    int pgsIterations = 8;

    std::vector<AABB> debugSweeps; // public for debug rendering
    std::unordered_map<size_t, Contact> contactCache; // public for debug rendering

private:
    float dt;
    PhysicsWorld physicsWorld;
    World* world = nullptr;
    FrameTimers* frameTimers;

    int currentSubstepAmount = 1;
    bool physicsFrameActive = false;
    int schedulerSubstep = 0;
    int highestSubstepIslandCount = 0;

    uint32_t contactsGeneratedThisFrame = 0;

    //-----------------------------
    //  Broadphase Add/Remove/Move
    //-----------------------------
    struct PhysCmd {
        enum class Type { Add, Remove, Move } type;
        RigidBodyHandle handle;
        BroadphaseBucket dst = BroadphaseBucket::None;
    };
    std::vector<PhysCmd> pending;
    void flushBroadphaseCommands();

    //------------------------
    //     Terrain
    //------------------------
    std::vector<Tri>* terrainTriangles;

    //------------------------
    //    Update functions
    //------------------------
    void updateBodiesAndColliders(const std::vector<RigidBodyHandle>& bodies, float dt);
    void updateContactCache();

    //------------------------
    //  Collision handling
    //------------------------
    BroadphaseManager broadphaseManager;
    NarrowphaseManager narrowphaseManager;
    PGSSolver pgsSolver;

    // caches for handles to pointers during narrow phase and contact generation to avoid multiple gen-checks and lookups in the slot map
    RuntimeCaches caches;

    //------------------------
    //   Collision Manifold
    //------------------------
    CollisionManifold* collisionManifold;

    //------------------------
    //  Collision Resolution
    //------------------------
    void resolveCollisions();

    //------------------------
    //       Sleeping
    //------------------------
    std::vector<RigidBodyHandle> toWake;
    std::vector<RigidBodyHandle> toSleep;
    void decideSleep();
    void updateSleepThresholds();
    void addSleepDamping();
};