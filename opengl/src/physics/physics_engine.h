#pragma once
#include "runtime_caches.h"
#include "physics_world.h"
#include "timer.h"
#include "narrowphase/collision_manifold.h"
#include "raycast/raycast.h"
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

class PhysicsEngine {
public:
    void init(World* world, FrameTimers* ft);

    //------------------------
    //     Main functions
    //------------------------
    void setupScene(std::vector<Tri>* terrainTriangles);
    void clear();
    void prepareStepLoop();
    int computeGlobalSubsteps(float dt);

    void beginPhysicsStep(float outerDt);
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

    int pgsIterations = 8;

    std::vector<AABB> debugSweeps; // public for debug rendering
    std::unordered_map<size_t, Contact> contactCache; // public for debug rendering

private:
    float dt;
    PhysicsWorld physicsWorld;
    World* world = nullptr;
    FrameTimers* frameTimers;

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