#pragma once

#include <random>

#include "runtime_caches.h"
#include "game/transform_utils.h"
#include "core/pointer_cache.h"
#include "physics_world.h"
#include "world.h"
#include "timer.h"
#include "engine_state.h"
#include "narrowphase/collision_manifold.h"
#include "raycast.h"
#include "game_object.h"
#include "bvh/bvh.h"
#include "bvh/bvh_terrain.h"
#include "bvh/treetree_query.h"
#include "broadphase/broadphase_manager.h"
#include "broadphase/broadphase_pairs.h"
#include "narrowphase/narrowphase_manager.h"

#include "tri.h"

struct DebugData {
    size_t awake = 0;
    size_t asleep = 0;
    size_t Static = 0;
    size_t colliders = 0;
    size_t terrainTris = 0;
    size_t contacts = 0;
};

class PhysicsEngine {
public:
    void init(World* world, FrameTimers* ft);

    //------------------------
    //     Main functions
    //------------------------
    void setupScene(std::vector<Tri>* terrainTriangles);
    void clearPhysicsData();
    void step(float deltaTime, std::mt19937 rng);

    void sleepAllObjects();
    void awakenAllObjects();

    void queueAdd(RigidBodyHandle& handle, BroadphaseBucket& target);
    void queueRemove(RigidBodyHandle& handle);
    void queueMove(RigidBodyHandle& handle, BroadphaseBucket& target);

    void setBVHDirty(RigidBodyHandle& handle);
    RaycastHit performRaycast(Ray& ray);

    // #TODO: fix better public API 
    //------------------------
    //        Getters
    //------------------------
    PhysicsWorld* getPhysicsWorld();
    std::vector<ExternalMotionContact>& getExternalMotionContacts();
    const DebugData getDebugData();
    const BVHTree& getDynamicAwakeBvh() const;
    const BVHTree& getDynamicAsleepBvh() const;
    const BVHTree& getStaticBvh() const;
    const TerrainBVH& getTerrainBvh() const;
    const std::unordered_map<size_t, Contact>& GetContactCache() const;

    std::vector<Contact*> contactsToSolve;

private:
    float dt;
    PhysicsWorld physicsWorld;
    World* world;

    FrameTimers* frameTimers;

    //-----------------------------
    // Debug visualization data
    //-----------------------------
    DebugData debugData;


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
    void updateBodiesAndColliders();
    void updateContactCache();

    //------------------------
    //  Collision detection 
    //------------------------
    BroadphaseManager broadphaseManager;
    NarrowphaseManager narrowphaseManager;

    // caches for handles to pointers during narrow phase and contact generation to avoid multiple gen-checks and lookups in the slot map
    RuntimeCaches caches;

    void detectAndSolveCollisions();
    void collectActiveContacts();

    //------------------------
    //   Collision Manifold
    //------------------------
    CollisionManifold* collisionManifold;
    std::unordered_map<size_t, Contact> contactCache;

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
};