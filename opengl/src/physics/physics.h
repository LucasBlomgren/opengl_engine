#pragma once

#include <random>

#include "pointer_cache.h"
#include "physics_world.h"
#include "world.h"
#include "timer.h"
#include "engine_state.h"
#include "collision_manifold.h"
#include "raycast.h"
#include "game_object.h"
#include "bvh/bvh.h"
#include "bvh/bvh_terrain.h"
#include "bvh/treetree_query.h"
#include "broadphase/broadphase_manager.h"
#include "broadphase/broadphase_pairs.h"

#include "tri.h"

struct DebugData {
    size_t awake = 0;
    size_t asleep = 0;
    size_t Static = 0;
    size_t terrainTris = 0;
    size_t collisions = 0;
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

    void queueAdd(ColliderHandle& handle, BroadphaseBucket& target);
    void queueRemove(ColliderHandle& handle);
    void queueMove(ColliderHandle& handle, BroadphaseBucket& target);

    RaycastHit performRaycast(Ray& ray);

    //------------------------
    //        Getters
    //------------------------
    PhysicsWorld* getPhysicsWorld();
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
        ColliderHandle handle;
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
    void updateStates();
    void updateContactCache();

    //------------------------
    //  Collision detection 
    //------------------------
    BroadphaseManager broadphaseManager;

    // cache for handles to pointers during narrow phase and contact generation to avoid multiple gen-checks and lookups in the slot map
    PointerCache<GameObject, GameObjectHandle> gameObjectPtrCache;
    PointerCache<Collider, ColliderHandle> colliderPtrCache;
    PointerCache<RigidBody, RigidBodyHandle> bodyPtrCache;

    void detectAndSolveCollisions();
    void narrowPhase(const std::vector<TerrainPair>& tHits, const std::vector<DynamicPair>& dHits);
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
    void pushAwayPlayer(GameObject& player, bool playerIsA, glm::vec3& n, float d);

    //------------------------
    //       Sleeping
    //------------------------
    std::vector<RigidBody*> toWake;
    std::vector<RigidBody*> toSleep;
    void decideSleep();
    void updateSleepThresholds();

    struct WakeUpInfo { bool A, B; };
    WakeUpInfo wakeUpCheck(RigidBody& A, RigidBody& B);
};