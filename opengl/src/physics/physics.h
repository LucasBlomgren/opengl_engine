#pragma once

#include <random>

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

    void queueAdd(GameObjectHandle& handle, BroadphaseBucket& target);
    void queueRemove(GameObjectHandle& handle);
    void queueMove(GameObjectHandle& handle, BroadphaseBucket& target);

    RaycastHit performRaycast(Ray& ray);

    //------------------------
    //        Getters
    //------------------------
    const DebugData getDebugData();
    const BVHTree& getDynamicAwakeBvh();
    const BVHTree& getDynamicAsleepBvh();
    const BVHTree& getStaticBvh();
    const TerrainBVH& getTerrainBvh();
    const std::unordered_map<size_t, Contact>& GetContactCache() const;

    std::vector<Contact*> contactsToSolve;

private:
    float dt;
    FrameTimers* frameTimers;
    World* world;
    DebugData debugData;

    //-----------------------------
    //  Broadphase Add/Remove/Move
    //-----------------------------
    struct PhysCmd {
        enum class Type { Add, Remove, Move } type;
        GameObjectHandle handle;
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
    struct StepPtrCache {
        std::vector<GameObject*> cache;
        SlotMap<GameObject, GameObjectHandle>* sm;

        void init(SlotMap<GameObject, GameObjectHandle>& slotMap) {
            sm = &slotMap;
        }

        void clear() {
            cache.assign(sm->slot_capacity(), nullptr);
        }

        GameObject* get(GameObjectHandle h) {
            // 1) Hämta vad cachen tror
            GameObject*& slot = cache[h.slot];

            // 2) Hämta "sanningen" från slotmap just nu
            GameObject* real = sm->try_get(h);  // gen-check + aktuell adress

            // Om handle är invalid -> crasha tidigt, eller returnera nullptr
            if (!real) {
                // Om du *vill* kunna cache:a nullptr, hantera separat.
                __debugbreak();
                return nullptr;
            }

            // 3) Om vi redan har cacheat en pekare, verifiera att den fortfarande matchar
            if (slot && slot != real) {
                // Här har dense flyttat, eller din cache-slot pekar på gammalt minne
                __debugbreak();
            }

            // 4) Fyll cachen om tom
            if (!slot) slot = real;

            return slot;
        }
    };
    StepPtrCache stepPtrCache;

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
    std::vector<GameObjectHandle> toWake;
    std::vector<GameObjectHandle> toSleep;
    void decideSleep();
    void updateSleepThresholds();

    struct WakeUpInfo { bool A, B; };
    WakeUpInfo wakeUpCheck(const GameObjectHandle& handleA, const GameObjectHandle& handleB, GameObject& objA, GameObject& objB);
};