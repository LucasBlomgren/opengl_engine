#pragma once

#include <random>

#include "timer.h"
#include "engine_state.h"
#include "collision_manifold.h"
#include "raycast.h"
#include "game_object.h"
#include "bvh.h"
#include "broadphase_manager.h"
#include "broadphase_pairs.h"

#include "tri.h"

class PhysicsEngine {
public:
    PhysicsEngine(FrameTimers* ft) { 
        this->collisionManifold = new CollisionManifold(); 
        this->frameTimers = ft;
    }

    //------------------------
    //     Main functions
    //------------------------
    void setupScene(std::vector<GameObject>* gameObjectList, std::vector<Tri>* terrainTriangles);
    void clearPhysicsData();
    void step(float deltaTime, std::mt19937 rng);

    void sleepAllObjects();
    void awakenAllObjects();

    void queueAdd(GameObject* obj, BroadphaseBucket& target);
    void queueRemove(GameObject* obj);
    void queueMove(GameObject* obj, BroadphaseBucket& target);

    RaycastHit performRaycast(Ray& ray);

    //------------------------
    //        Getters
    //------------------------
    const BVHTree<GameObject>& getDynamicAwakeBvh();
    const BVHTree<GameObject>& getDynamicAsleepBvh();
    const BVHTree<GameObject>& getStaticBvh();
    const BVHTree<Tri>& getTerrainBvh();
    const std::unordered_map<size_t, Contact>& GetContactCache() const;

private:
    float dt;
    FrameTimers* frameTimers;

    //-----------------------------
    //  Broadphase Add/Remove/Move
    //-----------------------------
    struct PhysCmd {
        enum class Type { Add, Remove, Move } type;
        GameObject* obj = nullptr;
        BroadphaseBucket dst = BroadphaseBucket::None;
    };
    std::vector<PhysCmd> pending;
    void flushBroadphaseCommands();

    //------------------------
    //     Game objects
    //------------------------
    std::vector<GameObject>* dynamicObjects;
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

    void detectAndSolveCollisions();
    void narrowPhase(const std::vector<TerrainPair>& tHits, const std::vector<DynamicPair>& dHits);
    void collectActiveContacts();

    //------------------------
    //   Collision Manifold
    //------------------------
    CollisionManifold* collisionManifold;
    std::unordered_map<size_t, Contact> contactCache;
    std::vector<Contact*> contactsToSolve;

    //------------------------
    //  Collision Resolution
    //------------------------
    void resolveCollisions();
    void pushAwayPlayer(GameObject& player, bool playerIsA, glm::vec3& n, float d);

    //------------------------
    //       Sleeping
    //------------------------
    std::vector<int> toWake;
    std::vector<int> toSleep;
    void decideSleep();
    void updateSleepThresholds();

    struct WakeUpInfo { bool A, B; };
    WakeUpInfo wakeUpCheck(GameObject& A, GameObject& B);
};