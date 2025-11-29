#pragma once

#include <random>

#include "engine_state.h"
#include "collision_manifold.h"
#include "raycast.h"
#include "game_object.h"
#include "bvh.h"

#include "tri.h"

class PhysicsEngine {
public:
    //------------------------
    //     Main functions
    //------------------------
    void init(EngineState* engineState);
    void setupScene(std::vector<GameObject>* gameObjectList, std::vector<Tri>* terrainTriangles);
    void clearPhysicsData();
    void step(float deltaTime, std::mt19937 rng);

    void sleepAllObjects();
    void awakenAllObjects();

    void queueAdd(GameObject* obj);
    void queueRemove(GameObject* obj);

    RaycastHit performRaycast(Ray& ray);

    //------------------------
    //        Getters
    //------------------------
    BVHTree<GameObject>& getDynamicAwakeBvh();
    BVHTree<GameObject>& getDynamicAsleepBvh();
    BVHTree<GameObject>& getStaticBvh();
    BVHTree<Tri>& getTerrainBvh();
    const std::unordered_map<size_t, Contact>& GetContactCache() const;

private:
    float dt;
    EngineState* engineState = nullptr;

    //------------------------
    //      Add/Remove
    //------------------------
    struct PhysCmd { 
        enum Type { Add, Remove } type; 
        GameObject* obj; 
    };
    std::vector<PhysCmd> pending;
    void insertPendingObjects();
    void moveToStatic(GameObject& obj);
    bool staticBvhDirty = false;    

    //------------------------
    //     Game objects
    //------------------------
    std::vector<GameObject>* dynamicObjects;
    std::vector<Tri>* terrainTriangles;

    std::vector<int> awake_DynamicObjects;   // indices in dynamicObjects
    std::vector<int> asleep_DynamicObjects;
    std::vector<int> static_Objects;         

    //------------------------
    //      BVH Trees
    //------------------------
    BVHTree<GameObject> dynamicAwakeBvh;
    BVHTree<GameObject> dynamicAsleepBvh;
    BVHTree<GameObject> staticBvh;
    BVHTree<Tri> terrainBvh;

    //------------------------
    //    Update functions
    //------------------------
    void updateStates();
    void updateContactCache();

    //------------------------
    //  Collision detection 
    //------------------------
    struct TerrainHit {
        GameObject* obj;
        std::vector<Tri*> tris;                                // original bvh query
    };

    struct DynamicHit {
        GameObject* A;
        GameObject* B;
    };

    void detectAndSolveCollisions();
    void broadPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits);
    void narrowPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits);
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
    void moveToAwake(GameObject& obj);
    void moveToAsleep(GameObject& obj);
    void decideSleep();
    void updateSleepThresholds(GameObject& obj);
    bool asleepBvhDirty = false;

    struct WakeUpInfo { bool A, B; };
    WakeUpInfo wakeUpCheck(GameObject& A, GameObject& B);
};