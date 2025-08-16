#pragma once

#include <random>

#include "engine_state.h"
#include "collision_manifold.h"
#include "raycast.h"
#include "game_object.h"
#include "bvh.h"

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
    RaycastHit performRaycast(Ray& ray);

    //------------------------
    //        Getters
    //------------------------
    BVHTree<GameObject>& getDynamicBvh();
    BVHTree<Tri>& getTerrainBvh();
    const std::unordered_map<size_t, Contact>& GetContactCache() const;

    //------------------------
    //          Misc
    //------------------------
    CollisionManifold* collisionManifold; // public for testing warmstarting

private:
    float dt;
    EngineState* engineState = nullptr;

    //------------------------
    //     Game objects
    //------------------------
    std::vector<GameObject>* dynamicObjects;
    std::vector<Tri>* terrainTriangles;

    //------------------------
    //      BVH Trees
    //------------------------
    BVHTree<GameObject> dynamicBvh;
    //BVHTree<GameObject> staticBvh;
    BVHTree<Tri> terrainBvh;

    //------------------------
    //    Update functions
    //------------------------
    void updatePositions();
    void updateSleepThresholds(GameObject& obj);
    void updateContactCache();

    //------------------------
    //  Collision detection 
    //------------------------
    struct TerrainHit {
        GameObject* obj;
        std::vector<Tri*> coarse;                                // original bvh query
        std::vector<std::pair<Tri*, std::vector<Tri*>>> refined; // refined trimesh vs coarse
    };

    struct DynamicHit {
        GameObject* A;
        GameObject* B;
        std::vector<Tri*> singleMeshTris;                   // one mesh             
        std::vector<std::pair<Tri*, Tri*>> doubleMeshTris;  // two meshes
    };

    void detectAndSolveCollisions();
    void broadPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits);
    void midPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits);
    void narrowPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits);
    void collectActiveContacts();

    //------------------------
    //   Collision Manifold
    //------------------------
    //CollisionManifold* collisionManifold;
    std::unordered_map<size_t, Contact> contactCache;
    std::vector<Contact*> contactsToSolve;

    //------------------------
    //  Collision Resolution
    //------------------------
    void resolveCollisions();
    bool updateSleep(GameObject& A, GameObject& B);
};