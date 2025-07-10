#pragma once

#include "engine_state.h"

#include <glm/glm.hpp>
#include <unordered_map>
#include <random>

#include "aabb.h"
#include "sat.h"
#include "collision_manifold.h"
#include "raycast.h"
#include "game_object.h"
#include "bvh.h"

class PhysicsEngine {
public:
    // --- main functions ---
    void init(EngineState* engineState);
    void setupScene(std::vector<GameObject>* gameObjectList, std::vector<Tri>* terrainTriangles);
    void clearPhysicsData();
    void step(float deltaTime, std::mt19937 rng);
    RaycastHit performRaycast(Ray& ray);

    // --- getters ---
    BVHTree<GameObject>& getDynamicBvh();
    //BVHTree<GameObject>& getStaticBvh();
    BVHTree<Tri>& getTerrainBvh();
    const std::unordered_map<size_t, Contact>& GetContactCache() const;


private:
    float dt;
    EngineState* engineState = nullptr;

    // --- objects ---
    std::vector<GameObject>* dynamicObjects;
    //std::vector<GameObject> staticObjects;
    std::vector<Tri>* terrainTriangles;

    // --- bvh trees ---
    BVHTree<GameObject> dynamicBvh;
    //BVHTree<GameObject> staticBvh;
    BVHTree<Tri> terrainBvh; 

    // --- update functions ---
    void updatePositions();
    void updateSleepThresholds(GameObject& obj);
    void updateContactCache();

    // --- collision detection ---
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

    CollisionManifold* collisionManifold;
    std::unordered_map<size_t, Contact> contactCache;

    void detectAndSolveCollisions();
    void broadPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits);
    void midPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits); 
    void narrowPhase(std::vector<TerrainHit>& tHits, std::vector<DynamicHit>& dHits);

    // --- collision resolution ---
    void resolveCollision(Contact& contact);
    bool updateSleep(GameObject& A, GameObject& B);
    void updateHelpers(GameObject& obj);
};