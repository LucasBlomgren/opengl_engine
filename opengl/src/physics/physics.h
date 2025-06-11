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
    BVHTree<GameObject>& getStaticBvh();
    BVHTree<Tri>& getTerrainBvh();
    const std::unordered_map<size_t, Contact>& GetContactCache() const;


private:
    EngineState* engineState = nullptr;

    // --- objects ---
    std::vector<GameObject>* dynamicObjects;
    std::vector<GameObject> staticObjects;
    std::vector<Tri>* terrainTriangles;

    // --- bvh trees ---
    BVHTree<GameObject> dynamicBvh;
    BVHTree<GameObject> staticBvh;
    BVHTree<Tri> terrainBvh; 

    // --- update functions ---
    void updatePositions(float deltaTime);
    void updateSleepThresholds(GameObject& obj);
    void updateContactCache();

    // --- collision detection ---
    struct TerrainHits {
        GameObject* obj;
        std::vector<Tri*> tris;
    };

    CollisionManifold* collisionManifold;
    std::unordered_map<size_t, Contact> contactCache;

    void detectCollisions(std::pair<GameObject*, GameObject*>* dHits, int& dCount);
    void broadPhase(std::vector<TerrainHits>& terrainHits, std::pair<GameObject*, GameObject*>* dynamicHits, int& dynamicCount);
    void midPhase();
    void narrowPhase();

    // --- collision resolution ---
    void resolveCollisions();
};