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
#include "rotate_cube.h"
#include "bvh.h"

class PhysicsEngine {
public:
    void init(std::vector<GameObject>* gameObjectList, EngineState* engineState, BVHTree* tree);
    void step(float deltaTime, std::mt19937 rng);
    void clearPhysicsData();

    BVHTree* getBvhTree() const;
    const std::unordered_map<size_t, Contact>& GetContactCache() const;
    RaycastHit performRaycast(Ray& ray);

private:
    EngineState* engineState = nullptr;
    std::vector<GameObject>* gameObjectList;
    BVHTree* bvhTree;
    CollisionManifold* collisionManifold;
    std::unordered_map<size_t, Contact> contactCache;

    void updatePositions(float deltaTime);
    void updateSleepThresholds(GameObject& obj);
    void updateContactCache();
};