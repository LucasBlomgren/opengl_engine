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

class PhysicsEngine 
{
public:
    void setPointers(std::vector<GameObject>* gameObjectList, BVHTree* tree);
    void step(float deltaTime, bool showNormals, std::mt19937 rng);
    void clearPhysicsData();

    void updatePositions(float deltaTime);

    BVHTree* getBvhTree() const;
    const std::unordered_map<size_t, Contact>& GetContactCache() const;
    RaycastHit performRaycast(Ray& ray);

    int amountCollisionPairs = 0;

    EngineState* engineState = nullptr;

private:
    std::vector<GameObject>* gameObjectList;
    BVHTree* bvhTree;
    std::unordered_map<size_t, Contact> contactCache;
};