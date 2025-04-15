#pragma once

#include <glm/glm.hpp>
#include <unordered_map>
#include <random>

#include "aabb.h"
#include "sweep_and_prune.h"
#include "sat.h"
#include "collision_manifold.h"
#include "game_object.h"
#include "rotate_cube.h"

class PhysicsEngine {

public:
    void step(std::vector<GameObject>& GameObjectList, float deltaTime, bool showNormals, std::mt19937 rng);
    void clearPhysicsData();
    void addAabbEdges(const AABB& box);

    const std::unordered_map<size_t, Contact>& GetContactCache() const;

    int amountCollisionPairs = 0;

private:
    std::vector<Edge> allEdgesX;
    std::vector<Edge> allEdgesY;
    std::vector<Edge> allEdgesZ;

    std::unordered_map<size_t, Contact> contactCache;
};