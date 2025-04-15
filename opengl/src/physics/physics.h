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

    void updatePositions(std::vector<GameObject>& GameObjectList, float deltaTime);

    const std::unordered_map<size_t, Contact>& GetContactCache() const;
    std::vector<Edge>* getSortedEdges() const;
    int getSelectedAxis() const;

    int amountCollisionPairs = 0;

private:
    std::vector<Edge> allEdgesX;
    std::vector<Edge> allEdgesY;
    std::vector<Edge> allEdgesZ;
    std::vector<Edge>* sortedEdges;
    int selectedAxis;

    std::unordered_map<size_t, Contact> contactCache;
};