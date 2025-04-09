#pragma once

#include <glm/glm.hpp>
#include <unordered_map>
#include <random>

#include "AABB.h"
#include "sweepandprune.h"
#include "SAT.h"
#include "collisionManifold.h"
#include "GameObject.h"
#include "rotateCube.h"

class PhysicsEngine {

public:
    void step(GLFWwindow* window, std::vector<GameObject>& meshList, float deltaTime, bool showNormals, std::mt19937 rng);
    void clearPhysicsData();
    void AddAABBEdges(const AABB& box);

    const std::unordered_map<size_t, Contact>& GetContactCache() const;

private:
    std::vector<Edge> allEdgesX;
    std::vector<Edge> allEdgesY;
    std::vector<Edge> allEdgesZ;

    std::unordered_map<size_t, Contact> contactCache;
};