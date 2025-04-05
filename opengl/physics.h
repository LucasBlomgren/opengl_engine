#pragma once

#include <glm/glm.hpp>
#include <unordered_map>
#include <random>

#include "sweepandprune.h"
#include "SAT.h"
#include "collisionManifold.h"
#include "AABB.h"
#include "mesh.h"
#include "rotateCube.h"

class PhysicsEngine {

public:
    std::unordered_map<size_t, Contact> contactCache;

    std::vector<Edge> allEdgesX;
    std::vector<Edge> allEdgesY;
    std::vector<Edge> allEdgesZ;

    void step(GLFWwindow* window, std::vector<Mesh>& meshList, float deltaTime, bool showNormals, std::mt19937 rng);
    void clearPhysicsData();
};