#pragma once

#include <glm/glm.hpp>
#include <unordered_map>
#include <random>

#include "aabb.h"
#include "sweep_and_prune.h"
#include "sat.h"
#include "collision_manifold.h"
#include "ray_cast_finite.h"
#include "game_object.h"
#include "rotate_cube.h"
#include "ray.h"

class PhysicsEngine 
{
public:
    void setPointers(std::vector<GameObject>* gameObjectList);
    void step(float deltaTime, bool showNormals, std::mt19937 rng);
    void clearPhysicsData();
    void addAabbEdges(const AABB& box);

    void updatePositions(float deltaTime);

    const std::unordered_map<size_t, Contact>& GetContactCache() const;
    std::vector<Edge>* getSortedEdges() const;
    int getSelectedAxis() const;
    RaycastHit performRayCastFinite(Physics::Ray& ray);

    int amountCollisionPairs = 0;

private:
    std::vector<GameObject>* gameObjectList;
    std::vector<Edge> allEdgesX;
    std::vector<Edge> allEdgesY;
    std::vector<Edge> allEdgesZ;
    std::vector<Edge>* sortedEdges;
    int selectedAxis;

    std::unordered_map<size_t, Contact> contactCache;
};