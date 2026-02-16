#pragma once
  
#include "slot_map.h"
#include "bvh/bvh.h"
#include "game_object.h"

struct Ray {
    float length;
    glm::vec3 direction;
    glm::vec3 start;
    glm::vec3 end;

    Ray(const glm::vec3 start, const glm::vec3 direction, float length)
        : start(start),
        direction(direction),
        length(length)
    {
        end = start + direction * length;
    }
};

struct RaycastHit {
    bool hit = false;
    GameObjectHandle objectHandle;
    glm::vec3 point;  
    glm::vec3 normal;
    float t;    
};

RaycastHit raycast(Ray& ray, const BVHTree& tree, SlotMap<GameObject, GameObjectHandle>* slotmap);