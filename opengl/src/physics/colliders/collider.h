#pragma once

#include <variant>

#include "aabb.h"
#include "oobb.h"
#include "sphere.h"
#include "mesh.h"

class GameObject;
class TriMesh;

enum class ColliderType {
    CUBOID,
    SPHERE,
    MESH,
};

using ColliderShape = std::variant<OOBB, Sphere, Mesh>;

struct Collider {
    GameObject* owner;
    ColliderShape shape;

    Collider(GameObject* o) 
        : owner(o) {}

    AABB& getAABB() const;
};