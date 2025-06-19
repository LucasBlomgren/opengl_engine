#pragma once

#include <variant>

#include "aabb.h"
#include "oobb.h"
#include "sphere.h"
#include "tri_mesh.h"

class GameObject;
class TriMesh;

enum class ColliderType {
    CUBOID,
    SPHERE,
    MESH,
};

using ColliderShape = std::variant<OOBB, Sphere, TriMesh>;

struct Collider {
    GameObject* owner;
    ColliderShape shape;

    Collider(GameObject* o) 
        : owner(o) {}

    AABB& getAABB() const;
};