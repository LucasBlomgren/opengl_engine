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
    TEAPOT
};

using ColliderShape = std::variant<OOBB, Sphere, Mesh>;

struct Collider {
    GameObject* owner;
    ColliderShape shape;

    Collider(GameObject* o) 
        : owner(o) {}

    AABB& getAABB() const;
};




//struct TriID { uint32_t mesh, tri; }; // eller const Tri* ptr
//
//struct Collider {
//    enum class Type : uint8_t { None, Sphere, OOBB, Mesh, Tri } type{ Type::None };
//
//    union {
//        Sphere sphere;  // liten POD
//        OOBB   oobb;    // liten POD
//        Mesh* mesh;    // handle
//        TriID  tri;     // handle
//    };
//
//    AABB getAABB() const {
//        switch (type) {
//        case Type::Sphere: return sphere.worldAABB();
//        case Type::OOBB:   return oobb.worldAABB();
//        case Type::Mesh:   return mesh->worldAABB();
//        case Type::Tri:    return triWorldAABB(tri); // från tri-pool
//        default:           return {};
//        }
//    }
//};
//static_assert(sizeof(Collider) <= 64, "håll den ~en cache line");