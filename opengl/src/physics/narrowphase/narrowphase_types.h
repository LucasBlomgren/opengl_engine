#pragma once

#include <vector>
#include "collision_manifold.h"

enum class NarrowphasePass {
    Normal,
    Speculative
};

enum class DynamicManifoldType {
    None,
    BoxBox,
    BoxSphere,
    SphereSphere,
    TerrainBox,
    TerrainSphere
};

struct DynamicContactCandidate {
    SAT::Result sat{};
    DynamicManifoldType manifoldType = DynamicManifoldType::None;

    bool speculative = false;
    float separation = 0.0f;
    float toi = 0.0f;
};

enum class TerrainManifoldType {
    BoxMesh,
    SphereMesh
};

struct TerrainContactCandidate {
    std::vector<SAT::Result> results;
    glm::vec3 normal{ 0.0f };
    TerrainManifoldType manifoldType;
};

struct ContactBuildInput {
    RigidBodyHandle bodyHandleA;
    RigidBodyHandle bodyHandleB;

    ColliderHandle colliderHandleA;
    ColliderHandle colliderHandleB;

    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    Collider* colliderA = nullptr;
    Collider* colliderB = nullptr;

    void swapAB()
    {
        std::swap(bodyHandleA, bodyHandleB);
        std::swap(colliderHandleA, colliderHandleB);
        std::swap(bodyA, bodyB);
        std::swap(colliderA, colliderB);
    }
};

struct ContactBatch {
    std::vector<Contact*> contacts;

    void clear() {
        contacts.clear();
    }
    size_t size() {
        return contacts.size();
    }
    void sortByMinY() {
        std::sort(contacts.begin(), contacts.end(),
            [](const Contact* a, const Contact* b) {
                if (a->minY < b->minY) return true;
                if (b->minY < a->minY) return false;
                return a->hashKey < b->hashKey;
            });
    }
};

struct ExternalMotionContact {
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
    glm::vec3 normal{ 0.0f };
    float penetration = 0.0f;

    ExternalMotionContact(const RigidBodyHandle& bodyA,
        const RigidBodyHandle& bodyB,
        const glm::vec3& normal,
        float penetration)
        : bodyA(bodyA), bodyB(bodyB), normal(normal), penetration(penetration) {}
    ExternalMotionContact() = default;
};