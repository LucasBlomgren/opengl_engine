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

enum class TerrainManifoldType {
    BoxMesh,
    SphereMesh
};

struct DynamicContactCandidate {
    SAT::Result sat{};
    DynamicManifoldType manifoldType = DynamicManifoldType::None;
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

struct PendingSpeculativeContact {
    ContactBuildInput input;
    DynamicContactCandidate candidate;
    RigidBodyHandle sweepOwner;
};

struct TerrainContactCandidate {
    std::vector<SAT::Result> results;
    glm::vec3 normal{ 0.0f };
    TerrainManifoldType manifoldType;
};

struct SpeculativeContact {
    RigidBodyHandle bodyHandleA;
    RigidBodyHandle bodyHandleB;

    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    glm::vec3 normal{ 0.0f };
    float separation = 0.0f;
    float toi = 0.0f;

    bool noSolverResponseA = false;
    bool noSolverResponseB = false;
    bool contributesMotionA = false;
    bool contributesMotionB = false;
};

struct ContactBatch {
    std::vector<Contact*> contacts;
    std::vector<SpeculativeContact> speculativeContacts;

    void clear() {
        contacts.clear();
        speculativeContacts.clear();
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

struct PairKey {
    uint64_t a;
    uint64_t b;

    bool operator==(const PairKey& other) const {
        return a == other.a && b == other.b;
    }
};

struct PairKeyHash {
    size_t operator()(const PairKey& k) const {
        uint64_t x = k.a ^ (k.b + 0x9e3779b97f4a7c15ull + (k.a << 6) + (k.a >> 2));
        return std::hash<uint64_t>{}(x);
    }
};  

struct DebugSpeculativeContact {
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
    glm::vec3 worldPos{ 0.0f };
    // add other debug information as needed
};