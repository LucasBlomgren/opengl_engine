#pragma once

#include <cstdint>
#include <utility>
#include <variant>
#include <vector>

#include "core/slot_map.h"
#include "physics/public/contact_types.h"
#include "SAT/sat.h"

namespace physics::internal {

// forward declarations
class RigidBody;
struct Collider;
struct Contact;


enum class ShapePairKind {
    BoxBox,
    BoxSphere,
    SphereSphere
};

enum class ContactPartnerType {
    RigidBody,
    Terrain
};

//======================================================
// Resolved rigid-rigid narrowphase input
//======================================================
struct ColliderEndpointRef {
    BodyHandle bodyHandle;
    ColliderHandle colliderHandle;
    RigidBody* body = nullptr;
    Collider* collider = nullptr;
};

struct ResolvedColliderPair {
    ColliderEndpointRef a;
    ColliderEndpointRef b;
    ShapePairKind shapePair;

    void swapAB() {
        std::swap(a, b);
    }
};

//======================================================
// Successful narrowphase test results
//======================================================
struct OverlapHit {
    ResolvedColliderPair pair;
    SAT::Result geometry;
};

struct SweepHit {
    ResolvedColliderPair pair;
    SAT::Result geometry;
    BodyHandle sweepOwner;
};

// Temporary bridge for the existing terrain path. This becomes a
// SurfaceSweepHit once terrain and triangle meshes share a surface endpoint.
struct TerrainSweepHit {
    ColliderEndpointRef collider;
    SAT::Result geometry;
    BodyHandle sweepOwner;
};

using PendingSweepHit = std::variant<SweepHit, TerrainSweepHit>;

//=============================================================
// SpeculativeContact
// output of the narrowphase, used to build solver constraints
//=============================================================
struct SpeculativeContact {
    ContactPartnerType partnerTypeA = ContactPartnerType::RigidBody;
    ContactPartnerType partnerTypeB = ContactPartnerType::RigidBody;

    BodyHandle bodyHandleA;
    BodyHandle bodyHandleB;

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

//======================================================
// ContactBatch
// output of the narrowphase, contains all contacts and 
// speculative contacts for the current frame
//======================================================
struct ContactBatch {
    std::vector<Contact*> contacts;
    std::vector<SpeculativeContact> speculativeContacts;

    void clear() {
        contacts.clear();
        speculativeContacts.clear();
    }

    size_t size() const {
        return contacts.size();
    }

    // narrowphase_types.cpp
    void sortByMinY();
};

//======================================================
// PairKey and PairKeyHash
// used to store pairs of rigid bodies to avoid duplicate 
// contacts in the narrowphase
//======================================================
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

//======================================================
// DebugSpeculativeContact
// for rendering speculative contacts for debugging purposes
//======================================================
struct DebugSpeculativeContact {
    BodyHandle bodyA;
    BodyHandle bodyB;
    glm::vec3 worldPos{ 0.0f };
    // add other debug information as needed
};


}
