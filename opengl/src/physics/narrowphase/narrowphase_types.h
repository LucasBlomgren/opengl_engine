#pragma once

#include <vector>
#include <cstdint>

#include "core/slot_map.h"
#include "physics/public/contact_types.h"
#include "SAT/sat.h"

namespace physics::internal {

// forward declarations
class RigidBody;
struct Collider;
struct Contact;


enum class ManifoldType {
    None,
    BoxBox,
    BoxSphere,
    SphereSphere,
    SphereTriangle,
    BoxTriangle,
};

enum class ContactPartnerType {
    RigidBody,
    Terrain
};

//======================================================
//  Contact Build Input 
//  used to build a contact from SAT results
//======================================================
struct ContactBuildInput {
    BodyHandle bodyHandleA;
    BodyHandle bodyHandleB;

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

//=============================================================
// Contact Candidates
// used to store SAT results and manifold type before building a contact
//=============================================================
struct DynamicContactCandidate {
    ContactPartnerType partnerTypeA = ContactPartnerType::RigidBody;
    ContactPartnerType partnerTypeB = ContactPartnerType::RigidBody;
    ManifoldType manifoldType = ManifoldType::None;
    SAT::Result sat{};
};

struct TerrainContactCandidate {
    std::vector<SAT::Result> results;
    glm::vec3 normal{ 0.0f };
    ManifoldType manifoldType = ManifoldType::None;
};

struct PendingSpeculativeContact {
    ContactBuildInput input;
    DynamicContactCandidate candidate;
    BodyHandle sweepOwner;
};

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
