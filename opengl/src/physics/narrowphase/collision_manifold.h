#pragma once

#include <unordered_map>

#include "runtime_caches.h"
#include "rigid_body.h"
#include "sat.h"
#include "collider_pose.h"

// for Sutherland-Hodgman clipping in box-box and box-mesh contact point generation
struct Plane {
    glm::vec3 normal{ 0.0f };
    glm::vec3 point{ 0.0f };
};

struct ContactPoint {
    glm::vec3 worldPos{ 0.0f };
    glm::vec3 localPos{ 0.0f };

    // pre-computed data for impulse solving
    float accumulatedImpulse = 0.0f;
    float accumulatedFrictionImpulse1 = 0.0f;
    float accumulatedFrictionImpulse2 = 0.0f;
    float m_eff = 0.0f;
    glm::vec3 rA{ 0.0f };
    glm::vec3 rB{ 0.0f };
    float depth = 0.0f;
    float targetBounceVelocity = 0.0f;
    float biasVelocity = 0.0f;
    float invMassT1{ 0.0f };
    float invMassT2{ 0.0f };

    bool wasUsedThisFrame = true;
    bool wasWarmStarted = false;
};

// to distinguish between body vs body and body vs terrain contacts, since they have different response and contact point generation logic
enum class ContactPartnerType {
    RigidBody,
    Terrain
};

// runtime data for contact point generation and impulse solving, not stored in contact cache
struct ContactRuntime {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    Collider* colliderA = nullptr;
    Collider* colliderB = nullptr;
    Transform* bodyRootA = nullptr;
    Transform* bodyRootB = nullptr;
};

struct Contact {
    size_t hashKey = -1; // generated from collider IDs, used for caching

    // contact points for this contact, can be up to 4 for box-box and box-mesh collisions, otherwise usually 1
    std::vector<ContactPoint> points;

    // contact normal, precomputed from SAT result for warm starting and impulse solving
    glm::vec3 normal{ 0.0f };

    // tangential basis vectors, precomputed from the contact normal for warm starting and impulse solving
    glm::vec3 t1{ 0.0f };
    glm::vec3 t2{ 0.0f };

    // precomputed data for impulse solving
    glm::mat3 invInertiaA{ 0.0f };
    glm::mat3 invInertiaB{ 0.0f };
    float accumulatedTwistImpulse = 0.0f;
    float invMassTwist = 0.0f;

    // runtime data for contact point generation and impulse solving, not stored in contact cache
    ContactRuntime runtimeData;

    // to distinguish between body vs body and body vs terrain contacts, since they have different response and contact point generation logic
    ContactPartnerType partnerTypeA = ContactPartnerType::RigidBody;
    ContactPartnerType partnerTypeB = ContactPartnerType::RigidBody;
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;

    // reference face and normal for box-box and box-mesh collisions, used for contact point generation and warm starting
    bool noSolverResponseA = false;     // default false for dynamic bodies, true for static/kinematic bodies vs terrain
    bool noSolverResponseB = true;      // default true for terrain
    bool contributesMotionA = true;     // default true for dynamic bodies vs terrain
    bool contributesMotionB = false;    // default false for terrain

    // for contact caching
    bool wasUsedThisFrame = true;
    int framesSinceUsed = 0;

    // for box-box and box-mesh collisions, to avoid redundant computations in contact point generation and warm starting
    bool objBisReference = true; 
    std::array<glm::vec3, 4> referenceFace{ glm::vec3(0.0f) };
    std::array<glm::vec3, 4> incidentFace{ glm::vec3(0.0f) };
    glm::vec3 referenceFaceNormal{ 0.0f };

    // body vs body
    Contact(RigidBodyHandle handleA, RigidBodyHandle handleB, ContactRuntime& data, glm::vec3& normal) :
        partnerTypeA(ContactPartnerType::RigidBody),
        partnerTypeB(ContactPartnerType::RigidBody),
        bodyA(handleA),
        bodyB(handleB),
        runtimeData(data),
        normal(normal)
    {
        points.reserve(8);
    }

    // body vs terrain
    Contact(RigidBodyHandle handleA, ContactRuntime& data, glm::vec3& normal) :
        partnerTypeA(ContactPartnerType::RigidBody),
        partnerTypeB(ContactPartnerType::Terrain),
        bodyA(handleA),
        runtimeData(data),   
        normal(normal)
    {
        points.reserve(8);
    }

    Contact() = default;
};

class CollisionManifold {
public:
    void init(RuntimeCaches* caches) { this->caches = caches; }
    size_t generateKey(int idA, int idB);

    void boxBox(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);
    void boxSphere(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);
    void boxMesh(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, std::vector<SAT::Result>& allResults);
    void sphereSphere(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);
    void sphereMesh(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, std::vector<SAT::Result>& allResults);

private:
    RuntimeCaches* caches = nullptr;

    // reference face selection for box-box and box-mesh collisions
    void selectOOBBCollisionRefFaceAndNormal(const Collider* collider, ColliderPose& pose, const glm::vec3& normal, std::array<glm::vec3, 4>& outFace, glm::vec3& outNormal);
    void selectOOBBCollisionIncidentFace(const Collider* collider, ColliderPose& pose, const glm::vec3& normal, std::array<glm::vec3, 4>& outFace);

    // furthest point selection for terrain collisions
    std::vector<glm::vec3> furthestPoints;
    std::vector<int> indices;
    void pickFourFurthestPoints();
    void addFurthestPoint(std::vector<int>& indices);

    // Sutherland-Hodgman clipping
    std::vector<float> allPointsDepthsScratch;
    std::array<Plane, 4> clippingPlanes;
    std::array<glm::vec3, 16> contactPoints;
    std::array<glm::vec3, 16> nextContactPoints;
    std::array<bool, 16> clippingStatus;
    std::vector<glm::vec3> clippedPoints;
    std::vector<glm::vec3> allClippedPoints;
    void createClippingPlanes(const std::array<glm::vec3, 4>& face, const glm::vec3& faceNormal);
    void getIntersectionPoint(const glm::vec3& v1, const glm::vec3& v2, const Plane& plane, glm::vec3& outPoint, bool& outBool);
    bool isPointInsidePlane(const glm::vec3& point, const glm::vec3& planeNormal, const glm::vec3& planePoint, const float tolerance);
    void clipPoints(const std::array<glm::vec3, 4>& referenceFace, const std::array<glm::vec3, 4>& incidentFace, int incidentCount, const glm::vec3& referenceFaceNormal);

    // contact point reduction and penetration depth computation
    void contactPointReduction(Contact& contact);
    void computePenetrationDepth(std::vector<glm::vec3>& points, std::array<glm::vec3, 4>& refFace, glm::vec3& refFaceNormal, std::vector<float>& out);
    void PreComputePointData(ContactPoint& cp, Contact& contact);

    // contact cache integration
    void integrateContact(std::unordered_map<size_t, Contact>& contactCache, Contact& contact);

    //std::array<glm::vec3, 2> edgeEdgePoints(glm::vec3& P0, glm::vec3& P1, glm::vec3& Q0, glm::vec3& Q1);
};