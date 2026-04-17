#pragma once

#include <unordered_map>

#include "runtime_caches.h"
#include "rigid_body.h"
#include "sat.h"
#include "collider_pose.h"

struct Plane {
    glm::vec3 normal{ 0.0f };
    glm::vec3 point{ 0.0f };
};

struct ContactPoint {
    glm::vec3 globalCoord{ 0.0f };
    glm::vec3 localCoord{ 0.0f };
    float accumulatedImpulse = 0.0f;
    float accumulatedFrictionImpulse1 = 0.0f;
    float accumulatedFrictionImpulse2 = 0.0f;
    float m_eff = 0.0f;
    glm::vec3 rA{ 0.0f };
    glm::vec3 rB{ 0.0f };
    float depth = 0.0f;
    float targetBounceVelocity = 0.0f;
    float biasVelocity = 0.0f;

    float invMassT1, invMassT2 = 0.0f;

    bool wasUsedThisFrame = true;
    bool wasWarmStarted = false;
};

enum class ContactPartnerType {
    RigidBody,
    Terrain
};

struct ContactRuntime {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    Collider* colliderA = nullptr;
    Collider* colliderB = nullptr;
    Transform* bodyRootA = nullptr;
    Transform* bodyRootB = nullptr;
};

struct Contact {
    size_t hashKey = -1;
    std::vector<ContactPoint> points;
    glm::vec3 normal{ 0.0f };
    glm::vec3 t1{ 0.0f };
    glm::vec3 t2{ 0.0f };

    glm::mat3 invInertiaA{ 0.0f };
    glm::mat3 invInertiaB{ 0.0f };
    float accumulatedTwistImpulse = 0.0f;
    float invMassTwist = 0.0f;

    ContactRuntime runtimeData;

    ContactPartnerType partnerTypeA = ContactPartnerType::RigidBody;
    ContactPartnerType partnerTypeB = ContactPartnerType::RigidBody;
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;

    bool noSolverResponseA = false; 
    bool noSolverResponseB = true;      // default true for terrain
    bool contributesMotionA = true;     // default true for dynamic bodies vs terrain
    bool contributesMotionB = false;

    bool wasUsedThisFrame = true;
    int framesSinceUsed = 0;

    bool objBisReference = true; 
    std::vector<glm::vec3> referenceFace;
    std::vector<glm::vec3> incidentFace;
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

    void boxBox(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);
    void boxSphere(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);
    void boxMesh(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, std::vector<SAT::Result>& allResults);
    void sphereSphere(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);
    void sphereMesh(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, std::vector<SAT::Result>& allResults);

    size_t generateKey(int idA, int idB);

private:
    RuntimeCaches* caches = nullptr;

    std::vector<glm::vec3> selectedFace;
    void selectOOBBCollisionFace(const Collider* collider, ColliderPose& pose, const glm::vec3& normal);

    std::vector<glm::vec3> furthestPoints; // uses allClippedPoints
    std::vector<int> indices; // indices of furthestPoints in allClippedPoints
    void pickFourFurthestPoints();
    void addFurthestPoint(std::vector<int>& indices);

    std::vector<Plane> clippingPlanes;
    std::array<glm::vec3, 16> contactPoints;
    std::array<glm::vec3, 16> nextContactPoints;
    std::array<bool, 16> clippingStatus;
    std::vector<glm::vec3> clippedPoints;
    std::vector<glm::vec3> allClippedPoints;
    void createClippingPlanes(const std::vector<glm::vec3>& face, const glm::vec3& faceNormal);
    void getIntersectionPoint(const glm::vec3& v1, const glm::vec3& v2, const Plane& plane, glm::vec3& outPoint, bool& outBool);
    bool isPointInsidePlane(const glm::vec3& point, const glm::vec3& planeNormal, const glm::vec3& planePoint, const float tolerance);
    void clipPoints(std::vector<glm::vec3>& referenceFace, std::vector<glm::vec3>& incidentFace, glm::vec3& referenceFaceNormal);

    std::array<glm::vec3, 2> edgeEdgePoints(glm::vec3& P0, glm::vec3& P1, glm::vec3& Q0, glm::vec3& Q1);

    void createLocalCoordinates(Contact& contact);
    void contactPointReduction(Contact& contact);
    void computePenetrationDepth(std::vector<glm::vec3>& points, std::vector<glm::vec3>& refFace, glm::vec3& refFaceNormal, std::vector<float>& out);
    void PreComputePointData(ContactPoint& cp, Contact& contact);
    void integrateContact(std::unordered_map<size_t, Contact>& contactCache, Contact& contact);
};