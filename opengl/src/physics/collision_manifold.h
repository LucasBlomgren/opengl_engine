#pragma once

#include <unordered_map>

#include "game_object.h"
#include "sat.h"

struct Plane {
    glm::vec3 normal;
    glm::vec3 point;
};

struct ContactPoint {
    glm::vec3 globalCoord;
    glm::vec3 localCoord;
    float accumulatedImpulse = 0.0f;
    float accumulatedFrictionImpulse1 = 0.0f;
    float accumulatedFrictionImpulse2 = 0.0f;
    float accumulatedTwistImpulse = 0.0f;
    float m_eff;
    glm::vec3 rA, rB;
    float depth;
    float targetBounceVelocity;
    float biasVelocity;

    float invMassT1, invMassT2;
    float invMassTwist;

    bool wasUsedThisFrame = true;
    bool wasWarmStarted = false;
};

struct Contact {
    size_t hashKey;
    std::vector<ContactPoint> points;
    glm::vec3 normal;
    glm::vec3 t1, t2;

    GameObject* objA_ptr;
    GameObject* objB_ptr;
    bool freezeA = false;
    bool freezeB = false;

    bool wasUsedThisFrame = true;
    int framesSinceUsed = 0;

    bool objBisReference = true; 
    std::vector<glm::vec3> referenceFace;
    std::vector<glm::vec3> incidentFace;
    glm::vec3 referenceFaceNormal;

    Contact(GameObject* A, GameObject* B) : objA_ptr(A), objB_ptr(B) { points.reserve(8); }
    Contact() = default;
};

class CollisionManifold {
public:
    void boxBox(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);
    void boxSphere(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);
    void boxMesh(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, std::vector<SAT::Result>& allResults);
    void sphereSphere(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);
    void sphereMesh(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, std::vector<SAT::Result>& allResults);
    void meshMesh(Contact& outContact, std::unordered_map<size_t, Contact>& contactCache, SAT::Result& satResult);

    size_t generateKey(int idA, int idB);

private:
    std::vector<glm::vec3> selectedFace;
    void selectCollisionFace(GameObject& obj, const glm::vec3& normal);

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