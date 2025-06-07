#pragma once
#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/io.hpp>

#include <optional>
#include <cmath>
#include <unordered_map>
#include "draw_line.h"
#include "game_object.h"

struct Plane {
    glm::vec3 normal;
    glm::vec3 point;
};

struct InitialContact {
    GameObject* objA_ptr;
    GameObject* objB_ptr;
    std::array<glm::vec3, 8> globalCoords;
    std::array<glm::vec3, 8> localCoords;
    std::array<float, 8> pointDepths;
    int counter = 0;
    std::array<glm::vec3, 4> referenceFace;
    std::array<glm::vec3, 4> incidentFace;
    glm::vec3 referenceFaceNormal;

    InitialContact(GameObject* objA, GameObject* objB, const std::array<glm::vec3, 4> refFace, const std::array<glm::vec3, 4> incFace, const glm::vec3 n)
        : objA_ptr(objA), objB_ptr(objB), referenceFace(refFace), incidentFace(incFace), referenceFaceNormal(n)
    {}
};

struct ContactPoint {
    glm::vec3 globalCoord;
    glm::vec3 localCoord;
    float accumulatedImpulse = 0.0f;
    float accumulatedFrictionImpulse1 = 0.0f;
    float accumulatedFrictionImpulse2 = 0.0f;
    float accumulatedTwistImpulse = 0.0f;
    float m_eff;
    glm::vec3 rA;
    glm::vec3 rB;
    glm::vec3 t1;
    glm::vec3 t2;
    float depth;
    float targetBounceVelocity;
    float biasVelocity;

    float invMassT1;
    float invMassT2;
    float invMassTwist;
};

struct Contact {
    bool wasUsedThisFrame = true;
    int framesSinceUsed = 0;
    GameObject* objA_ptr;
    GameObject* objB_ptr;
    std::array<ContactPoint, 4> points;
    int counter = 0;
    glm::vec3 normal;

    Contact() = default;
};

class CollisionManifold {
public:
    void createContact(Contact& outContact, std::unordered_map<size_t, Contact>& cache, GameObject& objA, GameObject& objB, glm::vec3 normal, int& collisionNormalOwner);

    void cuboidVsCuboid(Contact& outContact, std::unordered_map<size_t, Contact>& cache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner);
    void sphereVsCuboid(Contact& outContact, std::unordered_map<size_t, Contact>& cache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner);
    void sphereVsSphere(Contact& outContact, std::unordered_map<size_t, Contact>& cache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner);
    void meshVsCuboid(Contact& outContact, std::unordered_map<size_t, Contact>& cache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner);
    void meshVsSphere(Contact& outContact, std::unordered_map<size_t, Contact>& cache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner);
    void meshVsMesh(Contact& outContact, std::unordered_map<size_t, Contact>& cache, GameObject& A, GameObject& B, glm::vec3 n, int& normalOwner);

private:
    // createContactPoints
    std::array<Plane, 4> clippingPlanes;
    std::array<glm::vec3, 8> contactPoints;
    std::array<glm::vec3, 8> nextContactPoints;
    std::array<bool, 8> clippingStatus;

    void selectCollisionFace(GameObject& obj, const glm::vec3& normal, std::array<glm::vec3, 4>& outFace);
    void createClippingPlanes(const std::array<glm::vec3, 4>& face, const glm::vec3& faceNormal);
    void getIntersectionPoint(const glm::vec3& v1, const glm::vec3& v2, const Plane& plane, glm::vec3& outPoint, bool& outBool);
    bool isPointInsidePlane(const glm::vec3& point, const glm::vec3& planeNormal, const glm::vec3 planePoint);
    void createContactPoints(InitialContact& initialContact);
    void createLocalCoordinates(InitialContact& initialContact);
    void contactPointReduction(InitialContact& contactPoints);
    void computePenetrationDepth(InitialContact& initialContact);
    void PreComputePointData(ContactPoint& cp, glm::vec3& normal, GameObject& objA, GameObject& objB);
    size_t generateKey(int idA, int idB);
    void integrateContact(std::unordered_map<size_t, Contact>& contactCache, InitialContact& initialContact, Contact& finalContact);
};