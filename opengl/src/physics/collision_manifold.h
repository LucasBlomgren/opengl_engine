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

struct Plane
{
    glm::vec3 normal;
    glm::vec3 point;  // point on plane
};

struct InitialContact
{
    GameObject* objA_ptr;
    GameObject* objB_ptr;
    std::array<glm::vec3, 8> globalCoords;
    std::array<glm::vec3, 8> localCoords;
    std::array<float, 8> pointDepths;
    int counter = 0;
    std::array<glm::vec3, 4> referenceFace;
    std::array<glm::vec3, 4> incidentFace;
    glm::vec3 referenceFaceNormal;

    InitialContact() = default;

    InitialContact(GameObject* objA, GameObject* objB, const std::array<glm::vec3, 4> refFace, const std::array<glm::vec3, 4> incFace, const glm::vec3 n)
        : objA_ptr(objA), objB_ptr(objB), referenceFace(refFace), incidentFace(incFace), referenceFaceNormal(n)
    {}
};

struct ContactPoint
{
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
};

struct Contact
{
    bool wasUsedThisFrame = true;
    int framesSinceUsed = 0;
    GameObject* objA_ptr;
    GameObject* objB_ptr;
    std::array<ContactPoint, 4> points;
    int counter = 0;
    glm::vec3 normal;
    //float depth;

    Contact() = default;

    Contact(GameObject* objA, GameObject* objB, const glm::vec3 n)
        : objA_ptr(objA), objB_ptr(objB), normal(n)
    {}
};

std::array<glm::vec3, 4> selectCollisionFace(GameObject& obj, const glm::vec3& normal);
std::array<Plane, 4> createClippingPlanes(const std::array<glm::vec3, 4>& face, const glm::vec3& faceNormal);
std::optional<glm::vec3> getIntersectionPoint(const glm::vec3& v1, const glm::vec3& v2, const Plane& plane);
bool isPointInsidePlane(const glm::vec3& point, const glm::vec3& planeNormal, const glm::vec3 planePoint);
void createContactPoints(InitialContact& initialContact);
void createLocalCoordinates(InitialContact& initialContact);
void contactPointReduction(InitialContact& contactPoints);
void computePenetrationDepth(InitialContact& initialContact);
void PreComputeContactData(Contact& contact);
size_t generateKey(int idA, int idB);
void integrateContact(std::unordered_map<size_t, Contact>& contactCache, InitialContact& initialContact, Contact& finalContact);
Contact createContact(std::unordered_map<size_t, Contact>& contactCache, GameObject& objA, GameObject& objB, glm::vec3 normal, int& collisionNormalOwner);

void drawClippingPlanes(const Shader& shader, unsigned int& VAO, const std::array<Plane, 4>& clippingPlanes, const std::array<glm::vec3, 4>& referenceFace);