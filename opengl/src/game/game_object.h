#pragma once

#include "pch.h"

#define GLM_ENABLE_EXPERIMENTAL

#include "shaders/shader.h"
#include "mesh/mesh.h"
#include "colliders/collider.h"
#include "colliders/aabb.h"
#include "colliders/oobb.h"
#include "colliders/sphere.h"
#include "debug/oobb_renderer.h"
#include "debug/aabb_renderer.h"

#include <glm/gtc/quaternion.hpp>    // defines glm::quat
#include <glm/gtx/quaternion.hpp>    // extra helpers om du behöver

inline bool approxEqual(float a, float b, float epsilon = 0.0001f) {
    return fabs(a - b) < epsilon;
}

struct CircBuffer {
   static constexpr int SLOTS = 5;
   int buffer[SLOTS] = { 0 };
   int index = 0;
   int count = 0;

   // Ska köras varje frame med det nya värdet:
   void push(int newCount) {
      buffer[index] = newCount;
      index = (index + 1) % SLOTS;
      if (count < SLOTS) ++count;
   }

   // Ger medelvärdet över de senast count värdena (upp till 5)
   float average() const {
      if (count == 0) return 0.0f;
      
      int sum = 0;
      for (int i = 0; i < count; ++i) {
         sum += buffer[i];
      }
      return (float)sum / (float)count;
   }
};

class GameObject {
public:
    // player variables
    bool player = false;
    glm::vec3 playerMoveImpulse{ 0.0f };
    float playerJumpImpulse = 0.0f;
    bool onGround = false;
    bool hasJumped = false;

   // mesh variables
    int id;
    glm::vec3 position;
    std::vector<glm::vec3> verticesPositions;

    bool isInsideShadowFrustum = true;

    glm::mat4 modelMatrix;
    bool modelMatrixDirty = true;
    bool seeThrough = false;

    glm::mat4 invModelMatrix;
    glm::mat3 rotationMatrix;
    glm::vec3 translationVector;
    glm::mat3 invRotationMatrix;
    bool helperMatricesDirty = true;

    Shader* shader = nullptr;
    Mesh* mesh = nullptr;
    GLuint textureId;
    glm::vec3 color;

    // physics variables
    glm::quat orientation;
    glm::mat3 inverseInertia;
    glm::mat3 inverseInertiaWorld;
    glm::vec3 scale;
    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };
    glm::vec3 biasLinearVelocity;
    glm::vec3 biasAngularVelocity;
    float linearVelocityLen = 0.0f;
    float angularVelocityLen = 0.0f;

    float mass;
    float invMass;
    float invInertia;
    bool allowGravity;
    bool isStatic;
    bool shouldRotate;
    bool isRotated = false;
    bool canMoveLinearly = true;
    glm::vec3 g = glm::vec3(0.0f, -9.81f, 0.0f);
    float radius;
    float invRadius;

    // sleep variables
    bool asleep = false;
    bool allowSleep = true;
    float sleepCounter = 0;
    float sleepCounterThreshold;
    bool inSleepTransition = false;     // to avoid waking up immediately and to not add duplicate wake-up requests
    float velocityThreshold = 0;
    float angularVelocityThreshold = 0;

    int totalCollisionCount = 0;
    float lastAvg = 0.0f;
    CircBuffer collisionHistory;

    // collision variables
    AABB aabb;
    bool aabbDirty = true;
    Collider collider;
    ColliderType colliderType;
    OOBBRenderer oobbRenderer;

    int bvhLeafIdx       = -1;
    int dynamicObjectIdx = -1;
    int awakeListIdx     = -1;
    int asleepListIdx    = -1;
    int staticListIdx    = -1;

    // editor variables
    bool selectedByEditor = false;
    bool isRaycastHit = false;
    glm::vec3 lastPosition;
    glm::vec3 pushCorrection;
    
    // constructor
    GameObject(
        int id,
        Mesh* mesh,
        glm::vec3 position,
        glm::vec3 scale,
        ColliderType colliderType,
        float mass,
        bool isStatic,
        int textureId,
        glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
        float sleepCounterThreshold = 1.0f,
        bool asleep = false,
        glm::vec3 color = glm::vec3(255.0f, 255.0f, 255.0f)
    )
        : id(id),
        mesh(mesh),
        position(position),
        shouldRotate(true),
        scale(scale),
        allowGravity(true),
        isStatic(isStatic),
        mass(mass),
        textureId(textureId),
        orientation(orientation),
        sleepCounterThreshold(sleepCounterThreshold),
        asleep(asleep),
        color(color),
        colliderType(colliderType),
        collider(this)
    {
        // physics stuff
        setRotatedFlag();
        invRadius = 1.0f / (0.5f * glm::length(scale));

        // dynamic
        if (!isStatic) {
            invMass = 1.0f / mass;
            asleep = false;
        }
        // static
        else {
            mass = 0;
            invMass = 0;
            asleep = false;
        }

        // plain color
        if (textureId == 999)
           this->color = color / 255.0f;

        for (const Vertex& vertex : mesh->vertices) {
            verticesPositions.push_back(vertex.position);
        }

        setModelMatrix();

        // render mesh
        //setupMesh();

        // AABB 
        aabb.Init(verticesPositions);
        aabb.update(modelMatrix, position, scale, isRotated);

        // Collider
        if (colliderType == ColliderType::CUBOID) {
            OOBB box(verticesPositions, modelMatrix, scale);
            collider.shape = box;

            glm::vec3 size = box.lHalfExtents * 2.0f * scale;  
            bool isUniform = approxEqual(size.x, size.y) && approxEqual(size.y, size.z);

            oobbRenderer.setupWireframeBox();
            oobbRenderer.setupNormals();

            // detta ska vara baserat på OOBB storleken (halfExtents), inte bara scale som det är nu
            if (isStatic) {
                inverseInertia = glm::mat3(0.0f);
            } else if (isUniform) {
                calculateInverseInertiaForCube(size.x);
            } else {
                calculateInverseInertiaForCuboid(size.x, size.y, size.z);
            }
        }
        else if (colliderType == ColliderType::SPHERE) {
            Sphere sphere(modelMatrix, scale.x);
            collider.shape = sphere; 

            if (isStatic) {
                inverseInertia = glm::mat3(0.0f); 
            } else {
                calculateInverseInertiaForSolidSphere();
            }
        }

        inverseInertiaWorld = inverseInertia; 
    }

    void resetDirtyFlags();
    void setModelMatrix();
    void setHelperMatrices();
    void setRotatedFlag();
    void calculateInverseInertiaForCube(float side);
    void calculateInverseInertiaForCuboid(float sx, float sy, float sz);
    void calculateInverseInertiaForSolidSphere();
    void updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float deltaTime);
    void renderMesh(Shader& shader);
    void updateAABB();
    void updateCollider();
    void updatePos(const float& dt);
    void setAsleep();
    void setAwake();

    void applyImpulseLinear(const glm::vec3& j);
    void applyImpulseAngular(const glm::vec3& j);

    AABB getAABB() const;

private:

};