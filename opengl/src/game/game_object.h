#pragma once

#include "pch.h"

#define GLM_ENABLE_EXPERIMENTAL

#include "world.h"
#include "shaders/shader.h"
#include "mesh/mesh.h"
#include "colliders/collider.h"
#include "colliders/aabb.h"
#include "colliders/oobb.h"
#include "colliders/sphere.h"
#include "debug/oobb_renderer.h"
#include "debug/aabb_renderer.h"
#include "broadphase/broadphase_types.h"

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
    int id;

    // hot data
    glm::vec3 position;
    glm::quat orientation{ 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };
    glm::mat3 inverseInertiaWorld;
    float invMass;
    glm::mat3 inverseInertia;

    glm::mat4 modelMatrix;
    bool modelMatrixDirty = true;
    glm::mat4 invModelMatrix;
    glm::mat3 rotationMatrix;
    glm::vec3 translationVector;
    glm::mat3 invRotationMatrix;
    bool helperMatricesDirty = true;

    int number = 5;

    // physics
    glm::vec3 scale;
    glm::vec3 biasLinearVelocity;
    glm::vec3 biasAngularVelocity;
    float linearVelocityLen = 0.0f;
    float angularVelocityLen = 0.0f;

    float mass;
    bool allowGravity;
    bool isStatic;
    bool isRotated = true;
    bool canMoveLinearly = true;
    glm::vec3 g = glm::vec3(0.0f, -9.81f, 0.0f);
    float radius;
    float invRadius;

    // collision
    AABB aabb;
    bool aabbDirty = true;
    Collider collider;
    ColliderType colliderType;

    // BVH
    BroadphaseHandle broadphaseHandle;

    // sleep
    bool asleep = false;
    bool allowSleep = true;
    float sleepCounter = 0;
    float sleepCounterThreshold;
    bool inSleepTransition = false;     // to avoid waking up immediately and to not add duplicate wake-up requests
    float velocityThreshold = 0;
    float angularVelocityThreshold = 0;

    float anchorTimer = 0.0f;      // used for sleeping to prevent jittering when waking up
    glm::vec3 anchorPoint;    // used for sleeping to prevent jittering when waking up

    int totalCollisionCount = 0;
    float lastAvg = 0.0f;
    CircBuffer collisionHistory;

    // render
    Shader* shader = nullptr;
    Mesh* mesh = nullptr;
    GLuint textureId;
    glm::vec3 color;
    bool useRandomColor = false;
    bool seeThrough = false;
    bool isInsideShadowFrustum = true;
    int batchIdx = -1;
    int batchInstanceIdx = -1;

    // editor
    bool hoveredByEditor = false;
    bool selectedByEditor = false;
    bool selectedByPlayer = false;
    glm::vec3 lastPosition;

    // player variables
    bool player = false;
    glm::vec3 playerMoveImpulse{ 0.0f };
    float playerJumpImpulse = 0.0f;
    bool onGround = false;
    bool hasJumped = false;

    std::vector<glm::vec3> verticesPositions;

    // constructor
    GameObject(
        int id,
        Shader* shader,
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
        glm::vec3 color = glm::vec3(255.0f, 255.0f, 255.0f),
        bool seeThrough = false
    )
        : id(id),
        shader(shader),
        mesh(mesh),
        collider(this),
        position(position),
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
        seeThrough(seeThrough)
    {
        // physics stuff
        //setRotatedFlag();
        invRadius = 1.0f / (0.5f * glm::length(scale));

        // dynamic
        if (!isStatic) {
            invMass = 1.0f / mass;
        }
        // static
        else {
            mass = 0;
            invMass = 0;
            asleep = false;
        }

        anchorPoint = position;

        // broadphase bucket
        if (asleep) {
            broadphaseHandle.bucket = BroadphaseBucket::Asleep;
        } else if (isStatic) {
            broadphaseHandle.bucket = BroadphaseBucket::Static;
        } else {
            broadphaseHandle.bucket = BroadphaseBucket::Awake;
        }

        // plain color
        //if (textureId == 999 && color.x != -1)      // color.x for random color shader hack
        this->color = color / 255.0f;

        initMesh();
        initCollider();
    }

    ~GameObject() {}
    void initMesh();
    void initCollider();
    void resetDirtyFlags();
    void setModelMatrix();
    void setHelperMatrices();
    void setRotatedFlag();
    void calculateInverseInertia(); 
    void inertiaCube(float side);
    void inertiaCuboid(float sx, float sy, float sz);
    void inertiaSphere();
    void updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float deltaTime);
    void updateAABB();
    void updateCollider();
    void updatePos(const float& dt);
    void setAsleep();
    void setAwake();
    void setStatic();

    void applyImpulseLinear(const glm::vec3& j);
    void applyImpulseAngular(const glm::vec3& j);

    AABB getAABB() const;
};