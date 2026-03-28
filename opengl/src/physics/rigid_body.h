#pragma once

#include "game/game_object.h"

enum class BodyType {
    Dynamic,
    Kinematic,
    Static
};

class RigidBody {
public:
    BodyType type = BodyType::Dynamic;
    GameObjectHandle gameObjectHandle;
    ColliderHandle colliderHandle;

    bool player = false;

    glm::vec3 position;
    glm::vec3 lastPosition;
    glm::quat orientation{ 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };
    glm::mat3 invInertiaLocal{ 0.0f };
    glm::mat3 invInertiaWorld{ 0.0f };
    float mass = 0.0f;
    float invMass = 0.0f;

    bool allowGravity;
    bool canMoveLinearly = true;
    glm::vec3 g = glm::vec3(0.0f, -9.81f, 0.0f);
    float radius;
    float invRadius;

    // sleep
    bool asleep = false;
    bool allowSleep = true;
    bool inSleepTransition = false; // to avoid waking up immediately and to not add duplicate wake-up requests
    float sleepCounter = 0;
    float sleepCounterThreshold;
    float velocityThreshold = 0;
    float angularVelocityThreshold = 0;
    float anchorTimer = 0.0f;
    glm::vec3 anchorPoint;
    int totalCollisionCount = 0;
    float lastAvg = 0.0f;
    RingBuffer collisionHistory;

    void update(float dt);
    void updateInertiaWorld();
    void updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float dt);

    void setAsleep();
    void setAwake();
    void setStatic();

    void calculateInverseInertia(const ColliderType& type, const Transform& t);
    void inertiaCube(const float sideX);
    void inertiaCuboid(const glm::vec3& scale);
    void inertiaSphere(const glm::vec3& scale);
};