#pragma once

#include "game/transform.h"
#include "core/ring_buffer.h"
#include "core/slot_map.h"
#include "physics/colliders/collider.h"

enum class BodyType {
    Dynamic,
    Kinematic,
    Static
};

enum class MotionControl {
    Physics,
    External
};

enum class ContactResponseMode {
    Normal,
    Character
};

// #TODO: decide what is private/public in RigidBody
class RigidBody {
public:
    int id = -1;
    BodyType type = BodyType::Dynamic;
    MotionControl motionControl = MotionControl::Physics;
    ContactResponseMode responseMode = ContactResponseMode::Normal;

    // #TODO: lägga till COM
    // annars fungerar fysiken bara om collidernas lokala transform är centrerade runt COM

    // handles
    GameObjectHandle gameObjectHandle;
    BroadphaseHandle broadphaseHandle;
    TransformHandle rootTransformHandle;
    std::vector<ColliderHandle> colliderHandles;

    AABB aabb; // if compound, this is the AABB of the whole body, otherwise AABB of the single collider.

    // physics properties
    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };
    glm::mat3 invInertiaLocal{ 0.0f };
    glm::mat3 invInertiaWorld{ 0.0f };
    float mass = 0.0f;
    float invMass = 0.0f;

    bool allowGravity = true;
    bool canMoveLinearly = true;
    float radius = 0.0f;
    float invRadius = 0.0f;
    static constexpr glm::vec3 g = glm::vec3(0.0f, -9.81f, 0.0f);

    // sleep
    bool asleep = false;
    bool allowSleep = true;
    bool inSleepTransition = false; // to avoid waking up immediately and to not add duplicate wake-up requests
    float sleepCounter = 0;
    float sleepCounterThreshold = 1.5f;
    float velocityThreshold = 0;
    float angularVelocityThreshold = 0;
    float anchorTimer = 0.0f;
    glm::vec3 anchorPoint{ 0.0f };
    int totalCollisionCount = 0;
    float lastAvg = 0.0f;
    RingBuffer collisionHistory;


    bool isCompound() const {
        return colliderHandles.size() > 1;
    }

    void integrateVelocities(Transform& t, float dt);
    void applyGravity(float dt);
    void applyRollingFriction(ColliderType colliderType, float dt);
    void applyAntistuckFriction(float dt);
    void updateOrientation(glm::quat& orientation, const glm::vec3& angularVelocity, float dt);
    void updateInertiaWorld(Transform& t);

    void applyImpulseLinear(const glm::vec3& impulse);
    void applyImpulseAngular(const glm::vec3& impulse);

    void setAsleep(Transform& t);
    void setAwake();
    void setStatic();
    void setExternalControl(bool controlled);

    void calculateInverseInertia(const ColliderType& type, const Collider& collider, Transform& t);
    void inertiaCube(const float sideX);
    void inertiaCuboid(const glm::vec3& scale);
    void inertiaSphere(const glm::vec3& scale);

    bool approxEqual(float a, float b, float epsilon = 0.0001f);
};