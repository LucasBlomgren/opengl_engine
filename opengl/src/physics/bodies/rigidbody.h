#pragma once

#include "core/ring_buffer.h"
#include "physics/public/handles.h"
#include "physics/public/physics_types.h"
#include "physics/broadphase/rigidbody_types.h"
#include "physics/colliders/collider.h"

namespace physics::internal {

class RigidBody {
public:
    int id = -1;
    BodyType type = BodyType::Dynamic;
    MotionControl motionControl = MotionControl::Physics;
    ResponseMode responseMode = ResponseMode::Normal;

    // #TODO: lägga till COM
    // annars fungerar fysiken bara om collidernas 
    // lokala transform är centrerade runt COM

    Pose pose;

    // #TODO: ta bort scale helt från fysiken,
    // scale behövs bara initialt för att bakea en 
    // mesh till en collider och det kan göras utanför fysikmotorn
    glm::vec3 scale{ 1.0f };

    // physics-internal handles
    BroadphaseHandle broadphaseHandle;
    std::vector<ColliderHandle> colliderHandles;

    // if compound, this is the AABB of the whole body, 
    // otherwise AABB of the single collider.
    AABB aabb;

    //==============================
    // physics
    //=============================
    uint32_t lastBiasCommitFrame = 0;
    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };
    glm::vec3 biasLinearVelocity{ 0.0f };
    glm::vec3 biasAngularVelocity{ 0.0f };
    glm::mat3 invInertiaLocal{ 0.0f };
    glm::mat3 invInertiaWorld{ 0.0f };
    float mass = 0.0f;
    float invMass = 0.0f;

    bool allowGravity = true;
    float radius = 0.0f;
    float invRadius = 0.0f;

    static constexpr glm::vec3 g = 
        glm::vec3(0.0f, -9.81f, 0.0f);

    //==============================
    // sleep 
    //==============================
    bool asleep = false;
    bool allowSleep = true;

    // to avoid waking up immediately and 
    // to not add duplicate wake-up requests
    bool inSleepTransition = false;

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

    void pushBiasImpulseLinear(const glm::vec3& impulse);
    void pushBiasImpulseAngular(const glm::vec3& impulse);
    void commitBiasImpulses(float dt);

    void applyImpulseLinear(const glm::vec3& impulse);
    void applyImpulseAngular(const glm::vec3& impulse);

    void setAsleep();
    void setAwake();
    void setStatic();
    void setMotionControl(MotionControl mode);

    void integratePose(float dt);

    void updateOrientation(
        glm::quat& orientation, 
        const glm::vec3& angularVelocity, 
        float dt
    );

    void updateInertiaWorld();

    void applyGravity(float dt);
    void applyVelocityDamping(float dt);
    void applyRollingFriction(ColliderType colliderType, float dt);
    void applyAntistuckFriction(float dt);

    void calculateInverseInertia(
        const ColliderType& type,
        const Collider& collider,
        const glm::vec3& inertiaScale);

    void inertiaCube(const float sideX);
    void inertiaCuboid(const glm::vec3& scale);
    void inertiaSphere(const glm::vec3& scale);

    bool approxEqual(float a, float b, float epsilon = 0.0001f) {
        return fabs(a - b) < epsilon;
    }
};

}
