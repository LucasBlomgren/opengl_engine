#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "collider_desc.h"
#include "collider_state.h"
#include "physics_contact_types.h"
#include "physics_debug_types.h"
#include "physics_handles.h"
#include "physics_query_types.h"
#include "physics_scene_types.h"
#include "rigid_body_desc.h"
#include "rigid_body_state.h"

class EngineState;
class FrameTimers;

namespace physics {

namespace internal {
    struct EngineImpl;
}

class Engine {
public:
    //======================================
    // Construction and ownership
    //======================================
    Engine();
    ~Engine();

    // Disable copy semantics
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    // Allow move semantics
    Engine(Engine&&) noexcept;
    Engine& operator=(Engine&&) noexcept;

    //======================================
    // Initialization and scene lifetime
    //======================================
    void init(FrameTimers* frameTimers);
    void setupScene(const std::vector<Triangle>& terrainTriangles);
    void clear();

    //======================================
    // Simulation
    //======================================
    void prepareStepLoop();
    void step(float deltaTime, EngineState& engine);

    //================================================
    // Rigid body & collider creation and destruction
    //================================================
    [[nodiscard]] BodyHandle createRigidBody(const BodyDesc& desc);
    bool destroyRigidBody(BodyHandle body);

    [[nodiscard]] ColliderHandle createCollider(BodyHandle body, const ColliderDesc& desc);
    bool destroyCollider(ColliderHandle collider);

    //======================================
    // Rigid body commands
    //======================================
    bool applyLinearImpulse(BodyHandle body, const glm::vec3& impulse);
    bool setLinearVelocity(BodyHandle body, const glm::vec3& velocity);
    bool setAngularVelocity(BodyHandle body, const glm::vec3& velocity);
    bool setKinematicTarget(BodyHandle body, const Pose& target);
    bool setRigidBodyTransform(
        BodyHandle body,
        const Pose& pose,
        const glm::vec3& scale);

    bool setRigidBodySleepState(BodyHandle body, bool asleep);
    bool setRigidBodyType(BodyHandle body, BodyType type);
    bool setRigidBodyMotionControl(BodyHandle body, MotionControl motionControl);
    bool setRigidBodyResponseMode(BodyHandle body, ResponseMode responseMode);
    bool setRigidBodyMass(BodyHandle body, float mass);
    bool setRigidBodyAllowGravity(BodyHandle body, bool allowGravity);
    bool setRigidBodyAllowSleep(BodyHandle body, bool allowSleep);
    bool setRigidBodyCanMoveLinearly(BodyHandle body, bool canMoveLinearly);

    //======================================
    // Collider commands
    //======================================
    bool setColliderLocalPose(ColliderHandle collider, const Pose& localPose);
    bool setColliderLocalTransform(
        ColliderHandle collider,
        const Pose& localPose,
        const glm::vec3& localScale);
    bool setColliderShape(ColliderHandle collider, const ColliderShapeDesc& shape);
    bool setColliderEnabled(ColliderHandle collider, bool enabled);
    bool setColliderTrigger(ColliderHandle collider, bool isTrigger);

    //======================================
    // State queries
    //======================================
    std::optional<BodyState> getRigidBodyState(BodyHandle body) const;
    std::optional<ColliderState> getColliderState(ColliderHandle collider) const;

    //======================================
    // Physics queries
    //======================================
    RaycastHit raycast(
        const Ray& ray,
        BodyHandle ignoredBody = {});
    std::vector<BodyHandle> queryBodies(
        const AABB& bounds,
        BodySet bodySet) const;

    //======================================
    // Scene-wide commands
    //======================================
    void sleepAllObjects();
    void awakenAllObjects();

    //======================================
    // Solver configuration
    //======================================
    int getPgsIterations() const;
    void setPgsIterations(int iterations);

    //======================================
    // Simulation output
    //======================================
    std::vector<ExternalMotionContact>& getExternalMotionContacts();

    //======================================
    // Debug state
    //======================================
    debug::Data getDebugData() const;
    debug::StepPhase getDebugPhase() const;

    //======================================
    // Debug visualization
    //======================================
    void updateBVHRenderData(const debug::BvhType& type, bool update);

    std::vector<AABB> getDebugSweeps() const;
    std::vector<debug::SpeculativeContact> getDebugSpeculativeContacts() const;
    std::vector<debug::Contact> getDebugContacts() const;

    //======================================
    // Debug spatial data
    //======================================
    const std::vector<BodyHandle>& getAwakeList() const;
    debug::Bvh getDebugBvh(debug::BvhType type) const;
    debug::Bvh getTerrainDebugBvh() const;

private:
    std::unique_ptr<internal::EngineImpl> impl;
};

}
