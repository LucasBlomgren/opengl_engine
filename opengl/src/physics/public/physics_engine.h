#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include <optional>

#include "collider_desc.h"
#include "collider_state.h"
#include "physics_handles.h"
#include "rigid_body_desc.h"
#include "rigid_body_state.h"

#include "physics_debug_types.h"

#include "physics/world/physics_world.h"
#include "physics/narrowphase/collision_manifold.h"
#include "physics/raycast/raycast.h"

#include "physics/bvh/bvh.h"
#include "physics/bvh/bvh_terrain.h"

#include "physics/colliders/aabb.h"
#include "physics/colliders/tri.h"


class EngineState;
class FrameTimers;
class World;

class PhysicsEngine {
public:
    //======================================
    // Construction and ownership
    //======================================
    PhysicsEngine();
    ~PhysicsEngine();

    // Disable copy semantics
    PhysicsEngine(const PhysicsEngine&) = delete;
    PhysicsEngine& operator=(const PhysicsEngine&) = delete;

    // Allow move semantics
    PhysicsEngine(PhysicsEngine&&) noexcept;
    PhysicsEngine& operator=(PhysicsEngine&&) noexcept;

    //======================================
    // Initialization and scene lifetime
    //======================================
    void init(World* world, FrameTimers* frameTimers);
    void setupScene(std::vector<Tri>* terrainTriangles);
    void clear();

    //======================================
    // Simulation
    //======================================
    void prepareStepLoop();
    void step(float deltaTime, EngineState& engine);

    //================================================
    // Rigid body & collider creation and destruction
    //================================================
    [[nodiscard]] RigidBodyHandle createRigidBody(const RigidBodyDesc& desc);
    bool destroyRigidBody(RigidBodyHandle body);

    [[nodiscard]] ColliderHandle createCollider(RigidBodyHandle body, const ColliderDesc& desc);
    bool destroyCollider(ColliderHandle collider);

    //======================================
    // Rigid body commands
    //======================================
    bool applyLinearImpulse(RigidBodyHandle body, const glm::vec3& impulse);
    bool setLinearVelocity(RigidBodyHandle body, const glm::vec3& velocity);
    bool setAngularVelocity(RigidBodyHandle body, const glm::vec3& velocity);
    bool setKinematicTarget(RigidBodyHandle body, const PhysicsPose& target);

    bool setRigidBodySleepState(RigidBodyHandle body, bool awake);
    bool setRigidBodyType(RigidBodyHandle body, BodyType type);
    bool setRigidBodyMotionControl(RigidBodyHandle body, MotionControl motionControl);

    //======================================
    // Collider commands
    //======================================
    bool setColliderLocalPose(ColliderHandle collider, const PhysicsPose& localPose);
    bool setColliderEnabled(ColliderHandle collider, bool enabled);
    bool setColliderTrigger(ColliderHandle collider, bool isTrigger);

    //======================================
    // State queries
    //======================================
    std::optional<RigidBodyState> getRigidBodyState(RigidBodyHandle body) const;
    std::optional<ColliderState> getColliderState(ColliderHandle collider) const;

    //======================================
    // Physics queries
    //======================================
    Raycast::RaycastHit raycast(Raycast::Ray& ray);

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
    DebugData getDebugData() const;
    PhysicsStepDebugPhase getDebugPhase() const;

    //======================================
    // Debug visualization
    //======================================
    void updateBVHRenderData(const BVHType& type, bool update);

    const std::vector<AABB>& getDebugSweeps() const;
    const std::vector<DebugSpeculativeContact>& getDebugSpeculativeContacts() const;
    const std::unordered_map<size_t, Contact>& getContactCache() const;

    //======================================
    // Debug spatial data
    //======================================
    const std::vector<RigidBodyHandle>& getAwakeList() const;
    const BVHTree& getDynamicAwakeBvh() const;
    const BVHTree& getDynamicAsleepBvh() const;
    const BVHTree& getStaticBvh() const;
    const TerrainBVH& getTerrainBvh() const;

    //======================================
    // Temporary legacy API
    //======================================
    PhysicsWorld* getPhysicsWorld();

    void syncBodyFromTransform(RigidBodyHandle body);
    void setBVHDirty(RigidBodyHandle& handle);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};