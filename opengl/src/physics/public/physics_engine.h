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
#include "physics/broadphase/rigidbody_broadphase_types.h"
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
    PhysicsEngine();
    ~PhysicsEngine();

    PhysicsEngine(const PhysicsEngine&) = delete;
    PhysicsEngine& operator=(const PhysicsEngine&) = delete;

    PhysicsEngine(PhysicsEngine&&) noexcept;
    PhysicsEngine& operator=(PhysicsEngine&&) noexcept;

    void init(World* world, FrameTimers* frameTimers);
    void setupScene(std::vector<Tri>* terrainTriangles);
    void clear();

    void prepareStepLoop();
    void step(float deltaTime, EngineState& engine);

    void sleepAllObjects();
    void awakenAllObjects();

    RaycastHit performRaycast(Ray& ray);

    void updateBVHRenderData(const BVHType& type, bool update);

    DebugData getDebugData() const;
    PhysicsStepDebugPhase getDebugPhase() const;
    float getPausedDt() const;

    int getPgsIterations() const;
    void setPgsIterations(int iterations);

    const std::vector<AABB>& getDebugSweeps() const;
    const std::vector<DebugSpeculativeContact>& getDebugSpeculativeContacts() const;
    const std::unordered_map<size_t, Contact>& getContactCache() const;
    std::vector<ExternalMotionContact>& getExternalMotionContacts();

    const std::vector<RigidBodyHandle>& getAwakeList() const;
    const BVHTree& getDynamicAwakeBvh() const;
    const BVHTree& getDynamicAsleepBvh() const;
    const BVHTree& getStaticBvh() const;
    const TerrainBVH& getTerrainBvh() const;

    // Temporary legacy API.
    PhysicsWorld* getPhysicsWorld();

    void syncBodyFromTransform(RigidBodyHandle body);
    void queueAdd(RigidBodyHandle& handle, BroadphaseBucket& target);
    void queueRemove(RigidBodyHandle& handle);
    void queueMove(RigidBodyHandle& handle, BroadphaseBucket& target);
    void setBVHDirty(RigidBodyHandle& handle);


    [[nodiscard]] RigidBodyHandle createRigidBody(const RigidBodyDesc& desc);
    bool destroyRigidBody(RigidBodyHandle body);

    [[nodiscard]] ColliderHandle createCollider(RigidBodyHandle body, const ColliderDesc& desc);
    bool destroyCollider(ColliderHandle collider);

    std::optional<RigidBodyState> getRigidBodyState(RigidBodyHandle body) const;
    std::optional<ColliderState> getColliderState(ColliderHandle collider) const;

    bool applyLinearImpulse(RigidBodyHandle body, const glm::vec3& impulse);
    bool setLinearVelocity(RigidBodyHandle body, const glm::vec3& velocity);
    bool setKinematicTarget(RigidBodyHandle body, const PhysicsPose& target);

    bool setColliderLocalPose(ColliderHandle collider, const PhysicsPose& localPose);
    bool setColliderEnabled(ColliderHandle collider, bool enabled);
    bool setColliderTrigger(ColliderHandle collider, bool isTrigger);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};