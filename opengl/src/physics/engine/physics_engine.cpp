#include "pch.h"

#include <algorithm>

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=====================================
// Construction and ownership
//=====================================
PhysicsEngine::PhysicsEngine() : impl(std::make_unique<Impl>()) {}
PhysicsEngine::~PhysicsEngine() = default;
PhysicsEngine::PhysicsEngine(PhysicsEngine&&) noexcept = default;
PhysicsEngine& PhysicsEngine::operator=(PhysicsEngine&&) noexcept = default;

//=====================================
// Initialization and scene lifetime
//=====================================
void PhysicsEngine::init(World* world, FrameTimers* frameTimers) {
    impl->init(world, frameTimers);
}

void PhysicsEngine::setupScene(std::vector<Tri>* terrainTriangles) {
    impl->setupScene(terrainTriangles);
}

void PhysicsEngine::clear() {
    impl->clear();
}

//=====================================
// Simulation
//=====================================
void PhysicsEngine::prepareStepLoop() {
    impl->prepareStepLoop();
}

void PhysicsEngine::step(float deltaTime, EngineState& engine) {
    impl->step(deltaTime, engine);
}

//=====================================
// Physics queries
//=====================================
Raycast::RaycastHit PhysicsEngine::raycast(Raycast::Ray& ray) {
    return impl->raycast(ray);
}

//=====================================
// Scene-wide commands
//=====================================
void PhysicsEngine::sleepAllObjects() {
    impl->sleepAllObjects();
}

void PhysicsEngine::awakenAllObjects() {
    impl->awakenAllObjects();
}

//=====================================
// Solver configuration
//=====================================
int PhysicsEngine::getPgsIterations() const {
    return impl->pgsIterations;
}

void PhysicsEngine::setPgsIterations(int iterations) {
    impl->pgsIterations = (std::max)(1, iterations);
}

//=====================================
// Simulation output
//=====================================
std::vector<ExternalMotionContact>& PhysicsEngine::getExternalMotionContacts() {
    return impl->getExternalMotionContacts();
}

//=====================================
// Debug state
//=====================================
DebugData PhysicsEngine::getDebugData() const {
    return impl->getDebugData();
}

PhysicsStepDebugPhase PhysicsEngine::getDebugPhase() const {
    return impl->debugPhase;
}

float PhysicsEngine::getPausedDt() const {
    return impl->pausedDt;
}

//=====================================
// Debug visualization
//=====================================
void PhysicsEngine::updateBVHRenderData(const BVHType& type, bool update) {
    impl->updateBVHRenderData(type, update);
}

const std::vector<AABB>& PhysicsEngine::getDebugSweeps() const {
    return impl->debugSweeps;
}

const std::vector<DebugSpeculativeContact>& PhysicsEngine::getDebugSpeculativeContacts() const {
    return impl->debugSpeculativeContacts;
}

const std::unordered_map<size_t, Contact>& PhysicsEngine::getContactCache() const {
    return impl->getContactCache();
}

//=====================================
// Debug spatial data
//=====================================
const std::vector<RigidBodyHandle>& PhysicsEngine::getAwakeList() const {
    return impl->getAwakeList();
}

const BVHTree& PhysicsEngine::getDynamicAwakeBvh() const {
    return impl->getDynamicAwakeBvh();
}

const BVHTree& PhysicsEngine::getDynamicAsleepBvh() const {
    return impl->getDynamicAsleepBvh();
}

const BVHTree& PhysicsEngine::getStaticBvh() const {
    return impl->getStaticBvh();
}

const TerrainBVH& PhysicsEngine::getTerrainBvh() const {
    return impl->getTerrainBvh();
}

//=====================================
// Temporary legacy API
//=====================================
PhysicsWorld* PhysicsEngine::getPhysicsWorld() {
    return impl->getPhysicsWorld();
}

void PhysicsEngine::syncBodyFromTransform(RigidBodyHandle body) {
    impl->syncBodyFromTransform(body);
}

void PhysicsEngine::setBVHDirty(RigidBodyHandle& handle) {
    impl->setBVHDirty(handle);
}
