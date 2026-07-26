#include "pch.h"

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
// Facade: Initialization and scene lifetime
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
// Facade: Simulation
//=====================================
void PhysicsEngine::prepareStepLoop() {
    impl->prepareStepLoop();
}
void PhysicsEngine::step(float deltaTime, EngineState& engine) {
    impl->step(deltaTime, engine);
}

//=====================================
// Facade: Scene-wide commands
//=====================================
void PhysicsEngine::sleepAllObjects() {
    impl->submitSleepAllObjects();
}

void PhysicsEngine::awakenAllObjects() {
    impl->submitAwakenAllObjects();
}

//=====================================
// Facade: Solver configuration
//=====================================
int PhysicsEngine::getPgsIterations() const {
    return impl->getPgsIterations();
}

void PhysicsEngine::setPgsIterations(int iterations) {
    impl->setPgsIterations(iterations);
}
