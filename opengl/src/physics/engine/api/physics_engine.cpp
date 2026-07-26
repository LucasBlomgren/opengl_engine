#include "pch.h"

#include <algorithm>

#include "physics/public/physics_engine.h"
#include "physics/engine/physics_engine_impl.h"

//=====================================
// Public facade: construction and ownership
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
// Solver configuration
//=====================================
int PhysicsEngine::getPgsIterations() const {
    return impl->pgsIterations;
}

void PhysicsEngine::setPgsIterations(int iterations) {
    impl->pgsIterations = (std::max)(1, iterations);
}

